#!/usr/bin/env python3

import copy
import rclpy
from rclpy.node import Node
import numpy as np

import time

from PIL import Image
from typing import Any, Iterable
from threading import Semaphore
import cv_bridge

from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306

from duckietown_msgs.msg import DisplayFragment as DisplayFragmentMsg
from duckietown_msgs.msg import ButtonEvent as ButtonEventMsg

from dt_class_utils import DTReminder

from display_renderer import (
    REGION_FULL,
    REGION_HEADER,
    REGION_BODY,
    REGION_FOOTER,
    ALL_PAGES,
    PAGE_HOME,
    PAGE_SHUTDOWN,
    PAGE_TEST_OLED_DISPLAY,
    AbsDisplayFragmentRenderer,
    DisplayROI,
    DisplayFragment,
)

from hardware_test_oled_display import HardwareTestOledDisplay


class DisplayNode(Node):
    _REGIONS = {
        DisplayFragmentMsg.REGION_FULL: REGION_FULL,
        DisplayFragmentMsg.REGION_HEADER: REGION_HEADER,
        DisplayFragmentMsg.REGION_BODY: REGION_BODY,
        DisplayFragmentMsg.REGION_FOOTER: REGION_FOOTER,
    }
    _MAX_FREQUENCY_HZ = 5

    def __init__(self):
        super(DisplayNode, self).__init__(node_name="display_driver_node")
        # get parameters
        self.declare_parameter('veh', 'example_robot')
        self._veh = self.get_parameter("veh").get_parameter_value().string_value
        self.declare_parameter('bus', 1)
        self._i2c_bus = self.get_parameter("bus").get_parameter_value().integer_value
        self.declare_parameter('address', 0x3C)
        self._i2c_address = self.get_parameter("address").get_parameter_value().integer_value
        self.declare_parameter('frequency', 1)
        self._frequency = self.get_parameter("frequency").get_parameter_value().integer_value

        # create a display handler
        serial = i2c(port=self._i2c_bus, address=self._i2c_address)
        self._display = ssd1306(serial)
        # page selector
        self._page = PAGE_HOME
        self._pages = {PAGE_HOME}
        # create buffers
        self._fragments = {k: dict() for k in self._REGIONS}
        self._buffer = np.zeros(
            (
                self._REGIONS[DisplayFragmentMsg.REGION_FULL].height,
                self._REGIONS[DisplayFragmentMsg.REGION_FULL].width,
            ),
            dtype=np.uint8,
        )
        self._fragments_lock = Semaphore(1)
        self._device_lock = Semaphore(1)
        # create subscribers
        self._fragments_sub = self.create_subscription(
            DisplayFragmentMsg,
            "fragments",
            self._fragment_cb,
            10
        )
        self._button_sub = self.create_subscription(
            ButtonEventMsg,
            "button",
            self._button_event_cb,
            1
        )
        # create internal renderers
        self._pager_renderer = PagerFragmentRenderer()
        # create rendering loop
        self._timer = self.create_timer(1.0 / self._frequency, self._render)
        self._reminder = DTReminder(frequency=self._MAX_FREQUENCY_HZ)

        # user hardware test
        self._hardware_test = HardwareTestOledDisplay(
            fn_show_test_display=self.show_test_page,
            fn_remove_test_display=self.hide_test_page,
        )
        self._is_performing_test = False

    def show_test_page(self, test_page_msg):
        """
        Used in the user hardware test: show the test display and change to it
        """
        self._is_performing_test = True
        self._fragment_cb(test_page_msg, is_test_cmd=True)
        with self._fragments_lock:
            self._page = PAGE_TEST_OLED_DISPLAY

    def hide_test_page(self):
        """
        Used in the user hardware test: remove the test display; go to homepage
        """
        with self._fragments_lock:
            self._page = PAGE_HOME
        self._is_performing_test = False

    def _button_event_cb(self, msg: Any):
        # when performing hardware test, prevent default handling
        if self._is_performing_test:
            return

        if msg.event == ButtonEventMsg.EVENT_SINGLE_CLICK:
            with self._fragments_lock:
                pages = sorted(self._pages)
                # move to the next page
                try:
                    i = pages.index(self._page)
                    i = pages[(i + 1) % len(pages)]
                except ValueError:
                    i = 0
                # select page
                self._page = i

        if msg.event == ButtonEventMsg.EVENT_HELD_3SEC or msg.event == ButtonEventMsg.EVENT_HELD_10SEC:
            with self._fragments_lock:
                self._page = PAGE_SHUTDOWN
            self._render(None)

    def _fragment_cb(self, msg: Any, is_test_cmd: bool = False):
        # when performing hardware test, prevent default handling
        if self._is_performing_test and not is_test_cmd:
            return

        region = self._REGIONS[msg.region]
        # convert image to greyscale
        # img = imgmsg_to_mono8(msg.data)
        img = cv_bridge.CvBridge().imgmsg_to_cv2(msg.data, desired_encoding="mono8")
        # parse ROI
        roi = DisplayROI.from_sensor_msgs_ROI(msg.location)
        if roi is None:
            # no ROI was specified
            # the fragment will be used as is if it fits the region, resized otherwise
            rh, rw = region.height, region.width
            fh, fw = img.shape
            # does it fit?
            if fw > rw or fh > rh:
                img = Image.fromarray(img)
                img = img.resize((rw, rh), resample=Image.NEAREST)
                img = np.array(img)
                fh, fw = rh, rw
            roi = DisplayROI(x=0, y=0, w=fw, h=fh)
        else:
            # validate ROI
            fh, fw = img.shape
            # - find biggest offsets achievable
            fx, fy = min(region.width, roi.x), min(region.height, roi.y)
            # - find biggest canvas size achievable
            cw, ch = min(region.width - fx, roi.w), min(region.height - fy, roi.h)
            # does it fit?
            if fw > cw or fh > ch:
                img = Image.fromarray(img)
                img = img.resize((cw, ch), resample=Image.NEAREST)
                img = np.array(img)
                fh, fw = ch, cw
            # update ROI
            roi = DisplayROI(x=fx, y=fy, w=fw, h=fh)

        # threshold image at mid-range
        img = (img > 125).astype(np.uint8) * 255
        # list fragment for rendering
        with self._fragments_lock:
            self._fragments[msg.region][msg.id] = DisplayFragment(
                data=img, roi=roi, page=msg.page, z=msg.z, _ttl=msg.ttl, _time=self.get_clock().now().to_msg()
            )
        # force refresh if this fragment is on the current page
        if msg.page == self._page:
            self._render(None)

    def _render(self, _=None):
        # use a reminder object to control the maximum frequency
        if not self._reminder.is_time():
            return
        # make sure we are not shutdown
        if not rclpy.ok():
            return
        # ---
        with self._fragments_lock:
            # clean pages
            self._pages = {PAGE_HOME}
            # remove expired fragments and annotate how many pages we need
            for region, fragments in self._fragments.items():
                for fragment_id in copy.copy(set(fragments.keys())):
                    fragment = fragments[fragment_id]
                    if fragment.ttl() <= 0:
                        self.get_logger().debug(
                            f"Fragment `{fragment_id}` on page `{fragment.page}`, "
                            f"region `{region}` w/ TTL `{fragment.given_ttl}` "
                            f"expired, remove!"
                        )
                        del self._fragments[region][fragment_id]
                    if fragment.page != ALL_PAGES:
                        self._pages.add(fragment.page)
            # sanitize page selector
            if self._page not in self._pages:
                # go back to home
                self._page = PAGE_HOME
            # filter fragments by page
            data = {
                region: [
                    fragment for fragment in fragments.values() if fragment.page in [ALL_PAGES, self._page]
                ]
                for region, fragments in self._fragments.items()
            }
        # add pager fragment
        self._pager_renderer.update(self._pages, self._page)
        if self._pager_renderer.page in [ALL_PAGES, self._page]:
            data[self._pager_renderer.region.id].append(self._pager_renderer.as_fragment())
        # sort fragments by z-index
        data = {region: sorted(fragments, key=lambda f: f.z) for region, fragments in data.items()}
        # clear buffer
        self._buffer.fill(0)
        # render fragments
        for region_id, fragments in data.items():
            region = self._REGIONS[region_id]
            # render fragments
            for fragment in fragments:
                fx, fy = fragment.roi.x, fragment.roi.y
                fw, fh = fragment.roi.w, fragment.roi.h
                # move fragment to the right region
                fx += region.x
                fy += region.y
                # update buffer
                self._buffer[fy : fy + fh, fx : fx + fw] = fragment.data
        # convert buffer to 1-byte pixel
        buf = Image.fromarray(self._buffer, mode="L")
        # convert 1-byte pixel to 1-bit pixel
        buf = buf.convert(mode="1")
        # display buffer
        with self._device_lock:
            try:
                self._display.display(buf)
            except BlockingIOError:
                pass

    def on_shutdown(self):
        self.get_logger().info("Clearing buffer...")
        # stop rendering job
        self._timer.cancel()
        self._timer.destroy()
        # clear buffer
        self._buffer.fill(0)
        # render nothing
        buf = Image.fromarray(self._buffer, mode="1")
        self.get_logger().info("Clearing display...")
        with self._device_lock:
            try:
                self._display.display(buf)
            except BlockingIOError:
                pass


class PagerFragmentRenderer(AbsDisplayFragmentRenderer):

    SPACING_PX = 7
    UNSELECTED_PAGE_ICON = np.array(
        [
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 255, 255, 255, 0],
        ]
    )
    SELECTED_PAGE_ICON = np.array(
        [[0, 0, 0, 0, 0], [255, 255, 255, 255, 255], [255, 255, 255, 255, 255], [255, 255, 255, 255, 255]]
    )

    def __init__(self):
        super(PagerFragmentRenderer, self).__init__(
            "pager",
            page=ALL_PAGES,
            region=REGION_FOOTER,
            roi=DisplayROI(0, 0, REGION_FOOTER.width, REGION_FOOTER.height),
        )
        self._pages = {0}
        self._selected = 0

    def as_fragment(self):
        return DisplayFragment(
            data=self.buffer,
            roi=self.roi,
            page=self._page,
            z=self._z,
            _ttl=self._ttl,
            # _time=self.get_clock().now().nanoseconds / 1e9
            _time=time.time_ns() / 1e9,
        )

    def update(self, pages: Iterable[int], page: int):
        self._pages = pages
        self._selected = page if page in pages else 0

    def _render(self):
        # clear buffer
        self._buffer.fill(0)
        # render dots
        ch, cw = self.shape
        _, sw = self.SELECTED_PAGE_ICON.shape
        num_pages = len(self._pages)
        expected_w = num_pages * sw + (num_pages - 1) * self.SPACING_PX
        offset = int(np.floor((cw - expected_w) * 0.5))
        for page in sorted(self._pages):
            icon = self.SELECTED_PAGE_ICON if page == self._selected else self.UNSELECTED_PAGE_ICON
            self._buffer[:, offset : offset + sw] = icon
            offset += sw + self.SPACING_PX


if __name__ == "__main__":
    rclpy.init()
    node = DisplayNode()
    rclpy.spin(node)
    rclpy.shutdown()
