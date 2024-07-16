#!/usr/bin/env python3

import dataclasses
import time
from typing import Optional
import yaml

import rclpy
from rclpy.node import Node
from dt_class_utils import DTReminder
from sensor_msgs.msg import Range
from std_msgs.msg import Header
from duckietown_msgs.msg import DisplayFragment as DisplayFragmentMsg

from tof_accuracy import ToFAccuracy
from hardware_test_tof import HardwareTestToF
from dt_vl53l0x.VL53L0X import VL53L0X
from display_renderer import (
    DisplayROI,
    PAGE_TOF,
    REGION_BODY,
    MonoImageFragmentRenderer,
    monospace_screen,
    DisplayFragment
)


class ToFNode(Node):
    def __init__(self):
        super(ToFNode, self).__init__("tof_node")
        self.declare_parameter("veh", "")
        self.declare_parameter("connectors", "")
        self.declare_parameter("sensor_name", "")
        self.declare_parameter("frequency", 10)
        self.declare_parameter("mode", "BETTER")
        self.declare_parameter("display_fragment_frequency", 4)

        self._veh = self.get_parameter("veh").get_parameter_value().string_value
        connectors_string = self.get_parameter("connectors").get_parameter_value().string_value
        self._i2c_connectors = yaml.safe_load(connectors_string)
        self._sensor_name = self.get_parameter("sensor_name").get_parameter_value().string_value
        self._frequency = self.get_parameter("frequency").get_parameter_value().integer_value
        self._mode = self.get_parameter("mode").get_parameter_value().string_value
        self._display_fragment_frequency = self.get_parameter("display_fragment_frequency").get_parameter_value().integer_value
        self._accuracy = ToFAccuracy.from_string(self._mode)
        self.get_logger().info(f"veh: {self._veh}\n"
                               f"connectors: {self._i2c_connectors}\n"
                               f"sensor_name: {self._sensor_name}\n"
                               f"frequency: {self._frequency}\n"
                               f"mode: {self._mode}\n"
                               f"display_fragment_frequency: {self._display_fragment_frequency}\n"
                               f"accuracy: {self._accuracy}\n"
                               )
        # create a VL53L0X sensor handler
        self._sensor: Optional[VL53L0X] = self._find_sensor()
        if not self._sensor:
            conns: str = yaml.safe_dump(self._i2c_connectors, indent=2, sort_keys=True)
            self.get_logger().error(f"No VL53L0X device found. These connectors were tested:\n{conns}\n")
            exit(1)
        # create publisher
        self._display_pub = self.create_publisher(
            DisplayFragmentMsg,
            "fragments",
            1
        )
        self._pub = self.create_publisher(
            Range,
            "range",
            1
        )
        # user hardware test
        self._hardware_test = HardwareTestToF(self._sensor_name, self._accuracy)

        # create screen renderer
        self._renderer = ToFSensorFragmentRenderer(self._sensor_name, self._accuracy)
        # check frequency
        max_frequency = min(self._frequency, int(1.0 / self._accuracy.timing_budget))
        if self._frequency > max_frequency:
            self.get_logger().warn(
                f"Frequency of {self._frequency}Hz not supported. The selected mode "
                f"{self._mode} has a timing budget of {self._accuracy.timing_budget}s, "
                f"which yields a maximum frequency of {max_frequency}Hz."
            )
            self._frequency = max_frequency
        self.get_logger().info(f"Frequency set to {self._frequency}Hz.")
        # create timers
        self.timer = self.create_timer(1.0 / max_frequency, self._timer_cb)
        self._fragment_reminder = DTReminder(frequency=self._display_fragment_frequency)

    def _find_sensor(self) -> Optional[VL53L0X]:
        for connector in self._i2c_connectors:
            conn: str = "[bus:{bus}](0x{address:02X})".format(**connector)
            self.get_logger().info(f"Trying to open device on connector {conn}: connector[\"bus\"]={connector['bus']} and connector[\"address\"]={connector['address']}")
            sensor = VL53L0X(i2c_bus=connector["bus"], i2c_address=connector["address"])
            try:
                self.get_logger().info(f"Opening sensor")
                sensor.open()
            except FileNotFoundError:
                # i2c BUS not found
                self.get_logger().warn(f"No devices found on connector {conn}, the bus does NOT exist")
                continue

            self.get_logger().info(f"Accuracy mode is set to {self._accuracy.mode}")
            sensor.start_ranging(self._accuracy.mode)
            time.sleep(1)
            self.get_logger().info(f"Ranging ended")
            if sensor.get_distance() < 0:
                self.get_logger().warn(f"No devices found on connector {conn}, but the bus exists")
                continue
            self.get_logger().info(f"Device found on connector {conn}")
            return sensor

    def _timer_cb(self):
        # detect range
        distance_mm = self._sensor.get_distance()
        # pack observation into a message
        msg = Range(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id=f"{self._veh}/tof/{self._sensor_name}"),
            radiation_type=Range.INFRARED,
            field_of_view=self._accuracy.fov,
            min_range=self._accuracy.min_range,
            max_range=self._accuracy.max_range,
            range=distance_mm / 1000,
        )
        # publish
        self._pub.publish(msg)
        # publish display rendering (if it is a good time to do so)
        if self._fragment_reminder.is_time():
            self._renderer.update(distance_mm)
            msg = self._renderer.as_msg()
            self._display_pub.publish(msg)

    def on_shutdown(self):
        # noinspection PyBroadException
        try:
            self._sensor.stop_ranging()
        except BaseException:
            pass


class ToFSensorFragmentRenderer(MonoImageFragmentRenderer):
    def __init__(self, name: str, accuracy: ToFAccuracy):
        super(ToFSensorFragmentRenderer, self).__init__(
            f"__tof_{name}__",
            page=PAGE_TOF,
            region=REGION_BODY,
            roi=DisplayROI(0, 0, REGION_BODY.width, REGION_BODY.height),
        )
        self._name = name
        self._accuracy = accuracy
        name = self._name.replace("_", " ").title()
        self._title_h = 12
        self._title = monospace_screen((self._title_h, self.roi.w), f"ToF / {name}:", scale="vfill")

    def update(self, measurement_mm: float):
        pretty_measurement = (
            f" {(measurement_mm / 10):.1f}cm "
            if (measurement_mm / 1000) < self._accuracy.max_range
            else "Out-Of-Range"
        )
        reading = monospace_screen(
            (self.roi.h - self._title_h, self.roi.w), pretty_measurement, scale="hfill", align="center"
        )
        self.data[: self._title_h, :] = self._title
        self.data[self._title_h:, :] = reading


def main(args=None):
    rclpy.init(args=args)

    node = ToFNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()