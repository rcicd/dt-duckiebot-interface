import os
import rclpy
import netifaces
from PIL import Image, ImageOps
from duckietown_msgs.msg import DisplayFragment as DisplayFragmentMsg

from display_renderer import REGION_HEADER, DisplayROI, AbsDisplayFragmentRenderer, Z_SYSTEM, ALL_PAGES

from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown.utils.image.pil import pil_to_np


class NetworkingDisplayRendererNode(DTROS):
    def __init__(self):
        super(NetworkingDisplayRendererNode, self).__init__("networking_renderer_node")
        # get parameters
        self._veh = self.declare_parameter("veh").get_parameter_value().string_value
        self._assets_dir = self.declare_parameter("assets_dir").get_parameter_value().string_value
        self._frequency = self.declare_parameter("frequency").get_parameter_value().double_value
        # create publisher
        self._pub = self.create_publisher(
            DisplayFragmentMsg,
            "fragments",
            1
        )
        # create renderers
        self._wlan0_indicator = NetIFaceFragmentRenderer(self._assets_dir, "wlan0", DisplayROI(0, 0, 11, 16))
        self._eth0_indicator = NetIFaceFragmentRenderer(self._assets_dir, "eth0", DisplayROI(14, 0, 11, 16))
        self._renderers = [self._wlan0_indicator, self._eth0_indicator]
        # create loop
        self._timer = self.create_timer(1.0 / self._frequency, self._publish)

    def _publish(self):
        for renderer in self._renderers:
            msg = renderer.as_msg()
            self._pub.publish(msg)


class NetIFaceFragmentRenderer(AbsDisplayFragmentRenderer):
    def __init__(self, assets_dir: str, iface: str, roi: DisplayROI):
        super(NetIFaceFragmentRenderer, self).__init__(
            f"__iface_connection_{iface}__", page=ALL_PAGES, region=REGION_HEADER, roi=roi, z=Z_SYSTEM, ttl=30
        )
        self._assets_dir = assets_dir
        self._iface = iface
        # load assets
        _asset_path = lambda a: os.path.join(self._assets_dir, "icons", f"{a}.png")
        self._assets = {
            asset: pil_to_np(ImageOps.grayscale(Image.open(_asset_path(asset))))
            for asset in [f"{self._iface}_connected", f"{self._iface}_not_connected"]
        }

    def _render(self):
        # fetch info about iface
        try:
            iface_addrs = netifaces.ifaddresses(self._iface)
            connected = netifaces.AF_INET in iface_addrs
        except ValueError:
            connected = False
        icon = self._iface + ("" if connected else "_not") + "_connected"
        self._buffer[:, :] = self._assets[icon]


def main(args=None):
    rclpy.init(args=args)

    node = NetworkingDisplayRendererNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()