import rclpy
from rclpy.node import Node
import requests
from typing import Dict

from dt_robot_utils import get_robot_configuration
from duckietown_msgs.msg import DisplayFragment as DisplayFragmentMsg

from display_renderer import (
    PAGE_ROBOT_INFO,
    REGION_BODY,
    DisplayROI,
    TextFragmentRenderer,
)

from duckietown.dtros import DTROS, NodeType, TopicType

class RobotInfoRendererNode(Node):
    def __init__(self):
        super(RobotInfoRendererNode, self).__init__("robot_info_renderer_node")
        # get parameters
        self._veh = self.declare_parameter("veh").get_parameter_value().string_value
        # create publisher
        self._pub = self.create_publisher(
            DisplayFragmentMsg,
            "fragments",
            1
        )
        # create renderers
        renderer = RobotInfoRenderer()

        # fetch info from Duckiebot health-API
        health_api_url = f"http://{self._veh}.local/health/"
        # just need to get data from health API once
        health_data = None
        while rclpy.ok():
            try:
                health_data = requests.get(health_api_url).json()
                break
            except BaseException:
                # wait for health API to become available
                rclpy.sleep(5)
                continue
        self.get_logger().info("Health API data fetch successful.")

        # format texts for the display
        text = self._fmt({
            "Name": self._veh,
            "Model": get_robot_configuration().name,
            "Firmware": f"v{health_data['software']['version']}",
        })
        renderer.update(text)

        # this information doesn't need to be updated, so publishing once and latching is enough
        self._pub.publish(renderer.as_msg())
        self.get_logger().info("Robot Info Page published.")

        # keep node alive
        rclpy.spin(self)

    def _shorten_str(self, input, max_length):
        """If the input is longer than the allowed, it's trimmed and '...' is added"""

        # at least there should be enough space for "..."
        assert max_length >= 3, "Not enough space for displaying the information"

        output = input
        if len(input) > max_length:
            output = input[:(max_length - 3)] + "..."
        return output

    def _fmt(self, disp_data: Dict[str, str]) -> str:
        # number of characters per line
        # this is counted on the screen for reading comfort, parameterize only if really necessary
        DISPLAY_CHAR_TOTAL = 21

        output = ""
        for key, value_raw in disp_data.items():
            value = self._shorten_str(value_raw, DISPLAY_CHAR_TOTAL - len(key) - 1)
            space_len = DISPLAY_CHAR_TOTAL - len(key) - len(value)
            space = ' ' * space_len
            output += f"{key}{space}{value}\n"
        return output


class RobotInfoRenderer(TextFragmentRenderer):
    def __init__(self):
        super(RobotInfoRenderer, self).__init__(
            name=f"__robot_info__",
            page=PAGE_ROBOT_INFO,
            region=REGION_BODY,
            roi=DisplayROI(0, 0, REGION_BODY.width, REGION_BODY.height),
            scale="fill",
            ttl=-1,
        )


def main(args=None):
    rclpy.init(args=args)

    node = RobotInfoRendererNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()