#!/usr/bin/env python3

import sys
import signal

from dt_robot_rest_api import RobotRestAPI

# battery test
import requests
# wifi test
import subprocess
import re
import rclpy
from rclpy.node import Node

from hardware_test_robot_host import HardwareTestWifi, HardwareTestBattery


class RobotRestAPInode(Node):
    is_running = None
    def __init__(self):
        super(RobotRestAPInode, self).__init__(
            node_name="robot_http_api_node"
        )
        if self.is_running is not None:
            return
        self.is_running = False
        # user hardware tests
        self._hardware_test_battery = HardwareTestBattery()
        self._hardware_test_wifi = HardwareTestWifi()

    def spin_node(self):
        if self.is_running:
            return
        self.is_running = True
        api = RobotRestAPI(debug=False)
        api.run(host="0.0.0.0")
        rclpy.spin(self)
        self.is_running = False
        rclpy.shutdown()



def signal_handler(sig, frame):
    sys.exit(0)


if __name__ == "__main__":
    if RobotRestAPInode.is_running is None:
        signal.signal(signal.SIGINT, signal_handler)
        rclpy.init(args=sys.argv)
        ros_node = RobotRestAPInode()
        ros_node.spin_node()
