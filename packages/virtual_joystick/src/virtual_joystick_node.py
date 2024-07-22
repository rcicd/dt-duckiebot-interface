#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Header
from duckietown_msgs.msg import WheelsCmdStamped


class VirtualJoystick(Node):
    def __init__(self, node_name: str = "virtual_joystick_node"):
        super().__init__(node_name)
        self.declare_parameter("veh", "")
        self.vehicle = self.get_parameter("veh").get_parameter_value().string_value

        self.joystick_subscriber = self.create_subscription(Joy, f"/{self.vehicle}/virtual_joystick", self.callback, 1)
        self.wheels_publisher = self.create_publisher(WheelsCmdStamped, f"/{self.vehicle}/wheels_cmd", 1)

    def callback(self, msg: Joy):
        if not msg.buttons[4] == ord('s'):
            return
        wheels_cmd = WheelsCmdStamped()
        wheels_cmd.header = Header(stamp=msg.header.stamp)

        scale = msg.axes[0]

        left, right = 0, 0

        turns = msg.buttons[1] + msg.buttons[3]
        straight = msg.buttons[0] + msg.buttons[2]

        if turns % 2 == 0 and straight % 2 == 0:
            wheels_cmd.vel_left = 0.0
            wheels_cmd.vel_right = 0.0
            self.wheels_publisher.publish(wheels_cmd)
            return
        elif turns % 2 == 0:
            left += scale * (msg.buttons[0] - msg.buttons[2])
            right += scale * (msg.buttons[0] - msg.buttons[2])
        elif straight % 2 == 0:
            left += scale * (msg.buttons[1] / 2 - msg.buttons[3] / 2)
            right += scale * (-msg.buttons[1] / 2 + msg.buttons[3] / 2)
        else:
            left_value = (msg.buttons[0] - msg.buttons[2] + msg.buttons[1] / 2 - msg.buttons[3] / 2)
            right_value = (msg.buttons[0] - msg.buttons[2] - msg.buttons[1] / 2 + msg.buttons[3] / 2)

            left += scale * (left_value / abs(left_value) if abs(left_value) > 1 else 0)
            right += scale * (right_value / abs(right_value) if abs(right_value) > 1 else 0)
        wheels_cmd.vel_left = left
        wheels_cmd.vel_right = right
        self.wheels_publisher.publish(wheels_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = VirtualJoystick()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()