#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.timer import Timer
from rclpy.clock import ROSClock

from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped
from wheels_driver.dagu_wheels_driver import DaguWheelsDriver

from hardware_test_wheels import HardwareTestMotor, HardwareTestMotorSide


class WheelsDriverNode(Node):
    def __init__(self):
        super().__init__('wheels_driver_node')

        self.estop = False

        self.driver = DaguWheelsDriver()

        self.msg_wheels_cmd = WheelsCmdStamped()

        self.pub_wheels_cmd = self.create_publisher(
            WheelsCmdStamped,
            "wheels_cmd_executed",
            1
        )

        self.sub_topic = self.create_subscription(
            WheelsCmdStamped,
            "wheels_cmd",
            self.wheels_cmd_cb,
            1
        )
        self.sub_e_stop = self.create_subscription(
            BoolStamped,
            "emergency_stop",
            self.estop_cb,
            1
        )

        self._hardware_test_left = HardwareTestMotor(HardwareTestMotorSide.LEFT, self.driver)
        self._hardware_test_right = HardwareTestMotor(HardwareTestMotorSide.RIGHT, self.driver)

        self.get_logger().info("Initialized.")

    def wheels_cmd_cb(self, msg):
        if self.estop:
            vel_left = 0.0
            vel_right = 0.0
        else:
            vel_left = msg.vel_left
            vel_right = msg.vel_right

        self.driver.set_wheels_speed(left=vel_left, right=vel_right)
        self.msg_wheels_cmd.header = msg.header
        self.msg_wheels_cmd.header.stamp = self.get_clock().now().to_msg()
        self.msg_wheels_cmd.vel_left = vel_left
        self.msg_wheels_cmd.vel_right = vel_right
        self.pub_wheels_cmd.publish(self.msg_wheels_cmd)

    def estop_cb(self, msg):
        self.estop = msg.data
        if self.estop:
            self.get_logger().info("Emergency Stop Activated")
        else:
            self.get_logger().info("Emergency Stop Released")

    def on_shutdown(self):
        self.driver.set_wheels_speed(left=0.0, right=0.0)


def main(args=None):
    rclpy.init(args=args)

    node = WheelsDriverNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()