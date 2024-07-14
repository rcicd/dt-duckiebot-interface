#!/usr/bin/env python3

import os.path
import rclpy
from rclpy.node import Node
from rclpy.timer import Rate
from rclpy.parameter import Parameter
from tf2_ros import TransformBroadcaster
from math import pi

import uuid
import yaml

from std_msgs.msg import Header
from duckietown_msgs.msg import WheelEncoderStamped, WheelsCmdStamped
from wheel_encoder import WheelEncoderDriver, WheelDirection
from hardware_test_wheel_encoder import HardwareTestWheelEncoder

from geometry_msgs.msg import TransformStamped, Transform, Quaternion


class WheelEncoderNode(Node):
    def __init__(self):
        super(WheelEncoderNode, self).__init__('wheel_encoder_node')
        self.declare_parameter("veh", "duckiebot")
        self.declare_parameter("name", "")
        self.declare_parameter("gpio", 0)
        self.declare_parameter("resolution", 0)
        self.declare_parameter("configuration", "")
        self.declare_parameter("publish_frequency", 0.0)
        # get parameters
        self._veh = self.get_parameter("veh").get_parameter_value().string_value
        self._name = self.get_parameter("name").get_parameter_value().string_value
        self._gpio_pin = self.get_parameter("gpio").get_parameter_value().integer_value
        self._resolution = self.get_parameter("resolution").get_parameter_value().integer_value
        self._configuration = self.get_parameter("configuration").get_parameter_value().string_value
        self._publish_frequency = self.get_parameter("publish_frequency").get_parameter_value().double_value

        for param in self._parameters:
            print(f"Parameter {param}")

        print(f"veh: {self._veh}",
                f"name: {self._name}",
                f"gpio: {self._gpio_pin}",
                f"resolution: {self._resolution}",
                f"configuration: {self._configuration}",
                f"publish_frequency: {self._publish_frequency}",
              sep="\n")
        # tick storage
        self._tick = 0
        # publisher for wheel encoder ticks
        self._tick_pub = self.create_publisher(
            WheelEncoderStamped,
            "tick",
            1
        )
        # subscriber for the wheel command executed
        self.sub_wheels = self.create_subscription(
            WheelsCmdStamped,
            "wheels_cmd_executed",
            self._wheels_cmd_executed_cb,
            1
        )
        # tf broadcaster for wheel frame
        self._tf_broadcaster = TransformBroadcaster(self)
        # setup a timer
        self._timer = self.create_timer(1.0 / self._publish_frequency, self._cb_publish)
        # setup the driver
        self._driver = WheelEncoderDriver(self._gpio_pin, self._encoder_tick_cb)
        # user hardware test
        self._hardware_test = HardwareTestWheelEncoder(wheel_side=self._name)

    def _wheels_cmd_executed_cb(self, msg):
        if self._configuration == "left":
            if msg.vel_left >= 0:
                self._driver.set_direction(WheelDirection.FORWARD)
            else:
                self._driver.set_direction(WheelDirection.REVERSE)
        elif self._configuration == "right":
            if msg.vel_right >= 0:
                self._driver.set_direction(WheelDirection.FORWARD)
            else:
                self._driver.set_direction(WheelDirection.REVERSE)

    def _encoder_tick_cb(self, tick_no):
        """
        Callback that receives new ticks from the encoder.

            Args:
                tick_no (int): cumulative total number of ticks
        """
        self._tick = tick_no

    def _cb_publish(self):
        # Create header with timestamp
        header = Header()
        header.frame_id = f"{self._veh}/{self._name}_wheel_axis"
        header.stamp = self.get_clock().now().to_msg()
        # publish WheelEncoderStamped message
        self._tick_pub.publish(
            WheelEncoderStamped(
                header=header,
                data=self._tick,
                resolution=self._resolution,
                type=WheelEncoderStamped.ENCODER_TYPE_INCREMENTAL,
            )
        )
        # publish TF
        angle = (float(self._tick) / float(self._resolution)) * 2 * pi
        q = Quaternion(axis=[0, 1, 0], angle=angle)
        quat = Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)
        self._tf_broadcaster.sendTransform(
            TransformStamped(
                header=header,
                child_frame_id=f"{self._veh}/{self._name}_wheel",
                transform=Transform(rotation=Quaternion(x=quat.x, y=quat.y, z=quat.z, w=quat.w)),
            )
        )


def main(args=None):
    rclpy.init(args=args)

    node = WheelEncoderNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
