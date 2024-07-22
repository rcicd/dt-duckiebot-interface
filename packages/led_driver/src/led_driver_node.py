#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import ParameterType
from duckietown_msgs.msg import LEDPattern
from rgb_led import RGB_LED


class LEDDriverNode(Node):
    """Node for controlling LEDs.

    Calls the low-level functions of class :obj:`RGB_LED` that creates the PWM
    signal used to change the color of the LEDs. The desired behavior is specified by
    the LED index (Duckiebots and watchtowers have multiple of these) and a pattern.
    A pattern is a combination of colors and blinking frequency.

    Duckiebots have 5 LEDs that are indexed and positioned as following:

        +------------------+------------------------------------------+
        | Index            | Position (rel. to direction of movement) |
        +==================+==========================================+
        | 0                | Front left                               |
        +------------------+------------------------------------------+
        | 1                | Rear left                                |
        +------------------+------------------------------------------+
        | 2                | Top / Front middle  (DB1X models only)   |
        +------------------+------------------------------------------+
        | 3                | Rear right                               |
        +------------------+------------------------------------------+
        | 4                | Front right                              |
        +------------------+------------------------------------------+

    """

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super().__init__(
            node_name=node_name
        )
        # load params
        self.declare_parameter("idle.color", [""])
        self.declare_parameter("idle.intensity", [0.0])
        self.declare_parameter("power_off.color", [""] * 5)
        self.declare_parameter("power_off.intensity", [0.0] * 5)
        _idle_color_string = self.get_parameter("idle.color").value
        _power_off_color_string = self.get_parameter("power_off.color").value
        self._idle_intensity = self.get_parameter("idle.intensity").value
        self._power_off_intensity = self.get_parameter("power_off.intensity").value
        self._idle_color = [[int(value) for value in color.split(" ")] for color in _idle_color_string]
        self._power_off_color = [[int(value) for value in color.split(" ")] for color in _power_off_color_string]
        # initialize LED library
        self.led = RGB_LED()
        # turn OFF the LEDs
        for i in range(5):
            self.led.set_RGB(i, self._idle_color[i], self._idle_intensity[i])
        # subscribers
        self.sub_topic = self.create_subscription(
            LEDPattern,
            "led_pattern",
            self.led_cb,
            10
        )

        self.get_logger().info("Initialized.")

    def led_cb(self, msg):
        """Switches the LEDs to the requested signal."""
        for i in range(5):
            colors = (msg.rgb_vals[i].r, msg.rgb_vals[i].g, msg.rgb_vals[i].b)
            self.led.set_RGB(i, colors, msg.rgb_vals[i].a)

    def on_shutdown(self):
        """Shutdown procedure.

        At shutdown, changes the LED pattern to `LIGHT_OFF`.
        """
        for i in range(5):
            self.led.set_RGB(i, self._power_off_color[i], self._power_off_intensity[i])
        # Turn off the lights when the node dies
        self.get_logger().info("Shutting down. Turning LEDs off.")
        time.sleep(1)


def main(args=None):
    rclpy.init(args=args)

    # Create the LEDdriverNode object
    led_driver_node = LEDDriverNode(node_name="led_driver_node")

    # Keep it spinning to keep the node alive
    rclpy.spin(led_driver_node)

    led_driver_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
