import rclpy
from rclpy.node import Node
from rclpy.timer import Rate
from std_srvs.srv import SetBool
from sensor_msgs.msg import Range
from std_msgs.msg import Header

from dt_duckiebot_msgs.msg import LEDPattern
from dt_duckiebot_msgs.srv import SetCustomLEDPattern

from dt_class_utils import DTReminder
from dt_vl53l0x import VL53L0X
from display_renderer import (
    DisplayROI,
    PAGE_TOF,
    REGION_BODY,
    MonoImageFragmentRenderer,
)


class TrafficLightNode(Node):
    def __init__(self):
        super(TrafficLightNode, self).__init__("traffic_light_node")

        # Import protocols
        self._number_leds = self.get_parameter("number_leds").get_parameter_value().integer_value
        self._activation_order = self.get_parameter("activation_order").get_parameter_value().integer_array_value
        self._green_time = self.get_parameter("green_time").get_parameter_value().double_value
        self._all_red_time = self.get_parameter("all_red_time").get_parameter_value().double_value
        self._frequency = self.get_parameter("frequency").get_parameter_value().double_value

        self.green_idx = 0

        cycle_duration = self._green_time + self._all_red_time

        # Create the color mask
        self.color_mask = [0] * 5
        self.color_mask[0 : self._number_leds] = [1] * self._number_leds

        # Function mapping to LEDEmitterNode's `set_custom_pattern` service
        self.changePattern = self.create_client(
            SetCustomLEDPattern,
            "led_emitter_node/set_custom_pattern"
        )

        # Start a timer that will regularly call a method that changes
        # the direction that get green light
        self.traffic_cycle = self.create_timer(cycle_duration, self.change_direction)

        self.get_logger().info("Initialized.")

    def change_direction(self):
        # Move to next light in list
        self.green_idx = (self.green_idx + 1) % self._number_leds
        green_LED = self._activation_order[self.green_idx]

        # Only blink the green LED
        frequency_mask = [0] * 5
        frequency_mask[green_LED] = 1

        # Create Protocol (we fake 5 LEDs, but the last will not be updated)
        color_list = ["red"] * 5
        color_list[green_LED] = "green"

        # Build message
        pattern_msg = LEDPattern()
        pattern_msg.color_list = self.to_led_order(color_list)
        pattern_msg.color_mask = self.color_mask
        pattern_msg.frequency = self._frequency
        pattern_msg.frequency_mask = self.to_led_order(frequency_mask)

        self.changePattern.call_async(pattern_msg)

        # Keep the green light on
        rclpy.sleep(self._green_time)

        # Turn all on red for safety
        pattern_msg.color_list = ["red"] * 5
        pattern_msg.frequency = 0
        self.changePattern.call_async(pattern_msg)

    @staticmethod
    def to_led_order(unordered_list):
        """Change ordering from successive (0,1,2,3,4) to the one expected by the led emitter (0,4,1,3,2)

        Args:
            unordered_list (:obj:`list`): List to be ordered.
        Returns:
            :obj: `list`: Permutated list of length ~number_leds.
        """
        ordering = [0, 4, 1, 3, 2]
        ordered_list = [unordered_list[i] for i in ordering]
        return ordered_list


def main(args=None):
    rclpy.init(args=args)

    node = TrafficLightNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()