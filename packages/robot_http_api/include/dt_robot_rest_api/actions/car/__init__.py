import math
import time

import rclpy
from flask import Blueprint
from duckietown_msgs.msg import WheelsCmdStamped
from dt_robot_rest_api.utils import response_ok
from rclpy.node import Node

car_bp = Blueprint("car", __name__)

class CarStatusNode(Node):
    def __init__(self):
        super().__init__('car_status_node')
        self._last_moving_msg_time = 0
        self._last_moving_msg_time_thr = 1
        self._sub = self.create_subscription(WheelsCmdStamped, "~wheels_cmd", self._wheels_cmd_cb, 1)

    def _wheels_cmd_cb(self, msg):
        _car_is_moving = math.fabs(msg.vel_left) + math.fabs(msg.vel_right) > 0
        if _car_is_moving:
            self._last_moving_msg_time = time.time()

    @car_bp.route("/car/status")
    def _carstatus(self):
        # return current API car_car
        return response_ok(
            {
                # this means that the robot is considered NOT moving after
                # at least `_last_moving_msg_time_thr` sec with no non-zero
                # velocity messages received
                "engaged": (time.time() - self._last_moving_msg_time)
                           < self._last_moving_msg_time_thr
            }
        )

def main(args=None):
    rclpy.init(args=args)

    node = CarStatusNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()