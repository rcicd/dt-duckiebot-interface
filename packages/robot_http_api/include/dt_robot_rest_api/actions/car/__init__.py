import math
import time

import threading
import rclpy
from flask import Blueprint
from duckietown_msgs.msg import WheelsCmdStamped
from dt_robot_rest_api.utils import response_ok
from rclpy.node import Node

car_bp = Blueprint("car", __name__)

class CarStatusNode(Node):
    is_spinning = None
    def __init__(self):
        super().__init__('car_status_node')
        self._last_moving_msg_time = 0
        self._last_moving_msg_time_thr = 1
        if self.is_spinning is not None:
            return
        self.is_spinning = False
        self._sub = self.create_subscription(WheelsCmdStamped, "wheels_cmd", self._wheels_cmd_cb, 1)

    def _wheels_cmd_cb(self, msg):
        _car_is_moving = math.fabs(msg.vel_left) + math.fabs(msg.vel_right) > 0
        if _car_is_moving:
            self._last_moving_msg_time = time.time()

    def start_spinning(self):
        if not self.is_spinning:
            self.is_spinning = True
            rclpy.spin(self)
            self.is_spinning = False


if CarStatusNode.is_spinning is None:
    node = CarStatusNode()

@car_bp.route("/car/status")
def _carstatus():
    global node
    # return current API car_car
    return response_ok(
        {
            # this means that the robot is considered NOT moving after
            # at least `_last_moving_msg_time_thr` sec with no non-zero
            # velocity messages received
            "engaged": (time.time() - node._last_moving_msg_time)
                       < node._last_moving_msg_time_thr
        }
    )
