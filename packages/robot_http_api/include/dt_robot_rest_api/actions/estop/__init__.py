import rclpy
import threading
from flask import Blueprint
from std_msgs.msg import Bool
from dt_robot_rest_api.utils import response_ok
from rclpy.node import Node

estop_bp = Blueprint("estop", __name__)

class EstopNode(Node):
    is_spinning = None
    def __init__(self):
        super().__init__('estop_node')
        self._estop_value = False
        if self.is_spinning is not None:
            return
        self.is_spinning = False
        self._pub = self.create_publisher(Bool, "estop", 1)
        self._sub = self.create_subscription(Bool, "estop", self._estop_cb, 1)

    def _estop_cb(self, msg):
        self._estop_value = msg.data

    def _estop(self, value: bool):
        self._pub.publish(Bool(data=value))
        return response_ok({})

    def start_spinning(self):
        if not self.is_spinning:
            self.is_spinning = True
            rclpy.spin(self)
            self.is_spinning = False

if EstopNode.is_spinning is None:
    node = EstopNode()

@estop_bp.route("/estop/on")
def _estop_on():
    # return current API car_estop
    global node
    return node._estop(True)

@estop_bp.route("/estop/status")
def _estop_status():
    global node
    # return current API car_estop
    return response_ok({"engaged": node._estop_value})

@estop_bp.route("/estop/off")
def _estop_off():
    global node
    # return current API car_estop
    return node._estop(False)
