import rclpy
from flask import Blueprint
from std_msgs.msg import Bool
from dt_robot_rest_api.utils import response_ok
from rclpy.node import Node

estop_bp = Blueprint("estop", __name__)

class EstopNode(Node):
    def __init__(self):
        super().__init__('estop_node')
        self._estop_value = False
        self._pub = self.create_publisher(Bool, "~estop", 1)
        self._sub = self.create_subscription(Bool, "~estop", self._estop_cb, 1)

    def _estop_cb(self, msg):
        self._estop_value = msg.data

    def _estop(self, value: bool):
        self._pub.publish(Bool(data=value))
        return response_ok({})

    @estop_bp.route("/estop/on")
    def _estop_on(self):
        # return current API car_estop
        return self._estop(True)

    @estop_bp.route("/estop/status")
    def _estop_status(self):
        # return current API car_estop
        return response_ok({"engaged": self._estop_value})

    @estop_bp.route("/estop/off")
    def _estop_off(self):
        # return current API car_estop
        return self._estop(False)

def main(args=None):
    rclpy.init(args=args)

    node = EstopNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()