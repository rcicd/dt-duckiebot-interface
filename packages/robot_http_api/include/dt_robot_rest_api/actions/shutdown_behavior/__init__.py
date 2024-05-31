import rclpy
import threading
from flask import Blueprint
from std_srvs.srv import Trigger
from dt_robot_rest_api.utils import response_ok, response_error
from rclpy.node import Node

shutdown_behavior_bp = Blueprint("shutdown_behavior", __name__)

class ShutdownBehaviorNode(Node):
    is_spinning = None
    def __init__(self):
        super().__init__('shutdown_behavior_node')
        if self.is_spinning is not None:
            return
        self.is_spinning = False
        self._srv_proxy = self.create_client(Trigger, "/shutdown_behavior")

    def start_spinning(self):
        if not self.is_spinning:
            self.is_spinning = True
            rclpy.spin(self)
            self.is_spinning = False

    @property
    def srv_proxy(self):
        return self._srv_proxy


node = None
if ShutdownBehaviorNode.is_spinning is None:
    node = ShutdownBehaviorNode()

@shutdown_behavior_bp.route("/shutdown_behavior")
def _show_shutdown_behavior():
    global node
    try:
        req = Trigger.Request()
        future = node.srv_proxy.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            resp = future.result()
            if resp.success:
                return response_ok({})
            else:
                return response_error(message=resp.message)
        else:
            node.get_logger().error('Exception while calling service: %r' % future.exception())
            return response_error(message='Exception while calling service: %r' % future.exception())
    except Exception as e:
        node.get_logger().error('Failed to call service: %r' % e)
        return response_error(message='Failed to call service: %r' % e)
