import rclpy
from flask import Blueprint
from std_srvs.srv import Trigger
from dt_robot_rest_api.utils import response_ok, response_error
from rclpy.node import Node

shutdown_behavior_bp = Blueprint("shutdown_behavior", __name__)

class ShutdownBehaviorNode(Node):
    def __init__(self):
        super().__init__('shutdown_behavior_node')
        self._srv_proxy = self.create_client(Trigger, "~shutdown_behavior")

    @shutdown_behavior_bp.route("/shutdown_behavior")
    def _show_shutdown_behavior(self):
        try:
            req = Trigger.Request()
            future = self._srv_proxy.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                resp = future.result()
                if resp.success:
                    return response_ok({})
                else:
                    return response_error(message=resp.message)
            else:
                self.get_logger().error('Exception while calling service: %r' % future.exception())
        except Exception as e:
            self.get_logger().error('Failed to call service: %r' % e)

def main(args=None):
    rclpy.init(args=args)

    node = ShutdownBehaviorNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()