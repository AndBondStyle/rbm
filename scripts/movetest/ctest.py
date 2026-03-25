import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


ITERS = 300
START, END = 0, 6.28
STEP = (END - START) / ITERS


class CalibNode(Node):
    def __init__(self):
        super().__init__('calib_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.control_timer = self.create_timer(1 / 10, self.control_loop)
        self.control_timer.cancel()
        self.start_timer = self.create_timer(2.0, self.auto_start)
        self.get_logger().info('Test starts in 2 seconds')

        self.iter = 0
        self.value = START

    def send(self, value):
        msg = Twist()
        msg.angular.z = float(value)
        self.cmd_vel_pub.publish(msg)

    def control_loop(self):
        self.get_logger().info(f'Iter {self.iter} / {ITERS}')
        self.send(self.value)
        self.value += STEP
        self.iter += 1
        if self.iter == ITERS:
            self.get_logger().info('Test finish')
            self.control_timer.cancel()
            self.send(0)
            raise RuntimeError

    def auto_start(self):
        self.get_logger().info('Test start')
        self.control_timer.reset()
        self.start_timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = CalibNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('EXIT')
    finally:
        node.send(0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
