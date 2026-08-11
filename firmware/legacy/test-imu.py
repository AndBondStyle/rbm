import rclpy
from rclpy.node import Node
from serial import Serial
from threading import Thread
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
SPEED = 921600


class IMUTestNode(Node):
    def __init__(self):
        super().__init__("imu_test_node")
        self.ser = Serial(port=PORT, baudrate=SPEED, timeout=0.5, exclusive=True)
        self.ser.write(b"\n")
        self.get_logger().info("Serial connected")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.read_thread = Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

    def read_loop(self):
        while rclpy.ok():
            line = self.ser.readline()
            line = line.decode("utf-8", errors="ignore").strip()
            if not line: continue
            if line[0] == "Q":
                values = list(map(float, line[1:].split()))
                print(">>>", values)
                x, y, z, w, acc = values
                
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'imu'
                t.child_frame_id = 'base'
                t.transform.rotation.x = x
                t.transform.rotation.y = y
                t.transform.rotation.z = z
                t.transform.rotation.w = w
                self.tf_broadcaster.sendTransform(t)
            else:
                print("[?]", line)


def main():
    rclpy.init()
    node = IMUTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
