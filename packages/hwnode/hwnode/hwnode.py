import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from serial import Serial
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from threading import Thread
from hwnode.crc8 import crc8
from dataclasses import dataclass
import math
import struct

WHEEL_RADIUS = 0.021
WHEEL_BASE = 0.128

MAX_LINEAR = 0.1          # m/s
MAX_ANGULAR = 1.0         # rad/s
MAX_ACCEL_LINEAR = 5.5    # m/s² (accel + decel)
MAX_ACCEL_ANGULAR = 20.0  # rad/s² (accel + decel)
MAX_WHEEL_SPEED = 35      # rad/s
CONTROL_HZ = 30

LIN_ACCEL = MAX_ACCEL_LINEAR / CONTROL_HZ
ANG_ACCEL = MAX_ACCEL_ANGULAR / CONTROL_HZ

PORT = "/dev/ttyAMA0"
SPEED = 115200


@dataclass
class Feedback:
    accel_x: float
    accel_y: float
    accel_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    mag_x: float
    mag_y: float
    mag_z: float
    left_angle: float
    right_angle: float
    left_speed: float
    right_speed: float
    left_lidar: int
    right_lidar: int

    @classmethod
    def unpack(cls, buff: bytes):
        format = "<hhhhhhhhhffffHHx"
        if len(buff) != struct.calcsize(format): return
        values = struct.unpack(format, buff)
        return Feedback(
            accel_x=values[0],
            accel_y=values[1],
            accel_z=values[2],
            gyro_x=values[3] * 250.0 / 32768.0 / 180 * math.pi,
            gyro_y=values[4] * 250.0 / 32768.0 / 180 * math.pi,
            gyro_z=values[5] * 250.0 / 32768.0 / 180 * math.pi,
            mag_x=values[6],
            mag_y=values[7],
            mag_z=values[8],
            left_angle=values[9],
            right_angle=values[10],
            left_speed=values[11],
            right_speed=values[12],
            left_lidar=values[13],
            right_lidar=values[14],
        )


class HardwareNode(Node):
    def __init__(self):
        super().__init__("hardware_node")

        self.L = WHEEL_BASE
        self.R = WHEEL_RADIUS
        self.max_wheel = MAX_WHEEL_SPEED * self.R

        self.target_v = 0.0
        self.target_w = 0.0
        self.current_v = 0.0
        self.current_w = 0.0
        self.v_left = 0.0
        self.v_right = 0.0

        self.last_feedback: Feedback = None
        self.ser = Serial(port=PORT, baudrate=SPEED, timeout=0.1, exclusive=True)
        self.get_logger().info("Serial подключен")

        self.control_timer = self.create_timer(1.0 / CONTROL_HZ, self.update)
        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 1)  # /hardware/cmd
        self.imu_pub = self.create_publisher(Imu, "/hardware/imu", 1)
        self.status_pub = self.create_publisher(Float32MultiArray, "/hardware/status", 1)
        self.odom_pub = self.create_publisher(Odometry, "/hardware/odom", 1)
        # self.cmd_flt_pub = self.create_publisher(Twist, "/hardware/cmd/filtered", 1)

        self.read_thread = Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

    def read_loop(self):
        buff = b""

        while rclpy.ok():
            chunk = self.ser.read_until(b"\x7E")
            if len(buff) + len(chunk) > 39: buff = b""
            buff += chunk
            # chunk = chunk.rstrip(b"\x7E")
            feedback = Feedback.unpack(buff)
            # print(buff, len(buff), feedback)
            if feedback is None: continue

            now = self.get_clock().now().to_msg()

            imu = Imu()
            imu.header.frame_id = "base_link"  # FIXME
            imu.header.stamp = now
            imu.orientation_covariance = [-1.0] + [0.0] * 8
            imu.linear_acceleration_covariance = [-1.0] + [0.0] * 8
            imu.angular_velocity.x = feedback.gyro_x
            imu.angular_velocity.y = feedback.gyro_y
            imu.angular_velocity.z = feedback.gyro_z
            imu.angular_velocity_covariance = [0.0] * 9
            imu.angular_velocity_covariance[0] = 0.001
            imu.angular_velocity_covariance[4] = 0.001
            imu.angular_velocity_covariance[8] = 0.001
            self.imu_pub.publish(imu)

            odom = Odometry()
            odom.child_frame_id = "base_link"
            odom.header.frame_id = "base_link"
            odom.header.stamp = now
            linear_vel = (feedback.left_speed + feedback.right_speed) * self.R / 2
            odom.twist.twist.linear.x = linear_vel
            odom.twist.covariance = [0.0] * 36
            odom.twist.covariance[0] = 0.001
            odom.twist.covariance[35] = 1000.0
            if abs(feedback.left_speed) < 0.001 and abs(feedback.right_speed) < 0.001:
                odom.twist.covariance[0] = 0.000001
                odom.twist.covariance[35] = 0.000001
            self.odom_pub.publish(odom)

    def cmd_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        v = math.copysign(min(abs(v), MAX_LINEAR), v)
        if abs(w) > 0.09:
            w = math.copysign(max(abs(w), 0.3), w)
        w = math.copysign(min(abs(w), MAX_ANGULAR), w)
        self.target_v, self.target_w = v, w

    def update(self):
        self.current_v = self.target_v
        self.current_w = self.target_w
        self.v_left = (self.current_v - self.current_w * self.L / 2) / self.R
        self.v_right = (self.current_v + self.current_w * self.L / 2) / self.R

        buf = struct.pack("<ffB", self.v_left, self.v_right, 0)
        buf = b"\xA0" + buf
        crc = crc8()
        crc.update(buf)
        buf = b"\x7E" + buf + crc.digest()
        # debug = [f"{i:02x}" for i in buf]
        # print(f"-> {' '.join(debug)}")
        self.ser.write(buf)
        self.ser.flush()


def main():
    rclpy.init()
    node = HardwareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
