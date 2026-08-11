import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from serial import Serial
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Header
from sensor_msgs.msg import Imu, PointCloud2, PointField
from threading import Thread
from hwnode.crc8 import crc8
from hwnode.vl_lidar_reader import compute_zone_angles, distances_to_points
from dataclasses import dataclass
import math
import struct
import numpy as np

WHEEL_RADIUS = 0.045 / 2  # m
WHEEL_BASE = 0.140  # m
MAX_LINEAR = 0.5  # !!! 0.5  # m/s
MAX_ANGULAR = 2.5 # !!! 2.5  # rad/s
MAX_ACCEL_LINEAR = 0.3  # m/s²
MAX_ACCEL_ANGULAR = 5.0  # rad/s²
MAX_DECEL_LINEAR = 2.0  # m/s²
MAX_DECEL_ANGULAR = 5.0  # rad/s²
MAX_WHEEL_SPEED = 35.0  # rad/s

CONTROL_HZ = 30
ANGULAR_GAIN = 1.8
LINEAR_GAIN = 1.0

MIN_INPLACE_ANGULAR = 0.8
INPLACE_ANGULAR_THRESHOLD = 0.1
INPLACE_LINEAR_THRESHOLD = 0.05

#PORT = "/dev/ttyAMA0"
#SPEED = 115200
PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
SPEED = 921600


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
        self.last_cmd_time = self.get_clock().now()
        self.ser = Serial(port=PORT, baudrate=SPEED, timeout=0.5)
        self.get_logger().info("Serial подключен")

        self.control_timer = self.create_timer(1.0 / CONTROL_HZ, self.update)
        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 1)
        self.imu_pub = self.create_publisher(Imu, "/hardware/imu", 1)
        self.status_pub = self.create_publisher(Float32MultiArray, "/hardware/status", 1)
        self.odom_pub = self.create_publisher(Odometry, "/hardware/odom", 1)
        self.pc2_pub = self.create_publisher(PointCloud2, "/tof/cloud", 10)
        self.tof_tan_lookup = compute_zone_angles()

        self.read_thread = Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

    def numpy_to_pointcloud2(self, points, frame_id="tof_lidar"):
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        point_step = 12
        data = points[:, :3].astype(np.float32).tobytes()
        
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = points.shape[0]
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = point_step
        msg.row_step = point_step * points.shape[0]
        msg.is_dense = True
        msg.data = data
        
        return msg

    def read_loop(self):
        buff = b""

        while rclpy.ok():
            line = self.ser.readline()
            line = line.decode("utf-8", errors="ignore").strip()
            if not line: continue
            if line[0].upper() in "ABCD":
                # TODO: Parse motors feedback
                # print(line)
                try:
                    if line[0].upper() != "A": continue
                    values = list(map(float, line[1:].split()))
                    status = Float32MultiArray()
                    status.data = values
                    self.status_pub.publish(status)
                except Exception as err:
                    print("ERR", err, line)
                    continue
            if line[0].upper() == "L":
                #print(">>>", line)
                try:
                    distances, status = line[1:].split(";")
                    distances = [int(x) for x in distances.split(",")]
                    status = [int(x) for x in status.split(",")]
                    distances = np.array(distances, dtype=np.float32)
                    status = np.array(status, dtype=np.uint8)
                    points = distances_to_points(distances, *self.tof_tan_lookup)
                    mask = (status == 5)# & (distances >= MIN_RANGE_MM)
                    msg = self.numpy_to_pointcloud2(points[mask])
                    self.pc2_pub.publish(msg)
                except Exception as err:
                    print("ERR", err, line)
                    continue
                # print(dist, status)
        # while rclpy.ok():
        #     chunk = self.ser.read_until(b"\x7E")
        #     if len(buff) + len(chunk) > 39: buff = b""
        #     buff += chunk
        #     # chunk = chunk.rstrip(b"\x7E")
        #     feedback = Feedback.unpack(buff)
        #     # print(buff, len(buff), feedback)
        #     if feedback is None: continue

        #     now = self.get_clock().now().to_msg()

        #     status = Float32MultiArray()
        #     status.data = [
        #         self.v_left,
        #         feedback.left_speed,
        #         feedback.left_angle,
        #         self.v_right,
        #         feedback.right_speed,
        #         feedback.right_angle,
        #     ]
        #     self.status_pub.publish(status)

        #     imu = Imu()
        #     imu.header.frame_id = "base_link"
        #     imu.header.stamp = now
        #     imu.orientation_covariance = [-1.0] + [0.0] * 8
        #     imu.linear_acceleration_covariance = [-1.0] + [0.0] * 8
        #     imu.angular_velocity.x = feedback.gyro_x
        #     imu.angular_velocity.y = feedback.gyro_y
        #     imu.angular_velocity.z = feedback.gyro_z
        #     imu.angular_velocity_covariance = [0.0] * 9
        #     imu.angular_velocity_covariance[0] = 0.001
        #     imu.angular_velocity_covariance[4] = 0.001
        #     imu.angular_velocity_covariance[8] = 0.001
        #     self.imu_pub.publish(imu)

        #     odom = Odometry()
        #     odom.child_frame_id = "base_link"
        #     odom.header.frame_id = "base_link"
        #     odom.header.stamp = now
        #     linear_vel = (feedback.left_speed + feedback.right_speed) * self.R / 2
        #     odom.twist.twist.linear.x = linear_vel
        #     odom.twist.covariance = [0.0] * 36
        #     odom.twist.covariance[0] = 0.001
        #     odom.twist.covariance[35] = 1000.0
        #     if abs(feedback.left_speed) < 0.001 and abs(feedback.right_speed) < 0.001:
        #         odom.twist.covariance[0] = 0.000001
        #         odom.twist.covariance[35] = 0.000001
        #     self.odom_pub.publish(odom)

    def cmd_callback(self, msg: Twist):
        # x = msg.angular.z
        # L, R = x, -x
        # print(L, R)
        # A, B, C, D = L, -R, -R, L
        # buff = b""
        # buff += f"A{A:.2f}\n".encode()
        # buff += f"C{B:.2f}\n".encode()
        # buff += f"B{C:.2f}\n".encode()
        # buff += f"D{D:.2f}\n".encode()
        # self.ser.write(buff)
        # self.ser.flush()

        self.last_cmd_time = self.get_clock().now()
        v = msg.linear.x * LINEAR_GAIN
        w = msg.angular.z * ANGULAR_GAIN

        v = math.copysign(min(abs(v), MAX_LINEAR), v)
        w = math.copysign(min(abs(w), MAX_ANGULAR), w)

        if abs(v) < INPLACE_LINEAR_THRESHOLD and abs(w) > INPLACE_ANGULAR_THRESHOLD:
            w = math.copysign(max(abs(w), MIN_INPLACE_ANGULAR), w)
        elif abs(w) < INPLACE_ANGULAR_THRESHOLD:
            w = 0.0

        self.target_v, self.target_w = v, w
        # print(self.target_v, self.target_w)

    def ramp(self, current, target, accel_step, decel_step):
        step = accel_step if abs(target) > abs(current) else decel_step
        if target > current:
            return min(current + step, target)
        return max(current - step, target)

    def update(self):
        if (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9 > 0.3:
            # print("err")
            self.target_v = 0.0
            self.target_w = 0.0

        self.current_v = self.ramp(
            self.current_v,
            self.target_v,
            MAX_ACCEL_LINEAR / CONTROL_HZ,
            MAX_DECEL_LINEAR / CONTROL_HZ,
        )
        self.current_w = self.ramp(
            self.current_w,
            self.target_w,
            MAX_ACCEL_ANGULAR / CONTROL_HZ,
            MAX_DECEL_ANGULAR / CONTROL_HZ,
        )

        v_left = (self.current_v - self.current_w * self.L / 2) / self.R
        v_right = (self.current_v + self.current_w * self.L / 2) / self.R
        peak = max(abs(self.v_left), abs(self.v_right), 1e-9)
        scale = min(1.0, MAX_WHEEL_SPEED / peak)
        self.v_left = v_left * scale
        self.v_right = v_right * scale

        L, R = self.v_left * 1.0, self.v_right * 1.0
        A, B, C, D = L, -R, L, -R
        buff = b""
        buff += f"A{A:.2f}\n".encode()
        buff += f"C{B:.2f}\n".encode()
        buff += f"B{C:.2f}\n".encode()
        buff += f"D{D:.2f}\n".encode()
        self.ser.write(buff)
        self.ser.flush()
        #print(f">>> {buff.replace(b'\n', b' ')}")

        #buf = struct.pack("<ffB", self.v_left, self.v_right, 0)
        #buf = b"\xA0" + buf
        #crc = crc8()
        #crc.update(buf)
        #buf = b"\x7E" + buf + crc.digest()
        # debug = [f"{i:02x}" for i in buf]
        # print(f"-> {' '.join(debug)}")
        #self.ser.write(buf)
        #self.ser.flush()


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


if __name__ == "__main__":
    main()
