import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from serial import Serial
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Header
from sensor_msgs.msg import Imu, PointCloud2, PointField
from threading import Thread
from hwnode.vl_lidar_reader import compute_zone_angles, distances_to_points
from hwnode import proto
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
MAX_WHEEL_SPEED = 20.0  # rad/s

CONTROL_HZ = 30
ANGULAR_GAIN = 1.8
LINEAR_GAIN = 1.0

MIN_INPLACE_ANGULAR = 0.8
INPLACE_ANGULAR_THRESHOLD = 0.1
INPLACE_LINEAR_THRESHOLD = 0.05

PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
SPEED = 921600


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
        self.pid_sub = self.create_subscription(Float32MultiArray, "/hardware/pid", self.pid_callback, 1)
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

    def handle_status(self, payload):
        status = Float32MultiArray()
        status.data = [
            payload.front_left.target,
            payload.front_left.speed,
            payload.front_left.angle % (2 * math.pi),
            payload.front_left.effort,

            payload.rear_left.target,
            payload.rear_left.speed,
            payload.rear_left.angle % (2 * math.pi),
            payload.rear_left.effort,

            payload.rear_right.target,
            payload.rear_right.speed,
            payload.rear_right.angle % (2 * math.pi),
            payload.rear_right.effort,

            payload.front_right.target,
            payload.front_right.speed,
            payload.front_right.angle % (2 * math.pi),
            payload.front_right.effort,

            payload.batt_voltage,
            payload.batt_current,
            payload.batt_percent,
        ]
        self.status_pub.publish(status)

    def handle_imu(self, payload):
        imu = Imu()

        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = "imu_link"

        acc_scale = 9.80665 / 1000.0

        imu.linear_acceleration.x = float(payload.acc[0]) * acc_scale
        imu.linear_acceleration.y = float(payload.acc[1]) * acc_scale
        imu.linear_acceleration.z = float(payload.acc[2]) * acc_scale

        gyro_scale = math.pi / 180.0

        imu.angular_velocity.x = float(payload.gyr[0]) * gyro_scale
        imu.angular_velocity.y = float(payload.gyr[1]) * gyro_scale
        imu.angular_velocity.z = float(payload.gyr[2]) * gyro_scale

        imu.orientation.w = float(payload.quat[0])
        imu.orientation.x = float(payload.quat[1])
        imu.orientation.y = float(payload.quat[2])
        imu.orientation.z = float(payload.quat[3])

        self.imu_pub.publish(imu)

    def handle_tof(self, payload):
        distances = np.array(
            list(payload.distance_mm),
            dtype=np.float32
        )
        #print(distances.reshape(8,8))

        statuses = np.array(
            list(payload.status),
            dtype=np.uint8
        )

        tan_x, tan_y = self.tof_tan_lookup

        points = distances_to_points(
            distances,
            tan_x,
            tan_y
        )

        mask = (statuses == 5) & (distances >= 20)

        #points = points[mask]

        cloud = self.numpy_to_pointcloud2(
            points,
            frame_id="tof_lidar"
        )

        self.pc2_pub.publish(cloud)

    def read_loop(self):
        buff = b""

        while rclpy.ok():
            ret = proto.read_packet(self.ser)
            if ret is None: continue
            type, payload = ret
            if type == proto.PacketType.STATUS:
                self.handle_status(payload)
            elif type == proto.PacketType.IMU:
                self.handle_imu(payload)
            elif type == proto.PacketType.TOF:
                self.handle_tof(payload)
                 
    def cmd_callback(self, msg: Twist):
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

    def pid_callback(self, msg: Float32MultiArray):
        if len(msg.data) != 5:
            self.get_logger().warn("Bad PID message, expected 5 numbers: [kp, ki, kd, limit, lpf_tf]")
            return
        kp, ki, kd, limit, lpf_tf = msg.data
        pack = proto.PidState(kp, ki, kd, limit, lpf_tf)
        self.get_logger().info(f"Set PID params: {pack}")
        proto.write_packet(self.ser, pack)

    def ramp(self, current, target, accel_step, decel_step):
        step = accel_step if abs(target) > abs(current) else decel_step
        if target > current:
            return min(current + step, target)
        return max(current - step, target)

    def update(self):
        if (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9 > 0.3:
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

        pack = proto.ControlPacket(float(A), float(B), float(C), float(D))
        proto.write_packet(self.ser, pack)

        
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
