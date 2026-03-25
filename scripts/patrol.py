import math
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Empty, String
from geometry_msgs.msg import PoseStamped, PointStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class PatrolState(Enum):
    IDLE = "IDLE"
    READY = "READY"
    RUNNING = "RUNNING"
    FAILED = "FAILED"


class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

        self.declare_parameter('max_points', 4)
        self.declare_parameter('goal_timeout_sec', 90.0)
        self.declare_parameter('loop_forever', True)
        self.declare_parameter('default_frame_id', 'map')
        self.declare_parameter('success_dist_tolerance', 0.20)

        self.max_points = int(self.get_parameter('max_points').value)
        self.goal_timeout_sec = float(self.get_parameter('goal_timeout_sec').value)
        self.loop_forever = bool(self.get_parameter('loop_forever').value)
        self.default_frame_id = str(self.get_parameter('default_frame_id').value)
        self.success_dist_tolerance = float(self.get_parameter('success_dist_tolerance').value)

        self.points = []
        self.current_index = 0
        self.state = PatrolState.IDLE

        self.current_pose = None

        self._goal_handle = None
        self._result_future = None
        self._goal_start_time = None

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.clicked_point_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.on_clicked_point,
            10
        )

        self.start_sub = self.create_subscription(
            Empty,
            '/patrol/start',
            self.on_start,
            10
        )

        self.stop_sub = self.create_subscription(
            Empty,
            '/patrol/stop',
            self.on_stop,
            10
        )

        self.clear_sub = self.create_subscription(
            Empty,
            '/patrol/clear',
            self.on_clear,
            10
        )

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.on_pose,
            10
        )

        self.status_pub = self.create_publisher(String, '/patrol/status', 10)
        self.active_goal_pub = self.create_publisher(PoseStamped, '/patrol/active_goal', 10)

        self.timer = self.create_timer(0.2, self.on_timer)

        self.publish_status('started')
        self.get_logger().info('Patrol node started')

    def publish_status(self, text: str):
        msg = String()
        msg.data = f'{self.state.value}: {text}'
        self.status_pub.publish(msg)
        self.get_logger().info(msg.data)
        
    def on_pose(self, msg: PoseWithCovarianceStamped):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.current_pose = pose

    def yaw_to_quaternion(self, yaw: float):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qz, qw

    def make_pose_from_clicked_point(self, msg: PointStamped) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = msg.header.frame_id if msg.header.frame_id else self.default_frame_id

        pose.pose.position.x = msg.point.x
        pose.pose.position.y = msg.point.y
        pose.pose.position.z = msg.point.z

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        if self.current_pose is not None:
            dx = msg.point.x - self.current_pose.pose.position.x
            dy = msg.point.y - self.current_pose.pose.position.y

            if abs(dx) > 1e-6 or abs(dy) > 1e-6:
                yaw = math.atan2(dy, dx)
                qz, qw = self.yaw_to_quaternion(yaw)
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw

        return pose

    def on_clicked_point(self, msg: PointStamped):
        if len(self.points) >= self.max_points:
            self.publish_status(f'point ignored, already have {self.max_points} points')
            return

        point = self.make_pose_from_clicked_point(msg)
        self.points.append(point)

        self.publish_status(
            f'point added {len(self.points)}/{self.max_points} '
            f'frame={point.header.frame_id} '
            f'x={point.pose.position.x:.3f} y={point.pose.position.y:.3f}'
        )

        if len(self.points) == self.max_points and self.state in [PatrolState.IDLE, PatrolState.FAILED]:
            self.state = PatrolState.READY
            self.publish_status('ready to start')

    def on_start(self, _msg: Empty):
        if len(self.points) != self.max_points:
            self.publish_status(f'cannot start, need exactly {self.max_points} points')
            return

        if self.state == PatrolState.RUNNING:
            self.publish_status('already running')
            return

        self.state = PatrolState.RUNNING
        self.current_index = 0
        self.publish_status('starting patrol')
        self.send_current_goal()

    def on_stop(self, _msg: Empty):
        self.publish_status('stop requested')
        self.cancel_current_goal()
        self.state = PatrolState.READY if len(self.points) == self.max_points else PatrolState.IDLE

    def on_clear(self, _msg: Empty):
        self.publish_status('clear requested')
        self.cancel_current_goal()
        self.points.clear()
        self.current_index = 0
        self.state = PatrolState.IDLE

    def cancel_current_goal(self):
        if self._goal_handle is not None:
            try:
                cancel_future = self._goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self._cancel_done_cb)
            except Exception as e:
                self.get_logger().warning(f'cancel failed: {e}')

        self._goal_handle = None
        self._result_future = None
        self._goal_start_time = None

    def _cancel_done_cb(self, future):
        try:
            future.result()
            self.publish_status('goal cancel requested to server')
        except Exception as e:
            self.get_logger().warning(f'cancel callback error: {e}')

    def distance_to_goal(self, goal: PoseStamped):
        if self.current_pose is None:
            return None

        dx = self.current_pose.pose.position.x - goal.pose.position.x
        dy = self.current_pose.pose.position.y - goal.pose.position.y
        return math.hypot(dx, dy)

    def send_current_goal(self):
        if self.state != PatrolState.RUNNING:
            return

        if not self.points:
            self.state = PatrolState.IDLE
            self.publish_status('no points available')
            return

        if self.current_index >= len(self.points):
            if self.loop_forever:
                self.current_index = 0
            else:
                self.state = PatrolState.READY
                self.publish_status('patrol finished')
                return

        target = self.points[self.current_index]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target

        self.active_goal_pub.publish(target)

        self.publish_status(
            f'sending goal #{self.current_index + 1} '
            f'x={target.pose.position.x:.3f} '
            f'y={target.pose.position.y:.3f} '
            f'frame={target.header.frame_id}'
        )

        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.state = PatrolState.FAILED
            self.publish_status('navigate_to_pose action server not available')
            return

        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        _ = feedback_msg

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.state = PatrolState.FAILED
            self.publish_status(f'goal send failed: {e}')
            return

        if not goal_handle.accepted:
            self.state = PatrolState.FAILED
            self.publish_status('goal rejected')
            return

        self._goal_handle = goal_handle
        self._goal_start_time = self.get_clock().now()
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.result_callback)

        self.publish_status(f'goal accepted #{self.current_index + 1}')

    def result_callback(self, future):
        if self.state != PatrolState.RUNNING:
            return

        try:
            result = future.result()
        except Exception as e:
            self.state = PatrolState.FAILED
            self.publish_status(f'goal result failed: {e}')
            return

        status = result.status

        self._goal_handle = None
        self._result_future = None
        self._goal_start_time = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.publish_status(f'goal reached #{self.current_index + 1}')
            self.current_index += 1
            self.send_current_goal()

        elif status == GoalStatus.STATUS_CANCELED:
            self.state = PatrolState.READY if len(self.points) == self.max_points else PatrolState.IDLE
            self.publish_status('goal canceled')

        elif status == GoalStatus.STATUS_ABORTED:
            target = self.points[self.current_index]
            dist = self.distance_to_goal(target)

            if dist is not None and dist <= self.success_dist_tolerance:
                self.publish_status(
                    f'goal #{self.current_index + 1} aborted but close enough '
                    f'(dist={dist:.3f}), treating as reached'
                )
                self.current_index += 1
                self.send_current_goal()
            else:
                self.state = PatrolState.FAILED
                if dist is None:
                    self.publish_status('goal aborted and current pose is unknown')
                else:
                    self.publish_status(
                        f'goal aborted and too far from target (dist={dist:.3f})'
                    )

        else:
            self.state = PatrolState.FAILED
            self.publish_status(f'goal failed with status={status}')

    def on_timer(self):
        if self.state != PatrolState.RUNNING:
            return

        if self._goal_handle is None or self._goal_start_time is None:
            return

        elapsed = (self.get_clock().now() - self._goal_start_time).nanoseconds / 1e9
        if elapsed > self.goal_timeout_sec:
            self.publish_status(f'goal timeout after {elapsed:.1f} sec')
            self.state = PatrolState.FAILED
            self.cancel_current_goal()


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel_current_goal()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()