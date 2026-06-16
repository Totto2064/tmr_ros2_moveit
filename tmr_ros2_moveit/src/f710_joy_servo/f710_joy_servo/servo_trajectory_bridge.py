#!/usr/bin/env python3
"""
Servo の JointTrajectory (Topic) を
tm_driver の FollowJointTrajectory (Action) に変換するブリッジ。

tm_driver の set_pvt_traj は time_from_start=0 の第 1 点をスキップするため、
ROS 上の 2 点軌道では内部 PVT が 1 点しか作られず fake_run が動かない。
そのため 3 点（現在・中間・目標）の軌道を送る。
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration as DurationMsg
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


class ServoTrajectoryBridge(Node):
    # tm_driver の set_pvt_traj が要求するセグメント最小時間 [s]
    TMIN_SEC = 0.025

    def __init__(self):
        super().__init__('servo_trajectory_bridge')

        self.declare_parameter('input_topic', '/tmr_arm_controller/joint_trajectory')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('action_name', '/tmr_arm_controller/follow_joint_trajectory')
        self.declare_parameter('min_send_interval_sec', 0.12)
        self.declare_parameter('trajectory_duration_sec', 0.1)
        self.declare_parameter('min_position_delta_rad', 0.001)

        input_topic = self.get_parameter('input_topic').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        action_name = self.get_parameter('action_name').value
        self.min_interval = self.get_parameter('min_send_interval_sec').value
        self.trajectory_duration = self.get_parameter('trajectory_duration_sec').value
        self.min_delta = self.get_parameter('min_position_delta_rad').value

        if self.trajectory_duration < 2.0 * self.TMIN_SEC:
            self.get_logger().warn(
                f'trajectory_duration_sec ({self.trajectory_duration}) is too short; '
                f'using {2.0 * self.TMIN_SEC}'
            )
            self.trajectory_duration = 2.0 * self.TMIN_SEC

        self.last_send_time = self.get_clock().now()
        self.current_goal_handle = None
        self.goal_in_progress = False
        self.joint_state_map = {}

        self.joint_state_sub = self.create_subscription(
            JointState, joint_states_topic, self.joint_state_callback, 10
        )
        self.traj_sub = self.create_subscription(
            JointTrajectory, input_topic, self.trajectory_callback, 10
        )
        self.action_client = ActionClient(
            self, FollowJointTrajectory, action_name
        )
        self.get_logger().info(
            f'Bridge ready: {input_topic} -> {action_name} '
            f'(3-point traj, duration={self.trajectory_duration}s)'
        )

    def joint_state_callback(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.joint_state_map[name] = pos

    def _lookup_current_positions(self, joint_names):
        if not self.joint_state_map:
            return None
        try:
            return [self.joint_state_map[name] for name in joint_names]
        except KeyError:
            return None

    @staticmethod
    def _max_position_delta(current, target):
        return max(abs(t - c) for c, t in zip(current, target))

    @staticmethod
    def _duration_to_msg(seconds):
        sec = int(seconds)
        nanosec = int(round((seconds - sec) * 1e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        return DurationMsg(sec=sec, nanosec=nanosec)

    @staticmethod
    def _interpolate_positions(a, b, ratio):
        return [x + (y - x) * ratio for x, y in zip(a, b)]

    def _make_point(self, positions, velocities, time_sec):
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.velocities = list(velocities)
        point.time_from_start = self._duration_to_msg(time_sec)
        return point

    def _build_tm_driver_trajectory(self, msg: JointTrajectory):
        if not msg.joint_names or not msg.points:
            return None

        target_point = msg.points[-1]
        if len(target_point.positions) != len(msg.joint_names):
            return None

        current_positions = self._lookup_current_positions(msg.joint_names)
        if current_positions is None:
            self.get_logger().warn(
                'Waiting for /joint_states with matching joint names',
                throttle_duration_sec=2.0,
            )
            return None

        target_positions = list(target_point.positions)
        if self._max_position_delta(current_positions, target_positions) < self.min_delta:
            return None

        half_time = self.trajectory_duration / 2.0
        mid_positions = self._interpolate_positions(current_positions, target_positions, 0.5)

        if target_point.velocities and len(target_point.velocities) == len(target_positions):
            end_velocities = list(target_point.velocities)
        else:
            end_velocities = [
                (t - c) / self.trajectory_duration
                for c, t in zip(current_positions, target_positions)
            ]
        mid_velocities = [v * 0.5 for v in end_velocities]

        traj = JointTrajectory()
        traj.header = msg.header
        traj.joint_names = list(msg.joint_names)
        traj.points = [
            self._make_point(current_positions, [0.0] * len(current_positions), 0.0),
            self._make_point(mid_positions, mid_velocities, half_time),
            self._make_point(target_positions, end_velocities, self.trajectory_duration),
        ]
        return traj

    def trajectory_callback(self, msg: JointTrajectory):
        if self.goal_in_progress:
            return

        now = self.get_clock().now()
        elapsed = (now - self.last_send_time).nanoseconds / 1e9
        if elapsed < self.min_interval:
            return

        traj = self._build_tm_driver_trajectory(msg)
        if traj is None:
            return

        if not self.action_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn('Action server not ready', throttle_duration_sec=2.0)
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self.goal_in_progress = True
        send_future = self.action_client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response_callback)
        self.last_send_time = now

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by tm_driver', throttle_duration_sec=2.0)
            self.goal_in_progress = False
            return

        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        self.goal_in_progress = False
        try:
            result = future.result().result
            if result.error_code != result.SUCCESSFUL:
                self.get_logger().debug(
                    f'Goal finished with error_code={result.error_code}, '
                    f'message={result.error_string}'
                )
        except Exception as exc:
            self.get_logger().debug(f'Goal result unavailable: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = ServoTrajectoryBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
