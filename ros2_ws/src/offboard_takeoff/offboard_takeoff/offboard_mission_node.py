#!/usr/bin/env python3

import math
import time
from typing import List, Tuple

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandLong, SetMode

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class OffboardMissionNode(Node):
    def __init__(self) -> None:
        super().__init__('offboard_mission_node')

        self.declare_parameter('setpoint_rate_hz', 20.0)
        self.declare_parameter('preflight_timeout_sec', 30.0)
        self.declare_parameter('mission_timeout_sec', 180.0)
        self.declare_parameter('force_arm_if_needed', True)
        self.declare_parameter('waypoints', '0,0,3;5,0,3;5,5,3;0,0,3')
        self.declare_parameter('hold_sec_each_wp', 2.0)
        self.declare_parameter('position_tolerance_m', 0.6)

        self.setpoint_rate_hz = float(self.get_parameter('setpoint_rate_hz').value)
        self.preflight_timeout_sec = float(
            self.get_parameter('preflight_timeout_sec').value
        )
        self.mission_timeout_sec = float(self.get_parameter('mission_timeout_sec').value)
        self.force_arm_if_needed = bool(self.get_parameter('force_arm_if_needed').value)
        self.hold_sec_each_wp = float(self.get_parameter('hold_sec_each_wp').value)
        self.position_tolerance_m = float(
            self.get_parameter('position_tolerance_m').value
        )

        wp_param = str(self.get_parameter('waypoints').value).strip()
        self.waypoints = self._parse_waypoints(wp_param)
        if not self.waypoints:
            raise RuntimeError('No valid waypoints provided')

        self.current_state = State()
        self.local_pose = PoseStamped()

        qos_best_effort = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(State, '/mavros/state', self._state_cb, qos_best_effort)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self._pose_cb, qos_best_effort
        )
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10
        )

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.command_client = self.create_client(CommandLong, '/mavros/cmd/command')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.dt = 1.0 / self.setpoint_rate_hz

    @staticmethod
    def _parse_waypoints(text: str) -> List[Tuple[float, float, float]]:
        waypoints: List[Tuple[float, float, float]] = []
        for block in text.split(';'):
            part = block.strip()
            if not part:
                continue
            coords = [c.strip() for c in part.split(',')]
            if len(coords) != 3:
                continue
            try:
                x, y, z = float(coords[0]), float(coords[1]), float(coords[2])
            except ValueError:
                continue
            waypoints.append((x, y, z))
        return waypoints

    def _state_cb(self, msg: State) -> None:
        self.current_state = msg

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.local_pose = msg

    def _spin_sleep(self, seconds: float) -> None:
        end_time = time.time() + seconds
        while rclpy.ok() and time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _wait_for_service_clients(self, timeout_sec: float) -> bool:
        deadline = time.time() + timeout_sec
        while rclpy.ok() and time.time() < deadline:
            arming_ok = self.arming_client.wait_for_service(timeout_sec=0.2)
            command_ok = self.command_client.wait_for_service(timeout_sec=0.2)
            mode_ok = self.set_mode_client.wait_for_service(timeout_sec=0.2)
            if arming_ok and command_ok and mode_ok:
                return True
        return False

    def _wait_for_fcu_connection(self, timeout_sec: float) -> bool:
        deadline = time.time() + timeout_sec
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_state.connected:
                return True
        return False

    def _set_mode_offboard(self) -> bool:
        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'
        future = self.set_mode_client.call_async(req)
        end = time.time() + 5.0
        while rclpy.ok() and time.time() < end and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            self.get_logger().error('OFFBOARD mode request timed out')
            return False
        result = future.result()
        return bool(result and result.mode_sent)

    def _arm(self) -> bool:
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        end = time.time() + 5.0
        while rclpy.ok() and time.time() < end and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            self.get_logger().error('Arming request timed out')
            return False
        result = future.result()
        return bool(result and result.success)

    def _arm_force(self) -> bool:
        req = CommandLong.Request()
        req.broadcast = False
        req.command = 400
        req.confirmation = 0
        req.param1 = 1.0
        req.param2 = 21196.0

        future = self.command_client.call_async(req)
        end = time.time() + 5.0
        while rclpy.ok() and time.time() < end and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            self.get_logger().error('Force arm request timed out')
            return False
        result = future.result()
        return bool(result and result.success)

    def _publish_target(self, x: float, y: float, z: float) -> None:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        self.setpoint_pub.publish(pose)

    def _distance_to(self, x: float, y: float, z: float) -> float:
        dx = self.local_pose.pose.position.x - x
        dy = self.local_pose.pose.position.y - y
        dz = self.local_pose.pose.position.z - z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _reach_waypoint(self, x: float, y: float, z: float, timeout_sec: float) -> bool:
        deadline = time.time() + timeout_sec
        hold_start = None
        while rclpy.ok() and time.time() < deadline:
            self._publish_target(x, y, z)
            rclpy.spin_once(self, timeout_sec=0.05)

            if self._distance_to(x, y, z) <= self.position_tolerance_m:
                if hold_start is None:
                    hold_start = time.time()
                if time.time() - hold_start >= self.hold_sec_each_wp:
                    return True
            else:
                hold_start = None

            self._spin_sleep(self.dt)
        return False

    def run(self) -> int:
        self.get_logger().info('Waiting for MAVROS services...')
        if not self._wait_for_service_clients(self.preflight_timeout_sec):
            self.get_logger().error('MAVROS services unavailable')
            return 1

        self.get_logger().info('Waiting for FCU connection...')
        if not self._wait_for_fcu_connection(self.preflight_timeout_sec):
            self.get_logger().error('FCU not connected')
            return 1

        first_wp = self.waypoints[0]
        self.get_logger().info('Publishing initial setpoints')
        for _ in range(120):
            self._publish_target(first_wp[0], first_wp[1], first_wp[2])
            self._spin_sleep(self.dt)

        deadline = time.time() + self.preflight_timeout_sec
        last_mode_req = 0.0
        last_arm_req = 0.0
        used_force_arm = False

        while rclpy.ok() and time.time() < deadline:
            self._publish_target(first_wp[0], first_wp[1], first_wp[2])

            mode_ok = self.current_state.mode == 'OFFBOARD'
            arm_ok = self.current_state.armed

            if not mode_ok:
                if time.time() - last_mode_req > 1.0 and self._set_mode_offboard():
                    self.get_logger().info('OFFBOARD mode set')
                last_mode_req = time.time()
            elif not arm_ok:
                if time.time() - last_arm_req > 1.0 and self._arm():
                    self.get_logger().info('Vehicle armed')
                elif (
                    self.force_arm_if_needed
                    and not used_force_arm
                    and time.time() - last_arm_req > 1.0
                    and self._arm_force()
                ):
                    used_force_arm = True
                    self.get_logger().warn('Force arm accepted (SITL fallback)')
                last_arm_req = time.time()
            else:
                break

            self._spin_sleep(self.dt)

        if self.current_state.mode != 'OFFBOARD':
            self.get_logger().error('Could not switch to OFFBOARD')
            return 1
        if not self.current_state.armed:
            self.get_logger().error('Could not arm vehicle')
            return 1

        self.get_logger().info(f'Starting mission with {len(self.waypoints)} waypoints')
        start = time.time()
        per_wp_timeout = max(8.0, self.mission_timeout_sec / float(len(self.waypoints)))

        for idx, wp in enumerate(self.waypoints):
            x, y, z = wp
            self.get_logger().info(f'Waypoint {idx + 1}/{len(self.waypoints)} -> ({x}, {y}, {z})')
            if not self._reach_waypoint(x, y, z, per_wp_timeout):
                self.get_logger().error(f'Waypoint timeout at index {idx}')
                return 1

        elapsed = time.time() - start
        self.get_logger().info(f'Mission complete in {elapsed:.1f}s')
        return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OffboardMissionNode()
    code = 1
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)
