#!/usr/bin/env python3

import math
import time

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandLong, SetMode

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


class OffboardTakeoffNode(Node):
    def __init__(self) -> None:
        super().__init__('offboard_takeoff_node')

        self.declare_parameter('target_altitude', 5.0)
        self.declare_parameter('setpoint_rate_hz', 20.0)
        self.declare_parameter('preflight_timeout_sec', 30.0)
        self.declare_parameter('takeoff_timeout_sec', 60.0)
        self.declare_parameter('force_arm_if_needed', True)

        self.target_altitude = float(self.get_parameter('target_altitude').value)
        self.setpoint_rate_hz = float(self.get_parameter('setpoint_rate_hz').value)
        self.preflight_timeout_sec = float(self.get_parameter('preflight_timeout_sec').value)
        self.takeoff_timeout_sec = float(self.get_parameter('takeoff_timeout_sec').value)
        self.force_arm_if_needed = bool(self.get_parameter('force_arm_if_needed').value)

        self.current_state = State()
        self.local_pose = PoseStamped()

        qos_best_effort = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            State, '/mavros/state', self._state_cb, qos_best_effort
        )
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self._pose_cb, qos_best_effort
        )
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10
        )

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.command_client = self.create_client(CommandLong, '/mavros/cmd/command')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.target_pose = PoseStamped()
        self.target_pose.pose.position.x = 0.0
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = self.target_altitude
        self.target_pose.pose.orientation.w = 1.0

        self.dt = 1.0 / self.setpoint_rate_hz

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

    def _publish_setpoint_burst(self, count: int) -> None:
        for _ in range(count):
            if not rclpy.ok():
                return
            self.target_pose.header.stamp = self.get_clock().now().to_msg()
            self.setpoint_pub.publish(self.target_pose)
            self._spin_sleep(self.dt)

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
        # MAV_CMD_COMPONENT_ARM_DISARM
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

    def run(self) -> int:
        self.get_logger().info('Waiting for MAVROS services...')
        if not self._wait_for_service_clients(self.preflight_timeout_sec):
            self.get_logger().error('MAVROS services unavailable')
            return 1

        self.get_logger().info('Waiting for FCU connection...')
        if not self._wait_for_fcu_connection(self.preflight_timeout_sec):
            self.get_logger().error('FCU not connected')
            return 1

        self.get_logger().info('Publishing initial setpoints')
        self._publish_setpoint_burst(100)

        deadline = time.time() + self.preflight_timeout_sec
        last_mode_req = 0.0
        last_arm_req = 0.0
        used_force_arm = False

        while rclpy.ok() and time.time() < deadline:
            self.target_pose.header.stamp = self.get_clock().now().to_msg()
            self.setpoint_pub.publish(self.target_pose)

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

        self.get_logger().info(f'Taking off to {self.target_altitude:.1f} m')
        takeoff_deadline = time.time() + self.takeoff_timeout_sec
        while rclpy.ok() and time.time() < takeoff_deadline:
            self.target_pose.header.stamp = self.get_clock().now().to_msg()
            self.setpoint_pub.publish(self.target_pose)
            rclpy.spin_once(self, timeout_sec=0.1)

            current_z = self.local_pose.pose.position.z
            if not math.isnan(current_z) and current_z >= self.target_altitude * 0.95:
                self.get_logger().info(
                    f'Target altitude reached: z={current_z:.2f} m'
                )
                self._publish_setpoint_burst(int(self.setpoint_rate_hz * 3))
                return 0

            self._spin_sleep(self.dt)

        self.get_logger().error('Takeoff timeout')
        return 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OffboardTakeoffNode()
    code = 1
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)
