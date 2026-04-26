#!/usr/bin/env python3

import math
import time
from typing import List, Tuple

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandLong, SetMode

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class FWMissionNode(Node):
    def __init__(self) -> None:
        super().__init__('fw_mission_node')

        self.declare_parameter('waypoints', '0,0,30;50,0,30;50,50,30;0,50,30')
        self.declare_parameter('setpoint_rate_hz', 10.0)
        self.declare_parameter('preflight_timeout_sec', 120.0)
        self.declare_parameter('mission_timeout_sec', 300.0)
        self.declare_parameter('force_arm_if_needed', True)
        self.declare_parameter('position_tolerance_m', 15.0)
        self.declare_parameter('cruise_speed_mps', 12.0)
        self.declare_parameter('takeoff_altitude_m', 30.0)
        self.declare_parameter('loiter_after_mission', True)

        self.setpoint_rate_hz = float(self.get_parameter('setpoint_rate_hz').value)
        self.preflight_timeout_sec = float(
            self.get_parameter('preflight_timeout_sec').value
        )
        self.mission_timeout_sec = float(self.get_parameter('mission_timeout_sec').value)
        self.force_arm_if_needed = bool(self.get_parameter('force_arm_if_needed').value)
        self.position_tolerance_m = float(
            self.get_parameter('position_tolerance_m').value
        )
        self.cruise_speed_mps = float(self.get_parameter('cruise_speed_mps').value)
        self.takeoff_altitude_m = float(self.get_parameter('takeoff_altitude_m').value)
        self.loiter_after_mission = bool(self.get_parameter('loiter_after_mission').value)

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
        self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_body', self._vel_cb, qos_best_effort
        )

        # Fixed-wing requires PositionTarget (not PoseStamped) for proper path-following
        self.setpoint_pub = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', 10
        )

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.command_client = self.create_client(CommandLong, '/mavros/cmd/command')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.dt = 1.0 / self.setpoint_rate_hz
        self._current_local_pos = (0.0, 0.0, 0.0)

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
        p = msg.pose.position
        self._current_local_pos = (p.x, p.y, p.z)

    def _vel_cb(self, msg: TwistStamped) -> None:
        pass  # available for future use

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

    def _set_mode_rtl(self) -> bool:
        req = SetMode.Request()
        req.custom_mode = 'AUTO.RTL'
        future = self.set_mode_client.call_async(req)
        end = time.time() + 5.0
        while rclpy.ok() and time.time() < end and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            self.get_logger().error('RTL mode request timed out')
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

    def _publish_fw_setpoint(
        self, x: float, y: float, z: float,
        vx: float = 0.0, vy: float = 0.0, vz: float = 0.0,
        type_mask: int = 0,
    ) -> None:
        """
        Publish a PositionTarget for fixed-wing offboard control.

        type_mask values:
          0     = position + velocity (FW_POSCTRL_MODE_AUTO_PATH - plane flies toward point)
          4096  = takeoff setpoint
          8192  = land setpoint
          12288 = loiter setpoint (circles centered on point)
        """
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # For FW: must include velocity feedforward to get AUTO_PATH mode
        # type_mask bits: 0=px,1=py,2=pz,3=vx,4=vy,5=vz,6=afx,7=afy,8=afz,9=yaw,10=yaw_rate
        # Unset bits = ignore that field. We set position + velocity.
        # Default type_mask=0 means ALL fields valid.
        msg.type_mask = type_mask

        msg.position.x = x
        msg.position.y = y
        msg.position.z = z

        # Velocity feedforward: normalized direction toward target * cruise speed
        # This triggers FW_POSCTRL_MODE_AUTO_PATH instead of orbit mode
        if math.isfinite(vx) and (abs(vx) + abs(vy) + abs(vz)) > 0.01:
            msg.velocity.x = vx
            msg.velocity.y = vy
            msg.velocity.z = vz

        self.setpoint_pub.publish(msg)

    def _compute_velocity_to_waypoint(
        self, wx: float, wy: float, wz: float,
    ) -> Tuple[float, float, float]:
        """Compute velocity feedforward vector toward waypoint at cruise speed."""
        cx, cy, cz = self._current_local_pos
        dx = wx - cx
        dy = wy - cy
        dz = wz - cz
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist < 0.1:
            return (0.0, 0.0, 0.0)
        scale = self.cruise_speed_mps / dist
        return (dx * scale, dy * scale, dz * scale)

    def _publish_setpoint_burst(self, count: int) -> None:
        first_wp = self.waypoints[0]
        for _ in range(count):
            if not rclpy.ok():
                return
            vx, vy, vz = self._compute_velocity_to_waypoint(*first_wp)
            self._publish_fw_setpoint(
                first_wp[0], first_wp[1], first_wp[2],
                vx=vx, vy=vy, vz=vz,
            )
            self._spin_sleep(self.dt)

    def _distance_to(self, x: float, y: float, z: float) -> float:
        cx, cy, cz = self._current_local_pos
        dx = cx - x
        dy = cy - y
        dz = cz - z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _reach_waypoint(self, x: float, y: float, z: float, timeout_sec: float) -> bool:
        deadline = time.time() + timeout_sec
        while rclpy.ok() and time.time() < deadline:
            vx, vy, vz = self._compute_velocity_to_waypoint(x, y, z)
            self._publish_fw_setpoint(x, y, z, vx=vx, vy=vy, vz=vz)
            rclpy.spin_once(self, timeout_sec=0.05)

            dist = self._distance_to(x, y, z)
            if dist <= self.position_tolerance_m:
                self.get_logger().info(
                    f'Waypoint reached: dist={dist:.1f}m (tol={self.position_tolerance_m:.1f}m)'
                )
                return True

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

        # Publish initial setpoint burst (PX4 requires setpoints before OFFBOARD switch)
        self.get_logger().info('Publishing initial setpoints (FW path-following mode)')
        self._publish_setpoint_burst(100)

        # Switch to OFFBOARD + arm
        deadline = time.time() + self.preflight_timeout_sec
        last_mode_req = 0.0
        last_arm_req = 0.0
        used_force_arm = False

        while rclpy.ok() and time.time() < deadline:
            first_wp = self.waypoints[0]
            vx, vy, vz = self._compute_velocity_to_waypoint(*first_wp)
            self._publish_fw_setpoint(
                first_wp[0], first_wp[1], first_wp[2], vx=vx, vy=vy, vz=vz
            )

            mode_ok = self.current_state.mode == 'OFFBOARD'
            arm_ok = self.current_state.armed

            if not mode_ok:
                if time.time() - last_mode_req > 1.0 and self._set_mode_offboard():
                    self.get_logger().info('OFFBOARD mode set')
                    last_mode_req = time.time()
            elif not arm_ok:
                if time.time() - last_arm_req > 1.0:
                    elapsed = time.time() - (deadline - self.preflight_timeout_sec)
                    self.get_logger().info(
                        f'Waiting for PX4 pre-flight checks ({elapsed:.0f}s elapsed)...'
                    )
                    if self._arm():
                        self.get_logger().info('Vehicle armed')
                    elif self.force_arm_if_needed and elapsed > 15.0:
                        # Try force arm every attempt after 15s
                        self.get_logger().warn('Attempting force arm (SITL fallback)...')
                        if self._arm_force():
                            self.get_logger().warn('Force arm accepted!')
                    
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

        # Fly waypoints
        self.get_logger().info(f'Starting mission with {len(self.waypoints)} waypoints')
        start = time.time()
        per_wp_timeout = max(15.0, self.mission_timeout_sec / float(len(self.waypoints)))

        for idx, wp in enumerate(self.waypoints):
            x, y, z = wp
            self.get_logger().info(
                f'Waypoint {idx + 1}/{len(self.waypoints)} -> ({x}, {y}, {z})'
            )
            if not self._reach_waypoint(x, y, z, per_wp_timeout):
                self.get_logger().error(f'Waypoint timeout at index {idx}')
                return 1

        elapsed = time.time() - start
        self.get_logger().info(f'Mission complete in {elapsed:.1f}s')

        # Post-mission: loiter or RTL
        if self.loiter_after_mission:
            last_wp = self.waypoints[-1]
            self.get_logger().info(
                f'Loitering at last waypoint ({last_wp[0]}, {last_wp[1]}, {last_wp[2]})'
            )
            loiter_end = time.time() + 30.0
            while rclpy.ok() and time.time() < loiter_end:
                self._publish_fw_setpoint(
                    last_wp[0], last_wp[1], last_wp[2],
                    type_mask=12288,  # loiter setpoint
                )
                self._spin_sleep(self.dt)
        else:
            self.get_logger().info('Switching to RTL for landing')
            self._set_mode_rtl()

        return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FWMissionNode()
    code = 1
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(code)
