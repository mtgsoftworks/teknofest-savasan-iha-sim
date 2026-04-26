#!/usr/bin/env python3

import math
import time
from typing import List, Tuple

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class GeofenceNode(Node):
    def __init__(self) -> None:
        super().__init__('geofence_node')

        # Boundary: semicolon-separated x,y pairs (convex polygon in local frame)
        self.declare_parameter('boundary_corners', '-300,-200;300,-200;300,200;-300,200')
        # HSS zones: semicolon-separated x,y,radius entries
        self.declare_parameter('hss_zones', '300,0,50')
        self.declare_parameter('check_rate_hz', 5.0)
        self.declare_parameter('hss_penalty_per_sec', 5.0)
        self.declare_parameter('auto_rtl_on_breach', True)
        self.declare_parameter('auto_rtl_on_hss', False)
        self.declare_parameter('boundary_log_interval_sec', 3.0)
        self.declare_parameter('hss_log_interval_sec', 3.0)

        boundary_param = str(self.get_parameter('boundary_corners').value).strip()
        self.boundary_corners = self._parse_boundary(boundary_param)

        hss_param = str(self.get_parameter('hss_zones').value).strip()
        self.hss_zones = self._parse_hss(hss_param)

        self.check_rate_hz = float(self.get_parameter('check_rate_hz').value)
        self.hss_penalty_per_sec = float(self.get_parameter('hss_penalty_per_sec').value)
        self.auto_rtl_on_breach = bool(self.get_parameter('auto_rtl_on_breach').value)
        self.auto_rtl_on_hss = bool(self.get_parameter('auto_rtl_on_hss').value)
        self.boundary_log_interval_sec = float(
            self.get_parameter('boundary_log_interval_sec').value
        )
        self.hss_log_interval_sec = float(
            self.get_parameter('hss_log_interval_sec').value
        )

        self.current_state = State()
        self.current_pos = (0.0, 0.0, 0.0)

        qos_best_effort = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(State, '/mavros/state', self._state_cb, qos_best_effort)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self._pose_cb, qos_best_effort
        )

        self.geofence_pub = self.create_publisher(String, '/geofence/status', 10)
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self._hss_inside_time = 0.0
        self._last_check_time = 0.0
        self._total_penalty = 0.0
        self._breach_rtl_sent = False
        self._hss_rtl_sent = False
        self._was_boundary_breach = False
        self._was_in_hss = False
        self._last_boundary_log_time = 0.0
        self._last_hss_log_time = 0.0

    @staticmethod
    def _parse_boundary(text: str) -> List[Tuple[float, float]]:
        corners: List[Tuple[float, float]] = []
        for block in text.split(';'):
            part = block.strip()
            if not part:
                continue
            vals = [c.strip() for c in part.split(',')]
            if len(vals) != 2:
                continue
            try:
                corners.append((float(vals[0]), float(vals[1])))
            except ValueError:
                continue
        return corners

    @staticmethod
    def _parse_hss(text: str) -> List[Tuple[float, float, float]]:
        zones: List[Tuple[float, float, float]] = []
        for block in text.split(';'):
            part = block.strip()
            if not part:
                continue
            vals = [c.strip() for c in part.split(',')]
            if len(vals) != 3:
                continue
            try:
                zones.append((float(vals[0]), float(vals[1]), float(vals[2])))
            except ValueError:
                continue
        return zones

    def _state_cb(self, msg: State) -> None:
        self.current_state = msg

    def _pose_cb(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self.current_pos = (p.x, p.y, p.z)

    def _point_in_polygon(self, px: float, py: float, polygon: List[Tuple[float, float]]) -> bool:
        n = len(polygon)
        if n < 3:
            return False
        inside = False
        j = n - 1
        for i in range(n):
            xi, yi = polygon[i]
            xj, yj = polygon[j]
            if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi) + xi):
                inside = not inside
            j = i
        return inside

    def _point_in_hss(self, px: float, py: float) -> Tuple[bool, str]:
        for hx, hy, hr in self.hss_zones:
            dx = px - hx
            dy = py - hy
            if math.sqrt(dx * dx + dy * dy) <= hr:
                return True, f'HSS({hx},{hy},r={hr})'
        return False, ''

    def _set_mode_rtl(self) -> bool:
        if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            return False
        req = SetMode.Request()
        req.custom_mode = 'AUTO.RTL'
        future = self.set_mode_client.call_async(req)
        end = time.time() + 5.0
        while rclpy.ok() and time.time() < end and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.done() and future.result() is not None and future.result().mode_sent

    def check(self) -> None:
        now = time.time()
        if self._last_check_time > 0:
            dt = now - self._last_check_time
        else:
            dt = 0.0
        self._last_check_time = now

        px, py, pz = self.current_pos

        # Check boundary
        in_boundary = self._point_in_polygon(px, py, self.boundary_corners)
        boundary_breach = not in_boundary

        # Check HSS
        in_hss, hss_name = self._point_in_hss(px, py)

        # HSS penalty accumulation
        if in_hss and dt > 0:
            self._hss_inside_time += dt
            self._total_penalty += self.hss_penalty_per_sec * dt

        # Publish status
        status = String()
        parts = [f'pos=({px:.1f},{py:.1f},{pz:.1f})']
        if boundary_breach:
            parts.append('BOUNDARY_BREACH')
        else:
            parts.append('in_boundary')
        if in_hss:
            parts.append(f'IN_{hss_name}')
            parts.append(f'penalty={self._total_penalty:.0f}')
        else:
            parts.append(f'penalty={self._total_penalty:.0f}')
        status.data = ' | '.join(parts)
        self.geofence_pub.publish(status)

        # Auto RTL on boundary breach
        if boundary_breach and self.auto_rtl_on_breach and not self._breach_rtl_sent:
            self.get_logger().error(
                f'BOUNDARY BREACH at ({px:.1f},{py:.1f}) - sending RTL'
            )
            if self._set_mode_rtl():
                self._breach_rtl_sent = True
                self.get_logger().warn('RTL mode sent due to geofence breach')
            else:
                self.get_logger().error('Failed to send RTL mode')

        # Rate-limit repeated warning logs to prevent spam.
        if boundary_breach:
            should_log_boundary = (
                not self._was_boundary_breach
                or (now - self._last_boundary_log_time) >= self.boundary_log_interval_sec
            )
            if should_log_boundary:
                if self._was_boundary_breach:
                    self.get_logger().warn(f'STILL OUTSIDE BOUNDARY: ({px:.1f},{py:.1f})')
                else:
                    self.get_logger().error(f'OUTSIDE BOUNDARY: ({px:.1f},{py:.1f})')
                self._last_boundary_log_time = now

        elif in_hss:
            should_log_hss = (
                not self._was_in_hss
                or (now - self._last_hss_log_time) >= self.hss_log_interval_sec
            )
            if should_log_hss:
                if self._was_in_hss:
                    self.get_logger().warn(
                        f'STILL INSIDE {hss_name} - penalty: {self._total_penalty:.0f}'
                    )
                else:
                    self.get_logger().warn(
                        f'INSIDE {hss_name} - penalty: {self._total_penalty:.0f}'
                    )
                self._last_hss_log_time = now

        # Auto RTL on HSS violation
        if in_hss and self.auto_rtl_on_hss and not self._hss_rtl_sent:
            self.get_logger().error(
                f'HSS VIOLATION at ({px:.1f},{py:.1f}) in {hss_name} - sending RTL'
            )
            if self._set_mode_rtl():
                self._hss_rtl_sent = True
                self.get_logger().warn('RTL mode sent due to HSS violation')
            else:
                self.get_logger().error('Failed to send RTL mode for HSS violation')

        self._was_boundary_breach = boundary_breach
        self._was_in_hss = in_hss

    def run(self) -> int:
        self.get_logger().info(
            f'Geofence active: {len(self.boundary_corners)} corners, '
            f'{len(self.hss_zones)} HSS zones'
        )
        dt = 1.0 / self.check_rate_hz
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            self.check()
            time.sleep(dt)
        return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GeofenceNode()
    code = 1
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(code)
