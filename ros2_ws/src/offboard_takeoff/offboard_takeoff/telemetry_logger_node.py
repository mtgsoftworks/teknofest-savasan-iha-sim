#!/usr/bin/env python3

import csv
import math
import os
import time
from typing import Optional

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


class TelemetryLoggerNode(Node):
    def __init__(self) -> None:
        super().__init__('telemetry_logger_node')

        self.declare_parameter('log_dir', '/tmp/teknofest_logs')
        self.declare_parameter('log_rate_hz', 2.0)
        self.declare_parameter('mission_name', 'mission')

        log_dir = str(self.get_parameter('log_dir').value)
        mission_name = str(self.get_parameter('mission_name').value)
        self.log_rate_hz = float(self.get_parameter('log_rate_hz').value)

        os.makedirs(log_dir, exist_ok=True)
        ts = time.strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(log_dir, f'{mission_name}_{ts}.csv')

        self.fields = [
            'timestamp', 't_sec', 'lat', 'lon', 'alt_amsl',
            'local_x', 'local_y', 'local_z',
            'roll_rad', 'pitch_rad', 'yaw_rad',
            'speed_ms', 'battery_pct', 'voltage_v',
            'mode', 'armed', 'connected',
            'geofence_status',
        ]

        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.fields)
            writer.writeheader()

        self.get_logger().info(f'Telemetry log: {self.csv_path}')

        qos_best_effort = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.current_state = State()
        self.local_pose: Optional[PoseStamped] = None
        self.local_vel: Optional[TwistStamped] = None
        self.navsat: Optional[NavSatFix] = None
        self.battery: Optional[BatteryState] = None
        self.geofence_status: str = 'UNKNOWN'

        self.create_subscription(State, '/mavros/state', self._state_cb, qos_best_effort)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self._pose_cb, qos_best_effort
        )
        self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local',
            self._vel_cb, qos_best_effort
        )
        self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self._navsat_cb, qos_best_effort
        )
        self.create_subscription(
            BatteryState, '/mavros/battery', self._battery_cb, qos_best_effort
        )
        self.create_subscription(
            String, '/geofence/status', self._geofence_cb, 10
        )

        self._start_time = 0.0

    def _state_cb(self, msg: State) -> None:
        self.current_state = msg

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.local_pose = msg
        if self._start_time == 0.0:
            self._start_time = time.time()

    def _vel_cb(self, msg: TwistStamped) -> None:
        self.local_vel = msg

    def _navsat_cb(self, msg: NavSatFix) -> None:
        self.navsat = msg

    def _battery_cb(self, msg: BatteryState) -> None:
        self.battery = msg

    def _geofence_cb(self, msg: String) -> None:
        self.geofence_status = msg.data

    @staticmethod
    def _quat_to_euler(x: float, y: float, z: float, w: float):
        """Quaternion to roll, pitch, yaw (radians)."""
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        sinp = 2.0 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)
        return roll, pitch, yaw

    def _record_row(self) -> None:
        now = time.time()
        row = {
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(now)),
            't_sec': f'{now - self._start_time:.2f}' if self._start_time > 0 else '0.00',
            'lat': '0.0', 'lon': '0.0', 'alt_amsl': '0.0',
            'local_x': '0.0', 'local_y': '0.0', 'local_z': '0.0',
            'roll_rad': '0.0', 'pitch_rad': '0.0', 'yaw_rad': '0.0',
            'speed_ms': '0.0',
            'battery_pct': '0', 'voltage_v': '0.0',
            'mode': self.current_state.mode if self.current_state.mode else 'UNKNOWN',
            'armed': str(self.current_state.armed),
            'connected': str(self.current_state.connected),
            'geofence_status': self.geofence_status,
        }

        if self.local_pose is not None:
            p = self.local_pose.pose.position
            row['local_x'] = f'{p.x:.3f}'
            row['local_y'] = f'{p.y:.3f}'
            row['local_z'] = f'{p.z:.3f}'
            q = self.local_pose.pose.orientation
            roll, pitch, yaw = self._quat_to_euler(q.x, q.y, q.z, q.w)
            row['roll_rad'] = f'{roll:.4f}'
            row['pitch_rad'] = f'{pitch:.4f}'
            row['yaw_rad'] = f'{yaw:.4f}'

        if self.navsat is not None:
            row['lat'] = f'{self.navsat.latitude:.8f}'
            row['lon'] = f'{self.navsat.longitude:.8f}'
            row['alt_amsl'] = f'{self.navsat.altitude:.2f}'

        if self.local_vel is not None:
            v = self.local_vel.twist.linear
            speed = (v.x ** 2 + v.y ** 2 + v.z ** 2) ** 0.5
            row['speed_ms'] = f'{speed:.2f}'

        if self.battery is not None:
            pct = self.battery.percentage
            if pct >= 0:
                row['battery_pct'] = f'{pct * 100:.0f}'
            row['voltage_v'] = f'{self.battery.voltage:.2f}'

        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.fields)
            writer.writerow(row)

    def run(self) -> int:
        dt = 1.0 / self.log_rate_hz
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            self._record_row()
            time.sleep(dt)
        self.get_logger().info(f'Telemetry log saved: {self.csv_path}')
        return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TelemetryLoggerNode()
    code = 1
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(code)
