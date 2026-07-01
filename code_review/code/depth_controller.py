#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Imu
from rcl_interfaces.msg import SetParametersResult


ARMED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

CONTROL_SUB_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def quat_to_rotation_z_row(x: float, y: float, z: float, w: float):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return (0.0, 0.0, 1.0)
    x /= n
    y /= n
    z /= n
    w /= n

    return (
        2.0 * (x * z - w * y),
        2.0 * (y * z + w * x),
        1.0 - 2.0 * (x * x + y * y),
    )


class DepthController(Node):

    def __init__(self):
        super().__init__('depth_controller')

        # Parameters
        self.declare_parameter('depth_topic', '/depth_m')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('cmd_depth_topic', '/cmd_depth')
        self.declare_parameter('manual_wrench_topic', '/rov/wrench_manual')
        self.declare_parameter('heave_cmd_topic', '/ctrl/depth_heave')
        self.declare_parameter('depth_active_topic', '/ctrl/depth_active')
        self.declare_parameter('target_depth_status_topic', '/ctrl/depth_target')
        self.declare_parameter('depth_error_topic', '/ctrl/depth_error')
        self.declare_parameter('depth_rate_topic', '/ctrl/depth_rate')
        self.declare_parameter('armed_topic', '/rov/armed')
        self.declare_parameter('control_enabled', True)

        self.declare_parameter('kp_depth', 2.0)
        self.declare_parameter('ki_depth', 0.25)
        self.declare_parameter('kd_depth', 0.8)
        self.declare_parameter('max_heave', 2.0)
        self.declare_parameter('max_upward_heave', 0.8)
        self.declare_parameter('upward_heave_cmd_sign', -1.0)
        self.declare_parameter('depth_integral_limit', 2.0)

        self.declare_parameter('manual_heave_override_threshold', 0.05)
        self.declare_parameter('manual_wrench_timeout_sec', 0.5)
        self.declare_parameter('manual_heave_release_target_offset', 0.10)
        self.declare_parameter('capture_initial_depth_target', True)
        self.declare_parameter('pilot_depth_rate_enabled', True)
        self.declare_parameter('max_pilot_depth_rate', 0.35)
        self.declare_parameter('pilot_depth_rate_sign', -1.0)
        self.declare_parameter('min_target_depth', 0.0)
        self.declare_parameter('max_target_depth', 100.0)

        self.declare_parameter('heave_cmd_sign', -1.0)
        self.declare_parameter('depth_rate_alpha', 0.10)
        self.declare_parameter('max_heave_delta_per_cycle', 0.8)
        # depth 센서가 IMU보다 앞에 0.20 m 있으면
        # depth_sensor_offset_x = 0.20

        # depth 센서가 IMU보다 오른쪽에 0.10 m 있으면
        # depth_sensor_offset_y = -0.10

        # depth 센서가 IMU보다 아래에 0.05 m 있으면
        # depth_sensor_offset_z = -0.05

        self.declare_parameter('depth_sensor_offset_x', -0.3)
        self.declare_parameter('depth_sensor_offset_y', 0.0)
        self.declare_parameter('depth_sensor_offset_z', 0.0)
        self.declare_parameter('depth_sensor_offset_compensation_enabled', True)

        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.cmd_depth_topic = self.get_parameter('cmd_depth_topic').get_parameter_value().string_value
        self.manual_wrench_topic = self.get_parameter('manual_wrench_topic').get_parameter_value().string_value
        self.heave_cmd_topic = self.get_parameter('heave_cmd_topic').get_parameter_value().string_value
        self.depth_active_topic = self.get_parameter(
            'depth_active_topic').get_parameter_value().string_value
        self.target_depth_status_topic = self.get_parameter(
            'target_depth_status_topic').get_parameter_value().string_value
        self.depth_error_topic = self.get_parameter(
            'depth_error_topic').get_parameter_value().string_value
        self.depth_rate_topic = self.get_parameter(
            'depth_rate_topic').get_parameter_value().string_value
        self.armed_topic = self.get_parameter('armed_topic').get_parameter_value().string_value
        self.control_enabled = self.get_parameter('control_enabled').get_parameter_value().bool_value

        self.kp_depth = self.get_parameter('kp_depth').get_parameter_value().double_value
        self.ki_depth = self.get_parameter('ki_depth').get_parameter_value().double_value
        self.kd_depth = self.get_parameter('kd_depth').get_parameter_value().double_value
        self.max_heave = self.get_parameter('max_heave').get_parameter_value().double_value
        self.max_upward_heave = self.get_parameter(
            'max_upward_heave').get_parameter_value().double_value
        self.upward_heave_cmd_sign = self.get_parameter(
            'upward_heave_cmd_sign').get_parameter_value().double_value
        self.depth_integral_limit = self.get_parameter(
            'depth_integral_limit').get_parameter_value().double_value

        self.manual_heave_override_threshold = self.get_parameter(
            'manual_heave_override_threshold').get_parameter_value().double_value
        self.manual_wrench_timeout_sec = self.get_parameter(
            'manual_wrench_timeout_sec').get_parameter_value().double_value
        self.manual_heave_release_target_offset = self.get_parameter(
            'manual_heave_release_target_offset').get_parameter_value().double_value
        self.capture_initial_depth_target = self.get_parameter(
            'capture_initial_depth_target').get_parameter_value().bool_value
        self.pilot_depth_rate_enabled = self.get_parameter(
            'pilot_depth_rate_enabled').get_parameter_value().bool_value
        self.max_pilot_depth_rate = self.get_parameter(
            'max_pilot_depth_rate').get_parameter_value().double_value
        self.pilot_depth_rate_sign = self.get_parameter(
            'pilot_depth_rate_sign').get_parameter_value().double_value
        self.min_target_depth = self.get_parameter(
            'min_target_depth').get_parameter_value().double_value
        self.max_target_depth = self.get_parameter(
            'max_target_depth').get_parameter_value().double_value
        self.heave_cmd_sign = self.get_parameter(
            'heave_cmd_sign').get_parameter_value().double_value
        self.depth_rate_alpha = self.get_parameter(
            'depth_rate_alpha').get_parameter_value().double_value
        self.max_heave_delta_per_cycle = self.get_parameter(
            'max_heave_delta_per_cycle').get_parameter_value().double_value
        self.depth_sensor_offset_x = self.get_parameter(
            'depth_sensor_offset_x').get_parameter_value().double_value
        self.depth_sensor_offset_y = self.get_parameter(
            'depth_sensor_offset_y').get_parameter_value().double_value
        self.depth_sensor_offset_z = self.get_parameter(
            'depth_sensor_offset_z').get_parameter_value().double_value
        self.depth_sensor_offset_compensation_enabled = self.get_parameter(
            'depth_sensor_offset_compensation_enabled').get_parameter_value().bool_value

        # State
        self.target_depth = 0.0
        self.target_initialized = False

        self.current_depth = None
        self.prev_depth = None
        self.prev_time = None
        self.have_imu = False
        self.imu_z_row = (0.0, 0.0, 1.0)

        self.manual_heave = 0.0
        self.manual_heave_active = False
        self.prev_manual_heave_active = False
        self.last_manual_wrench_time = None

        self.filtered_depth_rate = 0.0
        self.depth_integral = 0.0
        self.prev_heave_cmd = 0.0

        self.armed = False
        self.prev_armed = False
        self.armed_received = False

        # Pub/Sub
        self.depth_sub = self.create_subscription(
            Float64,
            self.depth_topic,
            self.depth_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            CONTROL_SUB_QOS
        )

        self.cmd_depth_sub = self.create_subscription(
            Float64,
            self.cmd_depth_topic,
            self.cmd_depth_callback,
            10
        )

        self.manual_wrench_sub = self.create_subscription(
            Wrench,
            self.manual_wrench_topic,
            self.manual_wrench_callback,
            CONTROL_SUB_QOS
        )

        self.armed_sub = self.create_subscription(
            Bool,
            self.armed_topic,
            self.armed_callback,
            ARMED_QOS
        )

        self.heave_pub = self.create_publisher(
            Float64,
            self.heave_cmd_topic,
            10
        )
        self.depth_active_pub = self.create_publisher(
            Bool,
            self.depth_active_topic,
            10
        )
        self.target_depth_pub = self.create_publisher(
            Float64,
            self.target_depth_status_topic,
            10
        )
        self.depth_error_pub = self.create_publisher(
            Float64,
            self.depth_error_topic,
            10
        )
        self.depth_rate_pub = self.create_publisher(
            Float64,
            self.depth_rate_topic,
            10
        )

        self.add_on_set_parameters_callback(self.on_parameter_update)

        self.get_logger().info('DepthController initialized')
        self.get_logger().info(f'  depth_topic                     = {self.depth_topic}')
        self.get_logger().info(f'  imu_topic                       = {self.imu_topic}')
        self.get_logger().info(f'  cmd_depth_topic                 = {self.cmd_depth_topic}')
        self.get_logger().info(f'  manual_wrench_topic             = {self.manual_wrench_topic}')
        self.get_logger().info(f'  heave_cmd_topic                 = {self.heave_cmd_topic}')
        self.get_logger().info(f'  depth_active_topic              = {self.depth_active_topic}')
        self.get_logger().info(f'  target_depth_status_topic       = {self.target_depth_status_topic}')
        self.get_logger().info(f'  depth_error_topic               = {self.depth_error_topic}')
        self.get_logger().info(f'  depth_rate_topic                = {self.depth_rate_topic}')
        self.get_logger().info(f'  armed_topic                     = {self.armed_topic}')
        self.get_logger().info(f'  control_enabled                 = {self.control_enabled}')
        self.get_logger().info(f'  kp_depth                        = {self.kp_depth}')
        self.get_logger().info(f'  ki_depth                        = {self.ki_depth}')
        self.get_logger().info(f'  kd_depth                        = {self.kd_depth}')
        self.get_logger().info(f'  max_heave                       = {self.max_heave}')
        self.get_logger().info(f'  max_upward_heave                = {self.max_upward_heave}')
        self.get_logger().info(f'  upward_heave_cmd_sign           = {self.upward_heave_cmd_sign}')
        self.get_logger().info(f'  depth_integral_limit            = {self.depth_integral_limit}')
        self.get_logger().info(f'  manual_heave_override_threshold = {self.manual_heave_override_threshold}')
        self.get_logger().info(f'  manual_wrench_timeout_sec       = {self.manual_wrench_timeout_sec}')
        self.get_logger().info(
            f'  manual_heave_release_offset     = '
            f'{self.manual_heave_release_target_offset}'
        )
        self.get_logger().info(f'  capture_initial_depth_target    = {self.capture_initial_depth_target}')
        self.get_logger().info(f'  pilot_depth_rate_enabled        = {self.pilot_depth_rate_enabled}')
        self.get_logger().info(f'  max_pilot_depth_rate            = {self.max_pilot_depth_rate}')
        self.get_logger().info(f'  pilot_depth_rate_sign           = {self.pilot_depth_rate_sign}')
        self.get_logger().info(f'  min_target_depth                = {self.min_target_depth}')
        self.get_logger().info(f'  max_target_depth                = {self.max_target_depth}')
        self.get_logger().info(f'  heave_cmd_sign                  = {self.heave_cmd_sign}')
        self.get_logger().info(f'  depth_rate_alpha                = {self.depth_rate_alpha}')
        self.get_logger().info(f'  max_heave_delta_per_cycle       = {self.max_heave_delta_per_cycle}')
        self.get_logger().info(
            f'  depth_sensor_offset_xyz         = '
            f'({self.depth_sensor_offset_x:.3f}, '
            f'{self.depth_sensor_offset_y:.3f}, '
            f'{self.depth_sensor_offset_z:.3f}) m'
        )
        self.get_logger().info(
            f'  depth_sensor_offset_comp        = '
            f'{self.depth_sensor_offset_compensation_enabled}'
        )

    def armed_callback(self, msg: Bool):
        self.prev_armed = self.armed
        self.armed = bool(msg.data)
        self.armed_received = True

        # rising edge: DISARMED -> ARMED
        if (not self.prev_armed) and self.armed:
            if self.current_depth is not None:
                self.target_depth = self.current_depth
                self.target_initialized = True
                self.prev_heave_cmd = 0.0
                self.depth_integral = 0.0
                self.get_logger().info(
                    f'ARM rising edge -> captured current depth as new target: '
                    f'{self.target_depth:.3f} m'
                )

        # falling edge: ARMED -> DISARMED
        if self.prev_armed and (not self.armed):
            self.prev_heave_cmd = 0.0
            self.depth_integral = 0.0
            self.get_logger().info('DISARM -> depth controller output reset to zero')
            self.publish_depth_active(False)

    def publish_depth_active(self, active: bool):
        msg = Bool()
        msg.data = bool(active)
        self.depth_active_pub.publish(msg)

    def depth_control_is_active(self) -> bool:
        return (
            self.armed_received and
            self.armed and
            self.control_enabled and
            self.target_initialized and
            self.current_depth is not None
        )

    def clamp_target_depth(self):
        lo = min(float(self.min_target_depth), float(self.max_target_depth))
        hi = max(float(self.min_target_depth), float(self.max_target_depth))
        if self.target_depth < lo:
            self.target_depth = lo
        elif self.target_depth > hi:
            self.target_depth = hi

    def capture_manual_release_target(self, reason: str):
        self.target_depth = self.current_depth + self.manual_heave_release_target_offset
        self.clamp_target_depth()
        self.target_initialized = True
        self.depth_integral = 0.0
        self.get_logger().info(
            f'{reason} -> captured depth target with release offset: '
            f'current={self.current_depth:.3f} m, '
            f'offset={self.manual_heave_release_target_offset:+.3f} m, '
            f'target={self.target_depth:.3f} m'
        )

    def _set_control_enabled(self, enabled: bool):
        prev = self.control_enabled
        self.control_enabled = bool(enabled)

        if self.control_enabled:
            if self.current_depth is not None:
                self.target_depth = self.current_depth
                self.clamp_target_depth()
                self.target_initialized = True
                self.get_logger().info(
                    f'Depth control enabled; captured current depth as target: '
                    f'{self.target_depth:.3f} m'
                )
            else:
                self.get_logger().warn(
                    'Depth control enabled, but no depth sample has been received yet'
                )
            self.prev_heave_cmd = 0.0
            self.depth_integral = 0.0
        elif (not self.control_enabled) and prev:
            self.prev_heave_cmd = 0.0
            self.depth_integral = 0.0
            out = Float64()
            out.data = 0.0
            self.heave_pub.publish(out)
            self.publish_depth_active(False)
            self.get_logger().info('Depth control disabled; publishing zero heave command')

    def cmd_depth_callback(self, msg: Float64):
        self.target_depth = msg.data
        self.clamp_target_depth()
        self.target_initialized = True
        self.get_logger().info(f'Updated target depth from /cmd_depth: {self.target_depth:.3f} m')

    def imu_callback(self, msg: Imu):
        self.imu_z_row = quat_to_rotation_z_row(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        self.have_imu = True

    def compensate_depth_sensor_offset(self, sensor_depth: float) -> float:
        if not self.depth_sensor_offset_compensation_enabled:
            return sensor_depth
        if not self.have_imu:
            return sensor_depth

        zx, zy, zz = self.imu_z_row
        sensor_z_in_world = (
            zx * self.depth_sensor_offset_x +
            zy * self.depth_sensor_offset_y +
            zz * self.depth_sensor_offset_z
        )
        return sensor_depth + sensor_z_in_world

    def manual_wrench_callback(self, msg: Wrench):
        self.manual_heave = msg.force.z
        self.last_manual_wrench_time = self.get_clock().now()

        self.prev_manual_heave_active = self.manual_heave_active
        self.manual_heave_active = abs(self.manual_heave) > self.manual_heave_override_threshold

        # Edge detect: manual heave released
        if self.prev_manual_heave_active and not self.manual_heave_active:
            if self.current_depth is not None and self.armed:
                self.capture_manual_release_target('Manual heave released')

    def manual_wrench_is_fresh(self, now) -> bool:
        if self.manual_wrench_timeout_sec <= 0.0:
            return True
        if self.last_manual_wrench_time is None:
            return False

        age = (now - self.last_manual_wrench_time).nanoseconds * 1e-9
        return age <= self.manual_wrench_timeout_sec

    def clear_stale_manual_heave(self, now):
        if self.manual_wrench_is_fresh(now):
            return

        if self.manual_heave_active and self.current_depth is not None and self.armed:
            self.capture_manual_release_target('Manual heave timed out')

        self.manual_heave = 0.0
        self.prev_manual_heave_active = False
        self.manual_heave_active = False

    def depth_callback(self, msg: Float64):
        now = self.get_clock().now()
        sensor_depth = float(msg.data)
        depth = self.compensate_depth_sensor_offset(sensor_depth)
        self.current_depth = depth
        self.clear_stale_manual_heave(now)

        # startup initial capture
        if self.capture_initial_depth_target and not self.target_initialized:
            self.target_depth = depth
            self.clamp_target_depth()
            self.target_initialized = True
            self.get_logger().info(
                f'Initial depth target captured: {self.target_depth:.3f} m'
            )

        raw_depth_rate = 0.0
        dt = 0.0
        if self.prev_depth is not None and self.prev_time is not None:
            dt = (now - self.prev_time).nanoseconds * 1e-9
            if dt > 1e-6:
                raw_depth_rate = (depth - self.prev_depth) / dt

        self.filtered_depth_rate = (
            self.depth_rate_alpha * raw_depth_rate +
            (1.0 - self.depth_rate_alpha) * self.filtered_depth_rate
        )
        depth_rate = self.filtered_depth_rate

        heave_cmd = 0.0
        raw_cmd = 0.0
        error = 0.0

        active = self.depth_control_is_active()
        self.publish_depth_active(active)

        # armed 신호를 못 받았거나 disarm이면 무조건 0
        if (not self.armed_received) or (not self.armed):
            heave_cmd = 0.0
            self.depth_integral = 0.0

        elif not self.control_enabled:
            heave_cmd = 0.0
            self.depth_integral = 0.0

        elif self.target_initialized:
            if self.manual_heave_active and self.pilot_depth_rate_enabled and dt > 1e-6:
                depth_rate_cmd = (
                    self.pilot_depth_rate_sign *
                    self.manual_heave *
                    abs(self.max_pilot_depth_rate)
                )
                self.target_depth += depth_rate_cmd * dt
                self.clamp_target_depth()
                self.depth_integral = 0.0

            error = self.target_depth - depth
            if dt > 1e-6 and not self.manual_heave_active:
                self.depth_integral += error * dt
                limit = abs(self.depth_integral_limit)
                if self.depth_integral > limit:
                    self.depth_integral = limit
                elif self.depth_integral < -limit:
                    self.depth_integral = -limit

            raw_cmd = (
                self.kp_depth * error +
                self.ki_depth * self.depth_integral -
                self.kd_depth * depth_rate
            )
            heave_cmd = self.heave_cmd_sign * raw_cmd

            if heave_cmd > self.max_heave:
                heave_cmd = self.max_heave
            elif heave_cmd < -self.max_heave:
                heave_cmd = -self.max_heave

            upward_sign = 1.0 if self.upward_heave_cmd_sign >= 0.0 else -1.0
            max_upward = max(0.0, abs(self.max_upward_heave))
            if heave_cmd * upward_sign > max_upward:
                heave_cmd = upward_sign * max_upward

            delta = heave_cmd - self.prev_heave_cmd
            if delta > self.max_heave_delta_per_cycle:
                heave_cmd = self.prev_heave_cmd + self.max_heave_delta_per_cycle
            elif delta < -self.max_heave_delta_per_cycle:
                heave_cmd = self.prev_heave_cmd - self.max_heave_delta_per_cycle

        out = Float64()
        out.data = heave_cmd
        self.heave_pub.publish(out)

        target_msg = Float64()
        target_msg.data = self.target_depth
        self.target_depth_pub.publish(target_msg)

        error_msg = Float64()
        error_msg.data = error
        self.depth_error_pub.publish(error_msg)

        rate_msg = Float64()
        rate_msg.data = depth_rate
        self.depth_rate_pub.publish(rate_msg)

        self.prev_heave_cmd = heave_cmd
        self.prev_depth = depth
        self.prev_time = now

        # self.get_logger().debug(
        #     f'armed={self.armed}, depth={depth:.3f}, target={self.target_depth:.3f}, '
        #     f'error={error:.3f}, raw_depth_rate={raw_depth_rate:.3f}, '
        #     f'filtered_depth_rate={depth_rate:.3f}, raw_cmd={raw_cmd:.3f}, '
        #     f'integral={self.depth_integral:.3f}, heave_cmd={heave_cmd:.3f}, '
        #     f'manual_heave={self.manual_heave:.3f}, '
        #     f'manual_active={self.manual_heave_active}'
        # )

    def on_parameter_update(self, params):
        try:
            for p in params:
                if p.name == 'kp_depth':
                    self.kp_depth = float(p.value)
                elif p.name == 'ki_depth':
                    self.ki_depth = float(p.value)
                elif p.name == 'kd_depth':
                    self.kd_depth = float(p.value)
                elif p.name == 'max_heave':
                    self.max_heave = float(p.value)
                elif p.name == 'max_upward_heave':
                    self.max_upward_heave = float(p.value)
                elif p.name == 'upward_heave_cmd_sign':
                    self.upward_heave_cmd_sign = float(p.value)
                elif p.name == 'depth_integral_limit':
                    self.depth_integral_limit = float(p.value)
                    limit = abs(self.depth_integral_limit)
                    if self.depth_integral > limit:
                        self.depth_integral = limit
                    elif self.depth_integral < -limit:
                        self.depth_integral = -limit
                elif p.name == 'manual_heave_override_threshold':
                    self.manual_heave_override_threshold = float(p.value)
                elif p.name == 'manual_wrench_timeout_sec':
                    self.manual_wrench_timeout_sec = float(p.value)
                elif p.name == 'manual_heave_release_target_offset':
                    self.manual_heave_release_target_offset = float(p.value)
                elif p.name == 'capture_initial_depth_target':
                    self.capture_initial_depth_target = bool(p.value)
                elif p.name == 'pilot_depth_rate_enabled':
                    self.pilot_depth_rate_enabled = bool(p.value)
                elif p.name == 'max_pilot_depth_rate':
                    self.max_pilot_depth_rate = float(p.value)
                elif p.name == 'pilot_depth_rate_sign':
                    self.pilot_depth_rate_sign = float(p.value)
                elif p.name == 'min_target_depth':
                    self.min_target_depth = float(p.value)
                    self.clamp_target_depth()
                elif p.name == 'max_target_depth':
                    self.max_target_depth = float(p.value)
                    self.clamp_target_depth()
                elif p.name == 'control_enabled':
                    self._set_control_enabled(bool(p.value))
                elif p.name == 'heave_cmd_sign':
                    self.heave_cmd_sign = float(p.value)
                elif p.name == 'depth_rate_alpha':
                    self.depth_rate_alpha = float(p.value)
                elif p.name == 'max_heave_delta_per_cycle':
                    self.max_heave_delta_per_cycle = float(p.value)
                elif p.name == 'depth_sensor_offset_x':
                    self.depth_sensor_offset_x = float(p.value)
                elif p.name == 'depth_sensor_offset_y':
                    self.depth_sensor_offset_y = float(p.value)
                elif p.name == 'depth_sensor_offset_z':
                    self.depth_sensor_offset_z = float(p.value)
                elif p.name == 'depth_sensor_offset_compensation_enabled':
                    self.depth_sensor_offset_compensation_enabled = bool(p.value)

            self.get_logger().info('Depth parameters updated at runtime')
            return SetParametersResult(successful=True)
        except Exception as e:
            return SetParametersResult(successful=False, reason=str(e))


def main(args=None):
    rclpy.init(args=args)
    node = DepthController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
