#!/usr/bin/env python3
import math
from typing import Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Wrench, Vector3
from rcl_interfaces.msg import SetParametersResult


CONTROL_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def vec_norm(x: float, y: float, z: float) -> float:
    return math.sqrt(x * x + y * y + z * z)


def quat_normalize(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / n, y / n, z / n, w / n)


def quat_conj(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_mul(a: Tuple[float, float, float, float],
             b: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_to_rpy(x: float, y: float, z: float, w: float):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return quat_normalize((qx, qy, qz, qw))


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class AttitudeController(Node):

    def __init__(self):
        super().__init__('attitude_controller')

        # Topics / rate
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('manual_wrench_topic', '/rov/wrench_manual')
        self.declare_parameter('output_torque_topic', '/ctrl/attitude_torque')
        self.declare_parameter('cmd_attitude_topic', '/cmd_attitude')
        self.declare_parameter('cmd_attitude_trim_topic', '/cmd_attitude_trim')
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('control_enabled', True)

        # Gains
        self.declare_parameter('kp_roll', 0.50)
        self.declare_parameter('kd_roll', 0.0)
        self.declare_parameter('ki_roll', 0.04)
        self.declare_parameter('kp_pitch', 0.45)
        self.declare_parameter('kd_pitch', 0.08)
        self.declare_parameter('ki_pitch', 0.04)
        self.declare_parameter('kp_yaw', 0.4)
        self.declare_parameter('kd_yaw', 0.12)

        # Limits
        self.declare_parameter('tx_limit', 0.16)
        self.declare_parameter('ty_limit', 0.16)
        self.declare_parameter('tz_limit', 0.08)
        self.declare_parameter('rp_torque_slew_rate', 0.8)

        # Behavior
        self.declare_parameter('capture_initial_target', True)
        self.declare_parameter('level_roll_pitch_target', True)
        self.declare_parameter('target_roll_deg', 0.0)
        self.declare_parameter('target_pitch_deg', 0.0)
        self.declare_parameter('roll_trim_deg', 0.0)
        self.declare_parameter('pitch_trim_deg', 0.0)
        self.declare_parameter('heave_protect_threshold', 0.08)
        self.declare_parameter('strong_heave_threshold', 0.20)
        self.declare_parameter('xy_motion_protect_threshold', 0.05)
        self.declare_parameter('strong_xy_motion_threshold', 0.20)
        self.declare_parameter('rp_scale_when_xy_motion', 0.45)
        self.declare_parameter('rp_scale_when_strong_xy_motion', 0.25)
        self.declare_parameter('xy_soft_trim_ignore_forward', True)
        self.declare_parameter('forward_surge_sign', 1.0)
        self.declare_parameter('yaw_scale_when_heave', 1.0)
        self.declare_parameter('yaw_scale_when_strong_heave', 0.7)
        self.declare_parameter('max_body_rate_for_control', 1.5)
        self.declare_parameter('large_tilt_disable_deg', 85.0)
        self.declare_parameter('torque_deadband', 1e-4)
        self.declare_parameter('roll_pitch_error_deadband_deg', 1.0)
        self.declare_parameter('roll_integral_limit', 0.20)
        self.declare_parameter('orientation_filter_enabled', True)
        self.declare_parameter('orientation_filter_measurement_alpha', 0.08)
        self.declare_parameter('orientation_filter_max_correction_rate_deg', 45.0)
        self.declare_parameter('translation_tilt_ff_enabled', False)
        self.declare_parameter('translation_pitch_ff_gain', 0.0)
        self.declare_parameter('translation_roll_ff_gain', 0.0)
        self.declare_parameter('translation_tilt_ff_max', 0.08)
        self.declare_parameter('translation_tilt_ff_deadband', 0.03)

        # Yaw manual override / hold update
        self.declare_parameter('yaw_hold_enabled', True)
        self.declare_parameter('yaw_manual_override_threshold', 0.02)
        self.declare_parameter('capture_yaw_target_on_release', True)
        self.declare_parameter('yaw_hold_settle_time_sec', 0.25)
        self.declare_parameter('yaw_error_deadband_deg', 0.5)

        # Load parameters
        imu_topic = self.get_parameter('imu_topic').value
        manual_wrench_topic = self.get_parameter('manual_wrench_topic').value
        output_torque_topic = self.get_parameter('output_torque_topic').value
        cmd_attitude_topic = self.get_parameter('cmd_attitude_topic').value
        cmd_attitude_trim_topic = self.get_parameter('cmd_attitude_trim_topic').value
        control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.control_enabled = bool(self.get_parameter('control_enabled').value)

        self.kp_roll = float(self.get_parameter('kp_roll').value)
        self.kd_roll = float(self.get_parameter('kd_roll').value)
        self.ki_roll = float(self.get_parameter('ki_roll').value)
        self.kp_pitch = float(self.get_parameter('kp_pitch').value)
        self.kd_pitch = float(self.get_parameter('kd_pitch').value)
        self.ki_pitch = float(self.get_parameter('ki_pitch').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.kd_yaw = float(self.get_parameter('kd_yaw').value)

        self.tx_limit = float(self.get_parameter('tx_limit').value)
        self.ty_limit = float(self.get_parameter('ty_limit').value)
        self.tz_limit = float(self.get_parameter('tz_limit').value)
        self.rp_torque_slew_rate = float(self.get_parameter('rp_torque_slew_rate').value)

        self.capture_initial_target = bool(self.get_parameter('capture_initial_target').value)
        self.level_roll_pitch_target = bool(self.get_parameter('level_roll_pitch_target').value)
        self.target_roll_deg = float(self.get_parameter('target_roll_deg').value)
        self.target_pitch_deg = float(self.get_parameter('target_pitch_deg').value)
        self.roll_trim_deg = float(self.get_parameter('roll_trim_deg').value)
        self.pitch_trim_deg = float(self.get_parameter('pitch_trim_deg').value)

        self.heave_protect_threshold = float(self.get_parameter('heave_protect_threshold').value)
        self.strong_heave_threshold = float(self.get_parameter('strong_heave_threshold').value)
        self.xy_motion_protect_threshold = float(
            self.get_parameter('xy_motion_protect_threshold').value
        )
        self.strong_xy_motion_threshold = float(
            self.get_parameter('strong_xy_motion_threshold').value
        )
        self.rp_scale_when_xy_motion = float(
            self.get_parameter('rp_scale_when_xy_motion').value
        )
        self.rp_scale_when_strong_xy_motion = float(
            self.get_parameter('rp_scale_when_strong_xy_motion').value
        )
        self.xy_soft_trim_ignore_forward = bool(
            self.get_parameter('xy_soft_trim_ignore_forward').value
        )
        self.forward_surge_sign = float(self.get_parameter('forward_surge_sign').value)
        self.yaw_scale_when_heave = float(self.get_parameter('yaw_scale_when_heave').value)
        self.yaw_scale_when_strong_heave = float(self.get_parameter('yaw_scale_when_strong_heave').value)

        self.max_body_rate_for_control = float(self.get_parameter('max_body_rate_for_control').value)
        self.large_tilt_disable_deg = float(self.get_parameter('large_tilt_disable_deg').value)
        self.large_tilt_disable_rad = math.radians(self.large_tilt_disable_deg)

        self.torque_deadband = float(self.get_parameter('torque_deadband').value)
        self.roll_pitch_error_deadband = math.radians(
            float(self.get_parameter('roll_pitch_error_deadband_deg').value)
        )
        self.roll_integral_limit = float(self.get_parameter('roll_integral_limit').value)
        self.orientation_filter_enabled = bool(
            self.get_parameter('orientation_filter_enabled').value
        )
        self.orientation_filter_measurement_alpha = float(
            self.get_parameter('orientation_filter_measurement_alpha').value
        )
        self.orientation_filter_max_correction_rate = math.radians(
            float(self.get_parameter('orientation_filter_max_correction_rate_deg').value)
        )
        self.translation_tilt_ff_enabled = bool(
            self.get_parameter('translation_tilt_ff_enabled').value
        )
        self.translation_pitch_ff_gain = float(
            self.get_parameter('translation_pitch_ff_gain').value
        )
        self.translation_roll_ff_gain = float(
            self.get_parameter('translation_roll_ff_gain').value
        )
        self.translation_tilt_ff_max = float(
            self.get_parameter('translation_tilt_ff_max').value
        )
        self.translation_tilt_ff_deadband = float(
            self.get_parameter('translation_tilt_ff_deadband').value
        )

        self.yaw_hold_enabled = bool(self.get_parameter('yaw_hold_enabled').value)
        self.yaw_manual_override_threshold = float(
            self.get_parameter('yaw_manual_override_threshold').value
        )
        self.capture_yaw_target_on_release = bool(
            self.get_parameter('capture_yaw_target_on_release').value
        )
        self.yaw_hold_settle_time_sec = float(
            self.get_parameter('yaw_hold_settle_time_sec').value
        )
        self.yaw_error_deadband = math.radians(
            float(self.get_parameter('yaw_error_deadband_deg').value)
        )

        # State
        self.have_imu = False
        self.target_initialized = False

        self.q_current = (0.0, 0.0, 0.0, 1.0)
        self.q_target = (0.0, 0.0, 0.0, 1.0)

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.control_roll = 0.0
        self.control_pitch = 0.0
        self.control_attitude_filter_initialized = False

        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.target_yaw = 0.0

        self.p_rate = 0.0
        self.q_rate = 0.0
        self.r_rate = 0.0

        self.manual_wrench = Wrench()
        self.yaw_manual_active = False
        self.prev_yaw_manual_active = False
        self.last_manual_yaw_input_time = None
        self._log_counter = 0
        self.last_control_time = None
        self.roll_integral = 0.0
        self.pitch_integral = 0.0
        self.prev_tx_ctrl = 0.0
        self.prev_ty_ctrl = 0.0

        # If initial capture is disabled, level attitude (0,0) is used immediately.
        if not self.capture_initial_target:
            self._update_target_quaternion()

        # Subs / pubs
        self.sub_imu = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            10
        )

        self.sub_manual = self.create_subscription(
            Wrench,
            manual_wrench_topic,
            self.manual_wrench_callback,
            CONTROL_QOS
        )

        self.sub_cmd_att = self.create_subscription(
            Vector3,
            cmd_attitude_topic,
            self.cmd_attitude_callback,
            CONTROL_QOS
        )
        self.sub_cmd_att_trim = self.create_subscription(
            Vector3,
            cmd_attitude_trim_topic,
            self.cmd_attitude_trim_callback,
            CONTROL_QOS
        )

        self.pub_torque = self.create_publisher(
            Wrench,
            output_torque_topic,
            10
        )

        self.timer = self.create_timer(1.0 / control_rate_hz, self.control_loop)

        self.add_on_set_parameters_callback(self.on_parameter_update)

        self.get_logger().info('AttitudeController initialized')
        self.get_logger().info(f'  imu_topic={imu_topic}')
        self.get_logger().info(f'  manual_wrench_topic={manual_wrench_topic}')
        self.get_logger().info(f'  output_torque_topic={output_torque_topic}')
        self.get_logger().info(f'  cmd_attitude_topic={cmd_attitude_topic}')
        self.get_logger().info(f'  cmd_attitude_trim_topic={cmd_attitude_trim_topic}')
        self.get_logger().info(f'  control_rate_hz={control_rate_hz}')
        self.get_logger().info(f'  control_enabled={self.control_enabled}')
        self.get_logger().info(f'  capture_initial_target={self.capture_initial_target}')
        self.get_logger().info(f'  roll_trim_deg={self.roll_trim_deg:.2f}')
        self.get_logger().info(f'  pitch_trim_deg={self.pitch_trim_deg:.2f}')
        self.get_logger().info(f'  yaw_hold_enabled={self.yaw_hold_enabled}')
        self.get_logger().info(f'  yaw_manual_override_threshold={self.yaw_manual_override_threshold}')
        self.get_logger().info(f'  capture_yaw_target_on_release={self.capture_yaw_target_on_release}')
        self.get_logger().info(f'  yaw_hold_settle_time_sec={self.yaw_hold_settle_time_sec}')
        self.get_logger().info(f'  yaw_error_deadband_deg={math.degrees(self.yaw_error_deadband):.2f}')
        self.get_logger().info(
            f'  roll_pitch_error_deadband_deg={math.degrees(self.roll_pitch_error_deadband):.2f}'
        )
        self.get_logger().info(
            f'  orientation_filter={self.orientation_filter_enabled}, '
            f'measurement_alpha={self.orientation_filter_measurement_alpha:.3f}, '
            f'max_correction_rate_deg={math.degrees(self.orientation_filter_max_correction_rate):.1f}'
        )
        self.get_logger().info(
            f'  xy_soft_trim_ignore_forward={self.xy_soft_trim_ignore_forward}, '
            f'forward_surge_sign={self.forward_surge_sign:.1f}, '
            f'xy_threshold={self.xy_motion_protect_threshold:.3f}, '
            f'strong_xy_threshold={self.strong_xy_motion_threshold:.3f}'
        )
        self.get_logger().info(
            f'  translation_tilt_ff={self.translation_tilt_ff_enabled}, '
            f'pitch_gain={self.translation_pitch_ff_gain:.3f}, '
            f'roll_gain={self.translation_roll_ff_gain:.3f}, '
            f'max={self.translation_tilt_ff_max:.3f}'
        )

    def _update_target_quaternion(self):
        self.q_target = rpy_to_quat(self.target_roll, self.target_pitch, self.target_yaw)
        self.target_initialized = True

    def _force_level_roll_pitch_target(self):
        if not self.level_roll_pitch_target:
            return
        self.target_roll = math.radians(self.target_roll_deg + self.roll_trim_deg)
        self.target_pitch = math.radians(self.target_pitch_deg + self.pitch_trim_deg)
        self._update_target_quaternion()

    def _capture_current_attitude_as_target(self):
        self.target_yaw = wrap_to_pi(self.yaw)
        if self.level_roll_pitch_target:
            self.target_roll = math.radians(self.target_roll_deg + self.roll_trim_deg)
            self.target_pitch = math.radians(self.target_pitch_deg + self.pitch_trim_deg)
        else:
            self.target_roll = self.control_roll
            self.target_pitch = self.control_pitch
        self._update_target_quaternion()

    def _capture_current_yaw_as_target(self):
        self.target_yaw = wrap_to_pi(self.yaw)
        self._update_target_quaternion()

    def _set_control_enabled(self, enabled: bool):
        prev = self.control_enabled
        self.control_enabled = bool(enabled)

        if self.control_enabled and not prev and self.have_imu:
            self._reset_control_attitude_filter()
            self._capture_current_attitude_as_target()
            self.roll_integral = 0.0
            self.pitch_integral = 0.0
            self.get_logger().info(
                'Attitude control enabled; captured yaw and kept roll/pitch trim target'
            )
        elif (not self.control_enabled) and prev:
            self.roll_integral = 0.0
            self.pitch_integral = 0.0
            self.pub_torque.publish(Wrench())
            self.get_logger().info('Attitude control disabled; publishing zero torque')

    def imu_callback(self, msg: Imu):
        self.q_current = quat_normalize((
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ))

        self.roll, self.pitch, self.yaw = quat_to_rpy(*self.q_current)

        self.p_rate = msg.angular_velocity.x
        self.q_rate = msg.angular_velocity.y
        self.r_rate = msg.angular_velocity.z

        self.have_imu = True

        if self.capture_initial_target and not self.target_initialized:
            self._reset_control_attitude_filter()
            self._capture_current_attitude_as_target()
            self.get_logger().info(
                f'Initial target captured: '
                f'roll={self.target_roll:.3f}, pitch={self.target_pitch:.3f}, yaw={self.target_yaw:.3f}'
            )

    def manual_wrench_callback(self, msg: Wrench):
        self.manual_wrench = msg

    def cmd_attitude_callback(self, msg: Vector3):
        updated = False

        if not math.isnan(msg.x):
            self.target_roll = msg.x
            updated = True

        if not math.isnan(msg.y):
            self.target_pitch = msg.y
            updated = True

        if self.yaw_hold_enabled and (not math.isnan(msg.z)):
            self.target_yaw = wrap_to_pi(msg.z)
            updated = True

        if not updated:
            return

        self._update_target_quaternion()

        self.get_logger().info(
            f'Updated target attitude: '
            f'roll={self.target_roll:.3f}, pitch={self.target_pitch:.3f}, yaw={self.target_yaw:.3f}'
        )

    def cmd_attitude_trim_callback(self, msg: Vector3):
        updated = False

        if not math.isnan(msg.x):
            self.roll_trim_deg = math.degrees(msg.x)
            updated = True

        if not math.isnan(msg.y):
            self.pitch_trim_deg = math.degrees(msg.y)
            updated = True

        if not updated:
            return

        self._force_level_roll_pitch_target()
        self.get_logger().info(
            f'Updated attitude trim: '
            f'roll_trim={self.roll_trim_deg:.1f} deg, '
            f'pitch_trim={self.pitch_trim_deg:.1f} deg'
        )

    def _apply_deadband(self, x: float) -> float:
        return 0.0 if abs(x) < self.torque_deadband else x

    def _reset_control_attitude_filter(self):
        self.control_roll = self.roll
        self.control_pitch = self.pitch
        self.control_attitude_filter_initialized = True

    def _update_control_attitude_filter(self, dt: float):
        if (
            not self.orientation_filter_enabled or
            not self.control_attitude_filter_initialized or
            dt <= 0.0 or
            dt > 0.2
        ):
            self._reset_control_attitude_filter()
            return

        predicted_roll = self.control_roll + self.p_rate * dt
        predicted_pitch = self.control_pitch + self.q_rate * dt
        alpha = clamp(float(self.orientation_filter_measurement_alpha), 0.0, 1.0)
        max_correction = max(0.0, self.orientation_filter_max_correction_rate) * dt

        roll_correction = wrap_to_pi(self.roll - predicted_roll) * alpha
        pitch_correction = wrap_to_pi(self.pitch - predicted_pitch) * alpha
        if max_correction > 0.0:
            roll_correction = clamp(roll_correction, -max_correction, max_correction)
            pitch_correction = clamp(pitch_correction, -max_correction, max_correction)

        self.control_roll = wrap_to_pi(predicted_roll + roll_correction)
        self.control_pitch = wrap_to_pi(predicted_pitch + pitch_correction)

    def _apply_rp_torque_slew(self, tx_ctrl: float, ty_ctrl: float, dt: float):
        if dt <= 0.0:
            max_delta = 0.0
        else:
            # rp_torque_slew_rate is torque command units per second.
            max_delta = max(0.0, self.rp_torque_slew_rate) * min(dt, 0.1)
        tx_out = clamp(tx_ctrl, self.prev_tx_ctrl - max_delta, self.prev_tx_ctrl + max_delta)
        ty_out = clamp(ty_ctrl, self.prev_ty_ctrl - max_delta, self.prev_ty_ctrl + max_delta)
        self.prev_tx_ctrl = tx_out
        self.prev_ty_ctrl = ty_out
        return tx_out, ty_out

    def _translation_tilt_feedforward(self, manual_surge: float, manual_sway: float):
        if not self.translation_tilt_ff_enabled:
            return 0.0, 0.0

        surge_abs = max(0.0, abs(float(manual_surge)) - self.translation_tilt_ff_deadband)
        sway_abs = max(0.0, abs(float(manual_sway)) - self.translation_tilt_ff_deadband)
        drive = max(surge_abs, sway_abs)
        if drive <= 0.0:
            return 0.0, 0.0

        limit = max(0.0, abs(float(self.translation_tilt_ff_max)))
        tx_ff = self.translation_roll_ff_gain * self.target_roll * drive
        ty_ff = self.translation_pitch_ff_gain * self.target_pitch * drive
        return clamp(tx_ff, -limit, limit), clamp(ty_ff, -limit, limit)

    def control_loop(self):
        if not self.have_imu or not self.target_initialized:
            return

        now = self.get_clock().now()
        dt = 0.0
        if self.last_control_time is not None:
            dt = (now - self.last_control_time).nanoseconds * 1e-9
        self.last_control_time = now

        out = Wrench()
        out.force.x = 0.0
        out.force.y = 0.0
        out.force.z = 0.0
        out.torque.x = 0.0
        out.torque.y = 0.0
        out.torque.z = 0.0

        if not self.control_enabled:
            self.prev_tx_ctrl = 0.0
            self.prev_ty_ctrl = 0.0
            self.control_attitude_filter_initialized = False
            self.pub_torque.publish(out)
            return

        self._update_control_attitude_filter(dt)
        self._force_level_roll_pitch_target()

        manual_heave = self.manual_wrench.force.z
        manual_surge = self.manual_wrench.force.x
        manual_sway = self.manual_wrench.force.y
        tx_manual = self.manual_wrench.torque.x
        ty_manual = self.manual_wrench.torque.y
        tz_manual = self.manual_wrench.torque.z

        # Safety gate: tilt too large
        if (
            abs(self.control_roll) > self.large_tilt_disable_rad or
            abs(self.control_pitch) > self.large_tilt_disable_rad
        ):
            self.roll_integral = 0.0
            self.pitch_integral = 0.0
            self.prev_tx_ctrl = 0.0
            self.prev_ty_ctrl = 0.0
            self.pub_torque.publish(out)
            return

        # High body-rate guard: keep attitude hold active, but clamp the
        # derivative term. A hard zero-output gate makes trim attitude drop out
        # during translation, then snap back when the rate falls.
        body_rate_mag = vec_norm(self.p_rate, self.q_rate, self.r_rate)
        rate_limit = max(0.0, self.max_body_rate_for_control)
        if rate_limit > 1e-6:
            p_rate_ctrl = clamp(self.p_rate, -rate_limit, rate_limit)
            q_rate_ctrl = clamp(self.q_rate, -rate_limit, rate_limit)
            r_rate_ctrl = clamp(self.r_rate, -rate_limit, rate_limit)
        else:
            p_rate_ctrl = self.p_rate
            q_rate_ctrl = self.q_rate
            r_rate_ctrl = self.r_rate
        high_body_rate = rate_limit > 1e-6 and body_rate_mag > rate_limit

        # BlueROV-style heading hold:
        # - yaw stick active: manual yaw rate command wins and the heading target follows current yaw
        # - yaw stick released: damp yaw rate briefly, then lock the released heading
        yaw_hold_active = self.yaw_hold_enabled
        yaw_manual_active = abs(tz_manual) > self.yaw_manual_override_threshold
        yaw_released = self.prev_yaw_manual_active and not yaw_manual_active

        if yaw_manual_active:
            self.last_manual_yaw_input_time = now
            self._capture_current_yaw_as_target()
        elif yaw_released and self.capture_yaw_target_on_release:
            self.last_manual_yaw_input_time = now
            self._capture_current_yaw_as_target()
            self.get_logger().info(
                f'Yaw stick released -> captured heading target: '
                f'{math.degrees(self.target_yaw):+.1f} deg'
            )

        self.prev_yaw_manual_active = yaw_manual_active

        # Roll/pitch: direct angle error for robust recovery at large tilt.
        roll_err = wrap_to_pi(self.target_roll - self.control_roll)
        pitch_err = wrap_to_pi(self.target_pitch - self.control_pitch)
        if dt > 1e-6 and not high_body_rate:
            self.roll_integral += roll_err * dt
            self.pitch_integral += pitch_err * dt
            self.roll_integral = clamp(self.roll_integral, -self.roll_integral_limit, self.roll_integral_limit)
            self.pitch_integral = clamp(self.pitch_integral, -self.roll_integral_limit, self.roll_integral_limit)

        if abs(roll_err) < self.roll_pitch_error_deadband:
            roll_err = 0.0
        if abs(pitch_err) < self.roll_pitch_error_deadband:
            pitch_err = 0.0

        tx_ctrl = self.kp_roll * roll_err + self.ki_roll * self.roll_integral - self.kd_roll * p_rate_ctrl
        ty_ctrl = self.kp_pitch * pitch_err + self.ki_pitch * self.pitch_integral - self.kd_pitch * q_rate_ctrl
        tx_ff, ty_ff = self._translation_tilt_feedforward(manual_surge, manual_sway)
        tx_ctrl += tx_ff
        ty_ctrl += ty_ff

        # Yaw heading-position hold. Direct yaw error is easier to tune than the
        # quaternion z component because kp_yaw maps to radian heading error.
        yaw_err = wrap_to_pi(self.target_yaw - self.yaw)
        if abs(yaw_err) < self.yaw_error_deadband:
            yaw_err = 0.0
        tz_ctrl = 0.0 if not yaw_hold_active else (self.kp_yaw * yaw_err - self.kd_yaw * r_rate_ctrl)

        yaw_settle_active = False
        if yaw_hold_active and (not yaw_manual_active) and self.last_manual_yaw_input_time is not None:
            dt_since_manual = (now - self.last_manual_yaw_input_time).nanoseconds * 1e-9
            yaw_settle_active = (
                self.capture_yaw_target_on_release and
                dt_since_manual < self.yaw_hold_settle_time_sec
            )

        # While user is manually commanding yaw, disable yaw hold contribution
        if yaw_manual_active:
            tz_ctrl = 0.0
        elif yaw_settle_active:
            # ArduSub/BlueROV-style anti-bounce: briefly damp yaw rate before locking heading.
            tz_ctrl = -self.kd_yaw * r_rate_ctrl
            self.target_yaw = wrap_to_pi(self.yaw)
            self._update_target_quaternion()

        # Make roll/pitch trim hold softer while the pilot is translating.
        # Forward surge is excluded by default because forward thrust can create
        # a pitch-up moment; weakening pitch hold there lets a nose-down trim
        # collapse back toward level.
        xy_abs = max(abs(float(manual_surge)), abs(float(manual_sway)))
        forward_surge = float(manual_surge) * float(self.forward_surge_sign)
        keep_forward_trim = (
            self.xy_soft_trim_ignore_forward and
            forward_surge > self.xy_motion_protect_threshold and
            abs(float(manual_sway)) <= self.strong_xy_motion_threshold
        )
        rp_scale = 1.0
        if keep_forward_trim:
            mode = 'FORWARD_TRIM_HOLD'
        elif xy_abs > self.strong_xy_motion_threshold:
            rp_scale = self.rp_scale_when_strong_xy_motion
            mode = 'STRONG_XY_SOFT_TRIM'
        elif xy_abs > self.xy_motion_protect_threshold:
            rp_scale = self.rp_scale_when_xy_motion
            mode = 'XY_SOFT_TRIM'
        else:
            mode = 'NORMAL'

        tx_scale = rp_scale
        ty_scale = rp_scale

        # If the pilot/controller requested a non-level roll or pitch target,
        # keep that trim authority during translation. Otherwise lateral motion
        # can make a deliberate 25 deg pitch target collapse toward level until
        # the stick is released.
        if abs(self.target_roll) > self.roll_pitch_error_deadband:
            tx_scale = 1.0
        if abs(self.target_pitch) > self.roll_pitch_error_deadband:
            ty_scale = 1.0

        tx_ctrl *= tx_scale
        ty_ctrl *= ty_scale

        # Protect yaw hold during heave stick input without scaling roll/pitch further.
        heave_abs = abs(manual_heave)
        if heave_abs > self.strong_heave_threshold:
            if not yaw_manual_active:
                tz_ctrl *= self.yaw_scale_when_strong_heave
            mode += '|STRONG_HEAVE_PROTECT'
        elif heave_abs > self.heave_protect_threshold:
            if not yaw_manual_active:
                tz_ctrl *= self.yaw_scale_when_heave
            mode += '|HEAVE_PROTECT'

        if high_body_rate:
            mode += '|RATE_LIMIT'

        if yaw_manual_active:
            mode += '|YAW_MANUAL'
        elif yaw_settle_active:
            mode += '|YAW_SETTLE'

        tx_ctrl = clamp(tx_ctrl, -self.tx_limit, self.tx_limit)
        ty_ctrl = clamp(ty_ctrl, -self.ty_limit, self.ty_limit)
        tz_ctrl = clamp(tz_ctrl, -self.tz_limit, self.tz_limit)

        tx_ctrl, ty_ctrl = self._apply_rp_torque_slew(tx_ctrl, ty_ctrl, dt)

        # tx_ctrl = self._apply_deadband(tx_ctrl)
        # ty_ctrl = self._apply_deadband(ty_ctrl)
        # tz_ctrl = self._apply_deadband(tz_ctrl)

        # Keep existing manual torque bias if any
        out.torque.x = tx_manual + tx_ctrl
        out.torque.y = ty_manual + ty_ctrl
        out.torque.z = tz_manual + tz_ctrl

        self.pub_torque.publish(out)

        self._log_counter += 1
        # if self._log_counter % 25 == 0:
        #     self.get_logger().info(
        #         f'mode={mode}, heave={manual_heave:.3f}, yaw_manual={tz_manual:.3f}, '
        #         f'target_rpy=({self.target_roll:.3f}, {self.target_pitch:.3f}, {self.target_yaw:.3f}), '
        #         f'ctrl_torque=({tx_ctrl:.4f}, {ty_ctrl:.4f}, {tz_ctrl:.4f}), '
        #         f'out_torque=({out.torque.x:.4f}, {out.torque.y:.4f}, {out.torque.z:.4f}), '
        #         f'rpy=({self.roll:.3f}, {self.pitch:.3f}, {self.yaw:.3f}), '
        #         f'control_rp=({self.control_roll:.3f}, {self.control_pitch:.3f})'
        #     )

    def on_parameter_update(self, params):
        try:
            for p in params:
                if p.name == 'kp_roll':
                    self.kp_roll = float(p.value)
                elif p.name == 'kd_roll':
                    self.kd_roll = float(p.value)
                elif p.name == 'ki_roll':
                    self.ki_roll = float(p.value)
                elif p.name == 'kp_pitch':
                    self.kp_pitch = float(p.value)
                elif p.name == 'kd_pitch':
                    self.kd_pitch = float(p.value)
                elif p.name == 'ki_pitch':
                    self.ki_pitch = float(p.value)
                elif p.name == 'kp_yaw':
                    self.kp_yaw = float(p.value)
                elif p.name == 'kd_yaw':
                    self.kd_yaw = float(p.value)

                elif p.name == 'tx_limit':
                    self.tx_limit = float(p.value)
                elif p.name == 'ty_limit':
                    self.ty_limit = float(p.value)
                elif p.name == 'tz_limit':
                    self.tz_limit = float(p.value)
                elif p.name == 'rp_torque_slew_rate':
                    self.rp_torque_slew_rate = float(p.value)

                elif p.name == 'capture_initial_target':
                    self.capture_initial_target = bool(p.value)
                elif p.name == 'level_roll_pitch_target':
                    self.level_roll_pitch_target = bool(p.value)
                    self._force_level_roll_pitch_target()
                elif p.name == 'target_roll_deg':
                    self.target_roll_deg = float(p.value)
                    self._force_level_roll_pitch_target()
                elif p.name == 'target_pitch_deg':
                    self.target_pitch_deg = float(p.value)
                    self._force_level_roll_pitch_target()
                elif p.name == 'roll_trim_deg':
                    self.roll_trim_deg = float(p.value)
                    self._force_level_roll_pitch_target()
                elif p.name == 'pitch_trim_deg':
                    self.pitch_trim_deg = float(p.value)
                    self._force_level_roll_pitch_target()
                elif p.name == 'control_enabled':
                    self._set_control_enabled(bool(p.value))

                elif p.name == 'heave_protect_threshold':
                    self.heave_protect_threshold = float(p.value)
                elif p.name == 'strong_heave_threshold':
                    self.strong_heave_threshold = float(p.value)
                elif p.name == 'xy_motion_protect_threshold':
                    self.xy_motion_protect_threshold = float(p.value)
                elif p.name == 'strong_xy_motion_threshold':
                    self.strong_xy_motion_threshold = float(p.value)
                elif p.name == 'rp_scale_when_xy_motion':
                    self.rp_scale_when_xy_motion = float(p.value)
                elif p.name == 'rp_scale_when_strong_xy_motion':
                    self.rp_scale_when_strong_xy_motion = float(p.value)
                elif p.name == 'xy_soft_trim_ignore_forward':
                    self.xy_soft_trim_ignore_forward = bool(p.value)
                elif p.name == 'forward_surge_sign':
                    self.forward_surge_sign = float(p.value)
                elif p.name == 'yaw_scale_when_heave':
                    self.yaw_scale_when_heave = float(p.value)
                elif p.name == 'yaw_scale_when_strong_heave':
                    self.yaw_scale_when_strong_heave = float(p.value)

                elif p.name == 'yaw_manual_override_threshold':
                    self.yaw_manual_override_threshold = float(p.value)
                elif p.name == 'yaw_hold_enabled':
                    self.yaw_hold_enabled = bool(p.value)
                elif p.name == 'capture_yaw_target_on_release':
                    self.capture_yaw_target_on_release = bool(p.value)
                elif p.name == 'yaw_hold_settle_time_sec':
                    self.yaw_hold_settle_time_sec = float(p.value)
                elif p.name == 'yaw_error_deadband_deg':
                    self.yaw_error_deadband = math.radians(float(p.value))

                elif p.name == 'max_body_rate_for_control':
                    self.max_body_rate_for_control = float(p.value)
                elif p.name == 'large_tilt_disable_deg':
                    self.large_tilt_disable_deg = float(p.value)
                    self.large_tilt_disable_rad = math.radians(self.large_tilt_disable_deg)
                elif p.name == 'torque_deadband':
                    self.torque_deadband = float(p.value)
                elif p.name == 'roll_pitch_error_deadband_deg':
                    self.roll_pitch_error_deadband = math.radians(float(p.value))
                elif p.name == 'roll_integral_limit':
                    self.roll_integral_limit = float(p.value)
                    self.roll_integral = clamp(self.roll_integral, -self.roll_integral_limit, self.roll_integral_limit)
                    self.pitch_integral = clamp(self.pitch_integral, -self.roll_integral_limit, self.roll_integral_limit)
                elif p.name == 'orientation_filter_enabled':
                    self.orientation_filter_enabled = bool(p.value)
                    self._reset_control_attitude_filter()
                elif p.name == 'orientation_filter_measurement_alpha':
                    self.orientation_filter_measurement_alpha = float(p.value)
                elif p.name == 'orientation_filter_max_correction_rate_deg':
                    self.orientation_filter_max_correction_rate = math.radians(float(p.value))
                elif p.name == 'translation_tilt_ff_enabled':
                    self.translation_tilt_ff_enabled = bool(p.value)
                elif p.name == 'translation_pitch_ff_gain':
                    self.translation_pitch_ff_gain = float(p.value)
                elif p.name == 'translation_roll_ff_gain':
                    self.translation_roll_ff_gain = float(p.value)
                elif p.name == 'translation_tilt_ff_max':
                    self.translation_tilt_ff_max = float(p.value)
                elif p.name == 'translation_tilt_ff_deadband':
                    self.translation_tilt_ff_deadband = float(p.value)

            self.get_logger().info('Attitude parameters updated at runtime')
            return SetParametersResult(successful=True)
        except Exception as e:
            return SetParametersResult(successful=False, reason=str(e))

def main(args=None):
    rclpy.init(args=args)
    node = AttitudeController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
