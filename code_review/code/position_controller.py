#!/usr/bin/env python3
import math

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PointStamped, Wrench
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

from rov_interfaces.msg import DVLData


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def quat_normalize(x: float, y: float, z: float, w: float):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_from_rpy(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return quat_normalize(
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quat_rotate_vector(q, v):
    qv = (v[0], v[1], v[2], 0.0)
    rotated = quat_multiply(quat_multiply(q, qv), quat_conjugate(q))
    return (rotated[0], rotated[1], rotated[2])


class PositionController(Node):

    def __init__(self):
        super().__init__('position_controller')

        self.declare_parameter('dvl_topic', '/dvl/data')
        self.declare_parameter('dvl_position_topic', '/dvl/position')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('manual_wrench_topic', '/rov/wrench_manual')
        self.declare_parameter('position_force_topic', '/ctrl/position_force')
        self.declare_parameter('position_estimate_topic', '/rov/position_estimate')
        self.declare_parameter('armed_topic', '/rov/armed')
        self.declare_parameter('control_enabled', True)

        self.declare_parameter('kp_x', 0.8)
        self.declare_parameter('kd_x', 0.6)
        self.declare_parameter('kp_y', 0.8)
        self.declare_parameter('kd_y', 0.6)
        self.declare_parameter('max_force_x', 1.0)
        self.declare_parameter('max_force_y', 1.0)
        self.declare_parameter('max_force_z', 1.0)
        self.declare_parameter('force_deadband', 1e-4)
        self.declare_parameter('yaw_rate_damping_gain', 1.2)
        self.declare_parameter('manual_yaw_damping_boost', 0.6)
        self.declare_parameter('manual_yaw_override_threshold', 0.05)

        self.declare_parameter('manual_xy_override_threshold', 0.05)
        self.declare_parameter('capture_initial_position_target', True)
        self.declare_parameter('capture_target_on_manual_release', True)
        self.declare_parameter('valid_timeout_sec', 0.5)
        self.declare_parameter('hold_frame_id', 'dvl_odom')
        self.declare_parameter('use_dvl_position', True)
        self.declare_parameter('integrate_dvl_velocity_when_position_unavailable', True)
        self.declare_parameter('dvl_mount_roll_deg', 0.0)
        self.declare_parameter('dvl_mount_pitch_deg', 0.0)
        self.declare_parameter('dvl_mount_yaw_deg', 0.0)

        self.dvl_topic = str(self.get_parameter('dvl_topic').value)
        self.dvl_position_topic = str(self.get_parameter('dvl_position_topic').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.manual_wrench_topic = str(self.get_parameter('manual_wrench_topic').value)
        self.position_force_topic = str(self.get_parameter('position_force_topic').value)
        self.position_estimate_topic = str(self.get_parameter('position_estimate_topic').value)
        self.armed_topic = str(self.get_parameter('armed_topic').value)
        self.control_enabled = bool(self.get_parameter('control_enabled').value)
        self.hold_frame_id = str(self.get_parameter('hold_frame_id').value)
        self.active_position_frame_id = self.hold_frame_id
        self.use_dvl_position = bool(self.get_parameter('use_dvl_position').value)
        self.integrate_dvl_velocity_when_position_unavailable = bool(
            self.get_parameter('integrate_dvl_velocity_when_position_unavailable').value
        )
        self.dvl_mount_roll_deg = float(self.get_parameter('dvl_mount_roll_deg').value)
        self.dvl_mount_pitch_deg = float(self.get_parameter('dvl_mount_pitch_deg').value)
        self.dvl_mount_yaw_deg = float(self.get_parameter('dvl_mount_yaw_deg').value)

        self.kp_x = float(self.get_parameter('kp_x').value)
        self.kd_x = float(self.get_parameter('kd_x').value)
        self.kp_y = float(self.get_parameter('kp_y').value)
        self.kd_y = float(self.get_parameter('kd_y').value)
        self.max_force_x = float(self.get_parameter('max_force_x').value)
        self.max_force_y = float(self.get_parameter('max_force_y').value)
        self.max_force_z = float(self.get_parameter('max_force_z').value)
        self.force_deadband = float(self.get_parameter('force_deadband').value)
        self.yaw_rate_damping_gain = float(self.get_parameter('yaw_rate_damping_gain').value)
        self.manual_yaw_damping_boost = float(self.get_parameter('manual_yaw_damping_boost').value)
        self.manual_yaw_override_threshold = float(
            self.get_parameter('manual_yaw_override_threshold').value
        )

        self.manual_xy_override_threshold = float(
            self.get_parameter('manual_xy_override_threshold').value
        )
        self.capture_initial_position_target = bool(
            self.get_parameter('capture_initial_position_target').value
        )
        self.capture_target_on_manual_release = bool(
            self.get_parameter('capture_target_on_manual_release').value
        )
        self.valid_timeout_sec = float(self.get_parameter('valid_timeout_sec').value)

        self.have_imu = False
        self.target_initialized = False
        self.armed = False
        self.prev_armed = False
        self.armed_received = False

        self.q_world_body = (0.0, 0.0, 0.0, 1.0)
        self.position_x = 0.0
        self.position_y = 0.0
        self.velocity_world_x = 0.0
        self.velocity_world_y = 0.0
        self.current_yaw_rate = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.q_body_dvl = quat_from_rpy(
            math.radians(self.dvl_mount_roll_deg),
            math.radians(self.dvl_mount_pitch_deg),
            math.radians(self.dvl_mount_yaw_deg),
        )

        self.last_dvl_time = None
        self.last_valid_dvl_time = None
        self.last_dvl_position_time = None
        self.manual_wrench = Wrench()
        self.manual_xy_active = False
        self.prev_manual_xy_active = False

        self.sub_imu = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
        self.sub_dvl = self.create_subscription(DVLData, self.dvl_topic, self.dvl_callback, 10)
        self.sub_dvl_position = self.create_subscription(
            PointStamped,
            self.dvl_position_topic,
            self.dvl_position_callback,
            10
        )
        self.sub_manual = self.create_subscription(
            Wrench,
            self.manual_wrench_topic,
            self.manual_wrench_callback,
            10
        )
        self.sub_armed = self.create_subscription(
            Bool,
            self.armed_topic,
            self.armed_callback,
            10
        )

        self.pub_force = self.create_publisher(Wrench, self.position_force_topic, 10)
        self.pub_position = self.create_publisher(PointStamped, self.position_estimate_topic, 10)

        self.add_on_set_parameters_callback(self.on_parameter_update)

        self.get_logger().info('PositionController initialized (ArduSub-style poshold)')
        self.get_logger().info(f'  dvl_topic                       = {self.dvl_topic}')
        self.get_logger().info(f'  dvl_position_topic              = {self.dvl_position_topic}')
        self.get_logger().info(f'  imu_topic                       = {self.imu_topic}')
        self.get_logger().info(f'  manual_wrench_topic             = {self.manual_wrench_topic}')
        self.get_logger().info(f'  position_force_topic            = {self.position_force_topic}')
        self.get_logger().info(f'  position_estimate_topic         = {self.position_estimate_topic}')
        self.get_logger().info(f'  armed_topic                     = {self.armed_topic}')
        self.get_logger().info(f'  hold_frame_id                   = {self.hold_frame_id}')
        self.get_logger().info(f'  use_dvl_position                = {self.use_dvl_position}')
        self.get_logger().info(
            f'  integrate_dvl_velocity_fallback = '
            f'{self.integrate_dvl_velocity_when_position_unavailable}'
        )
        self.get_logger().info(
            f'  dvl_mount_rpy_deg               = '
            f'({self.dvl_mount_roll_deg:.1f}, {self.dvl_mount_pitch_deg:.1f}, {self.dvl_mount_yaw_deg:.1f})'
        )
        self.get_logger().info(f'  control_enabled                 = {self.control_enabled}')

    def _publish_zero_force(self):
        self.pub_force.publish(Wrench())

    def _has_valid_position_reference(self, now) -> bool:
        if not self.have_imu:
            return False
        if self.last_valid_dvl_time is None:
            return False
        age = (now - self.last_valid_dvl_time).nanoseconds * 1e-9
        return age <= self.valid_timeout_sec

    def _set_control_enabled(self, enabled: bool):
        prev = self.control_enabled
        self.control_enabled = bool(enabled)
        now = self.get_clock().now()

        if self.control_enabled and not prev and self._has_valid_position_reference(now):
            self._capture_current_position_as_target()
            self.get_logger().info('Position control enabled; captured current hold position')
        elif (not self.control_enabled) and prev:
            self._publish_zero_force()
            self.get_logger().info('Position control disabled; publishing zero planar force')

    def _capture_current_position_as_target(self):
        self.target_x = self.position_x
        self.target_y = self.position_y
        self.target_initialized = True

    def _apply_deadband(self, value: float) -> float:
        return 0.0 if abs(value) < self.force_deadband else value

    def _dvl_position_is_finite(self, msg: PointStamped) -> bool:
        return math.isfinite(float(msg.point.x)) and math.isfinite(float(msg.point.y))

    def imu_callback(self, msg: Imu):
        self.q_world_body = quat_normalize(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        self.current_yaw_rate = float(msg.angular_velocity.z)
        self.have_imu = True

    def manual_wrench_callback(self, msg: Wrench):
        self.manual_wrench = msg
        self.prev_manual_xy_active = self.manual_xy_active
        self.manual_xy_active = (
            abs(msg.force.x) > self.manual_xy_override_threshold or
            abs(msg.force.y) > self.manual_xy_override_threshold
        )

        if (
            self.capture_target_on_manual_release and
            self.prev_manual_xy_active and
            (not self.manual_xy_active) and
            self._has_valid_position_reference(self.get_clock().now())
        ):
            self._capture_current_position_as_target()
            self.get_logger().info(
                f'Manual XY released -> captured current hold position: '
                f'x={self.target_x:.3f}, y={self.target_y:.3f}'
            )

    def armed_callback(self, msg: Bool):
        self.prev_armed = self.armed
        self.armed = bool(msg.data)
        self.armed_received = True

        if (
            (not self.prev_armed) and
            self.armed and
            self._has_valid_position_reference(self.get_clock().now())
        ):
            self._capture_current_position_as_target()
            self.get_logger().info(
                f'ARM rising edge -> captured current hold position: '
                f'x={self.target_x:.3f}, y={self.target_y:.3f}'
            )

        if self.prev_armed and (not self.armed):
            self._publish_zero_force()
            self.get_logger().info('DISARM -> position controller output reset to zero')

    def dvl_position_callback(self, msg: PointStamped):
        now = self.get_clock().now()
        stamp = now

        if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0:
            stamp = Time.from_msg(msg.header.stamp)

        self.last_dvl_position_time = stamp

        if self.use_dvl_position and self._dvl_position_is_finite(msg):
            self.position_x = float(msg.point.x)
            self.position_y = float(msg.point.y)
            self.active_position_frame_id = msg.header.frame_id or self.hold_frame_id
            self.last_valid_dvl_time = now

            if self.capture_initial_position_target and not self.target_initialized:
                self._capture_current_position_as_target()
                self.get_logger().info(
                    f'Initial hold position captured from DVL position: '
                    f'x={self.target_x:.3f}, y={self.target_y:.3f}'
                )

        self.publish_position_estimate(now)
        self.publish_control_output(now)

    def dvl_callback(self, msg: DVLData):
        now = self.get_clock().now()
        stamp = now

        if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0:
            stamp = Time.from_msg(msg.header.stamp)

        dt = 0.0
        if self.last_dvl_time is not None:
            dt = (stamp - self.last_dvl_time).nanoseconds * 1e-9
        self.last_dvl_time = stamp

        if msg.velocity_valid and self.have_imu:
            v_dvl = (float(msg.vx), float(msg.vy), float(msg.vz))
            v_body = quat_rotate_vector(self.q_body_dvl, v_dvl)
            v_world = quat_rotate_vector(self.q_world_body, v_body)

            self.velocity_world_x = v_world[0]
            self.velocity_world_y = v_world[1]

            if (
                (not self.use_dvl_position) and
                self.integrate_dvl_velocity_when_position_unavailable and
                dt > 1e-6
            ):
                self.position_x += self.velocity_world_x * dt
                self.position_y += self.velocity_world_y * dt
                self.active_position_frame_id = self.hold_frame_id

            if not self.use_dvl_position:
                self.last_valid_dvl_time = now

            if (
                (not self.use_dvl_position) and
                self.capture_initial_position_target and
                (not self.target_initialized)
            ):
                self._capture_current_position_as_target()
                self.get_logger().info(
                    f'Initial hold position captured: '
                    f'x={self.target_x:.3f}, y={self.target_y:.3f}'
                )
        else:
            self.velocity_world_x = 0.0
            self.velocity_world_y = 0.0

        self.publish_position_estimate(now)
        self.publish_control_output(now)

    def publish_position_estimate(self, stamp):
        msg = PointStamped()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.active_position_frame_id
        msg.point.x = self.position_x
        msg.point.y = self.position_y
        msg.point.z = 0.0
        self.pub_position.publish(msg)

    def publish_control_output(self, now):
        out = Wrench()

        if not self.target_initialized or not self._has_valid_position_reference(now):
            self.pub_force.publish(out)
            return

        if self.armed_received and not self.armed:
            self.pub_force.publish(out)
            return

        if not self.control_enabled:
            self.pub_force.publish(out)
            return

        error_x = self.target_x - self.position_x
        error_y = self.target_y - self.position_y

        extra_damping = self.yaw_rate_damping_gain * abs(self.current_yaw_rate)
        if abs(self.manual_wrench.torque.z) > self.manual_yaw_override_threshold:
            extra_damping += self.manual_yaw_damping_boost

        effective_kd_x = self.kd_x + extra_damping
        effective_kd_y = self.kd_y + extra_damping

        force_world_x = self.kp_x * error_x - effective_kd_x * self.velocity_world_x
        force_world_y = self.kp_y * error_y - effective_kd_y * self.velocity_world_y
        force_world = (force_world_x, force_world_y, 0.0)
        force_body = quat_rotate_vector(quat_conjugate(self.q_world_body), force_world)
        force_body_x = force_body[0]
        force_body_y = force_body[1]
        force_body_z = force_body[2]

        if self.manual_xy_active:
            force_body_x = 0.0
            force_body_y = 0.0
            force_body_z = 0.0

        out.force.x = self._apply_deadband(clamp(force_body_x, -self.max_force_x, self.max_force_x))
        out.force.y = self._apply_deadband(clamp(force_body_y, -self.max_force_y, self.max_force_y))
        out.force.z = self._apply_deadband(clamp(force_body_z, -self.max_force_z, self.max_force_z))
        out.torque.x = 0.0
        out.torque.y = 0.0
        out.torque.z = 0.0

        self.pub_force.publish(out)

    def on_parameter_update(self, params):
        try:
            for p in params:
                if p.name == 'kp_x':
                    self.kp_x = float(p.value)
                elif p.name == 'kd_x':
                    self.kd_x = float(p.value)
                elif p.name == 'kp_y':
                    self.kp_y = float(p.value)
                elif p.name == 'kd_y':
                    self.kd_y = float(p.value)
                elif p.name == 'max_force_x':
                    self.max_force_x = float(p.value)
                elif p.name == 'max_force_y':
                    self.max_force_y = float(p.value)
                elif p.name == 'max_force_z':
                    self.max_force_z = float(p.value)
                elif p.name == 'force_deadband':
                    self.force_deadband = float(p.value)
                elif p.name == 'yaw_rate_damping_gain':
                    self.yaw_rate_damping_gain = float(p.value)
                elif p.name == 'manual_yaw_damping_boost':
                    self.manual_yaw_damping_boost = float(p.value)
                elif p.name == 'manual_yaw_override_threshold':
                    self.manual_yaw_override_threshold = float(p.value)
                elif p.name == 'manual_xy_override_threshold':
                    self.manual_xy_override_threshold = float(p.value)
                elif p.name == 'capture_initial_position_target':
                    self.capture_initial_position_target = bool(p.value)
                elif p.name == 'capture_target_on_manual_release':
                    self.capture_target_on_manual_release = bool(p.value)
                elif p.name == 'valid_timeout_sec':
                    self.valid_timeout_sec = float(p.value)
                elif p.name == 'hold_frame_id':
                    self.hold_frame_id = str(p.value)
                    if not self.use_dvl_position:
                        self.active_position_frame_id = self.hold_frame_id
                elif p.name == 'use_dvl_position':
                    self.use_dvl_position = bool(p.value)
                    if not self.use_dvl_position:
                        self.active_position_frame_id = self.hold_frame_id
                elif p.name == 'integrate_dvl_velocity_when_position_unavailable':
                    self.integrate_dvl_velocity_when_position_unavailable = bool(p.value)
                elif p.name == 'dvl_mount_roll_deg':
                    self.dvl_mount_roll_deg = float(p.value)
                    self.q_body_dvl = quat_from_rpy(
                        math.radians(self.dvl_mount_roll_deg),
                        math.radians(self.dvl_mount_pitch_deg),
                        math.radians(self.dvl_mount_yaw_deg),
                    )
                elif p.name == 'dvl_mount_pitch_deg':
                    self.dvl_mount_pitch_deg = float(p.value)
                    self.q_body_dvl = quat_from_rpy(
                        math.radians(self.dvl_mount_roll_deg),
                        math.radians(self.dvl_mount_pitch_deg),
                        math.radians(self.dvl_mount_yaw_deg),
                    )
                elif p.name == 'dvl_mount_yaw_deg':
                    self.dvl_mount_yaw_deg = float(p.value)
                    self.q_body_dvl = quat_from_rpy(
                        math.radians(self.dvl_mount_roll_deg),
                        math.radians(self.dvl_mount_pitch_deg),
                        math.radians(self.dvl_mount_yaw_deg),
                    )
                elif p.name == 'control_enabled':
                    self._set_control_enabled(bool(p.value))

            self.get_logger().info('Position controller parameters updated at runtime')
            return SetParametersResult(successful=True)
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))


def main(args=None):
    rclpy.init(args=args)
    node = PositionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
