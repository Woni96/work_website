#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Vector3, Wrench
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, Float64MultiArray
from rcl_interfaces.msg import SetParametersResult

import numpy as np


CONTROL_SUB_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def normalize(v):
    v = np.array(v, dtype=float)
    n = np.linalg.norm(v)
    if n < 1e-9:
        return v
    return v / n


def normalize_group_unit(v):
    arr = np.array(v, dtype=float)
    max_abs = float(np.max(np.abs(arr)))
    if max_abs > 1.0:
        arr = arr / max_abs
    return arr


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


def quat_to_rpy(x: float, y: float, z: float, w: float):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return 0.0, 0.0, 0.0
    x /= n
    y /= n
    z /= n
    w /= n

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


class AllocatorNode(Node):

    def __init__(self):
        super().__init__('allocator_node')

        # 실제 plugin / can 방향 반대면 여기 부호만 바꾸면 됨
        self.signs = np.array([
            1.0, 1.0, 1.0, 1.0,   # thruster 1~4
            1.0, 1.0, 1.0, 1.0    # thruster 5~8
        ], dtype=float)

        # -------------------------
        # parameters
        # -------------------------
        self.declare_parameter('wrench_cmd_topic', '/rov/wrench_cmd')
        self.declare_parameter('thruster_cmd_topic', '/thruster_cmd')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('cmd_attitude_topic', '/cmd_attitude')
        self.declare_parameter('cmd_attitude_trim_topic', '/cmd_attitude_trim')
        self.declare_parameter('output_scale_topic', '/rov/output_scale')
        self.declare_parameter('joy_speed_scale_topic', '/rov/joy_speed_scale')
        self.declare_parameter('use_joy_speed_scale_for_output', False)
        self.declare_parameter('heave_gain', 1.4)
        self.declare_parameter('horizontal_output_gain', 2.5)
        self.declare_parameter('yaw_output_gain', 0.35)
        self.declare_parameter('vertical_output_gain', 3.0)
        self.declare_parameter('rear_vertical_bias', 0.0)
        self.declare_parameter('pitch_torque_gain', 1.5)
        self.declare_parameter('torque_first_allocation', True)
        self.declare_parameter('slew_rate', 4.0)
        self.declare_parameter('max_output', 1.0)
        self.declare_parameter('output_scale', 0.25)
        self.declare_parameter('output_deadband', 0.02)
        self.declare_parameter('level_horizontal_compensation_enabled', True)
        self.declare_parameter('level_horizontal_compensation_gain', 1.0)
        self.declare_parameter('level_horizontal_compensation_max', 0.5)
        self.declare_parameter('level_horizontal_compensation_heave_sign', -1.0)
        self.declare_parameter('level_horizontal_compensation_min_z', 0.35)
        self.declare_parameter('level_horizontal_compensation_uses_spare_only', True)
        self.declare_parameter('attitude_priority_horizontal_slowdown_enabled', False)
        self.declare_parameter('attitude_priority_horizontal_slowdown_start', 0.06)
        self.declare_parameter('attitude_priority_horizontal_slowdown_full', 0.12)
        self.declare_parameter('attitude_priority_horizontal_min_scale', 0.25)
        self.declare_parameter('surge_pitch_moment_compensation_enabled', False)
        self.declare_parameter('surge_pitch_moment_gain', -0.12)
        self.declare_parameter('surge_pitch_moment_max', 0.12)
        self.declare_parameter('surge_pitch_moment_min_surge', 0.03)
        self.declare_parameter('surge_pitch_moment_target_gate_enabled', True)
        self.declare_parameter('surge_pitch_moment_min_target_deg', 3.0)
        self.declare_parameter('surge_pitch_moment_full_target_deg', 20.0)
        self.declare_parameter('imu_pitch_hold_compensation_enabled', False)
        self.declare_parameter('imu_pitch_hold_gain', 0.25)
        self.declare_parameter('imu_pitch_hold_max', 0.10)
        self.declare_parameter('imu_pitch_hold_deadband_deg', 1.0)
        self.declare_parameter('imu_pitch_hold_min_surge', 0.03)

        self.wrench_cmd_topic = str(self.get_parameter('wrench_cmd_topic').value)
        self.thruster_cmd_topic = str(self.get_parameter('thruster_cmd_topic').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.cmd_attitude_topic = str(self.get_parameter('cmd_attitude_topic').value)
        self.cmd_attitude_trim_topic = str(self.get_parameter('cmd_attitude_trim_topic').value)
        self.output_scale_topic = str(self.get_parameter('output_scale_topic').value)
        self.joy_speed_scale_topic = str(self.get_parameter('joy_speed_scale_topic').value)
        self.use_joy_speed_scale_for_output = bool(
            self.get_parameter('use_joy_speed_scale_for_output').value
        )
        self.heave_gain = float(self.get_parameter('heave_gain').value)
        self.horizontal_output_gain = float(self.get_parameter('horizontal_output_gain').value)
        self.yaw_output_gain = float(self.get_parameter('yaw_output_gain').value)
        self.vertical_output_gain = float(self.get_parameter('vertical_output_gain').value)
        self.rear_vertical_bias = float(self.get_parameter('rear_vertical_bias').value)
        self.pitch_torque_gain = float(self.get_parameter('pitch_torque_gain').value)
        self.torque_first_allocation = bool(self.get_parameter('torque_first_allocation').value)
        self.slew_rate = float(self.get_parameter('slew_rate').value)
        self.max_output = float(self.get_parameter('max_output').value)
        self.output_scale = float(self.get_parameter('output_scale').value)
        self.output_deadband = float(self.get_parameter('output_deadband').value)
        self.level_horizontal_compensation_enabled = bool(
            self.get_parameter('level_horizontal_compensation_enabled').value
        )
        self.level_horizontal_compensation_gain = float(
            self.get_parameter('level_horizontal_compensation_gain').value
        )
        self.level_horizontal_compensation_max = float(
            self.get_parameter('level_horizontal_compensation_max').value
        )
        self.level_horizontal_compensation_heave_sign = float(
            self.get_parameter('level_horizontal_compensation_heave_sign').value
        )
        self.level_horizontal_compensation_min_z = float(
            self.get_parameter('level_horizontal_compensation_min_z').value
        )
        self.level_horizontal_compensation_uses_spare_only = bool(
            self.get_parameter('level_horizontal_compensation_uses_spare_only').value
        )
        self.attitude_priority_horizontal_slowdown_enabled = bool(
            self.get_parameter('attitude_priority_horizontal_slowdown_enabled').value
        )
        self.attitude_priority_horizontal_slowdown_start = float(
            self.get_parameter('attitude_priority_horizontal_slowdown_start').value
        )
        self.attitude_priority_horizontal_slowdown_full = float(
            self.get_parameter('attitude_priority_horizontal_slowdown_full').value
        )
        self.attitude_priority_horizontal_min_scale = float(
            self.get_parameter('attitude_priority_horizontal_min_scale').value
        )
        self.surge_pitch_moment_compensation_enabled = bool(
            self.get_parameter('surge_pitch_moment_compensation_enabled').value
        )
        self.surge_pitch_moment_gain = float(
            self.get_parameter('surge_pitch_moment_gain').value
        )
        self.surge_pitch_moment_max = float(
            self.get_parameter('surge_pitch_moment_max').value
        )
        self.surge_pitch_moment_min_surge = float(
            self.get_parameter('surge_pitch_moment_min_surge').value
        )
        self.surge_pitch_moment_target_gate_enabled = bool(
            self.get_parameter('surge_pitch_moment_target_gate_enabled').value
        )
        self.surge_pitch_moment_min_target = math.radians(
            float(self.get_parameter('surge_pitch_moment_min_target_deg').value)
        )
        self.surge_pitch_moment_full_target = math.radians(
            float(self.get_parameter('surge_pitch_moment_full_target_deg').value)
        )
        self.imu_pitch_hold_compensation_enabled = bool(
            self.get_parameter('imu_pitch_hold_compensation_enabled').value
        )
        self.imu_pitch_hold_gain = float(
            self.get_parameter('imu_pitch_hold_gain').value
        )
        self.imu_pitch_hold_max = float(
            self.get_parameter('imu_pitch_hold_max').value
        )
        self.imu_pitch_hold_deadband = math.radians(
            float(self.get_parameter('imu_pitch_hold_deadband_deg').value)
        )
        self.imu_pitch_hold_min_surge = float(
            self.get_parameter('imu_pitch_hold_min_surge').value
        )

        # torque gain
        self.roll_torque_gain = 1.0
        self.yaw_torque_gain = 1.0

        # 이전 출력 저장 (slew rate용)
        self.prev_thrust = np.zeros(8, dtype=float)
        self.last_slew_time = None
        self.have_imu = False
        self.world_z_from_body = (0.0, 0.0, 1.0)
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        self.target_roll = 0.0
        self.target_pitch = 0.0

        self.sub = self.create_subscription(
            Wrench,
            self.wrench_cmd_topic,
            self.callback,
            CONTROL_SUB_QOS
        )
        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            CONTROL_SUB_QOS
        )
        self.cmd_attitude_sub = self.create_subscription(
            Vector3,
            self.cmd_attitude_topic,
            self.cmd_attitude_callback,
            CONTROL_SUB_QOS
        )
        self.cmd_attitude_trim_sub = self.create_subscription(
            Vector3,
            self.cmd_attitude_trim_topic,
            self.cmd_attitude_trim_callback,
            CONTROL_SUB_QOS
        )
        self.output_scale_sub = self.create_subscription(
            Float64,
            self.output_scale_topic,
            self.output_scale_callback,
            CONTROL_SUB_QOS
        )
        self.joy_speed_scale_sub = self.create_subscription(
            Float64,
            self.joy_speed_scale_topic,
            self.joy_speed_scale_callback,
            CONTROL_SUB_QOS
        )

        self.pub = self.create_publisher(
            Float64MultiArray,
            self.thruster_cmd_topic,
            10
        )

        self.init_matrices()

        # 런타임 파라미터 업데이트 콜백 등록
        self.add_on_set_parameters_callback(self.on_parameter_update)

        self.get_logger().info('Allocator node started (priority allocator)')
        self.get_logger().info(
            f'wrench_cmd_topic={self.wrench_cmd_topic}, '
            f'thruster_cmd_topic={self.thruster_cmd_topic}, '
            f'imu_topic={self.imu_topic}, '
            f'cmd_attitude_topic={self.cmd_attitude_topic}, '
            f'cmd_attitude_trim_topic={self.cmd_attitude_trim_topic}, '
            f'output_scale_topic={self.output_scale_topic}, '
            f'joy_speed_scale_topic={self.joy_speed_scale_topic}, '
            f'use_joy_speed_scale_for_output={self.use_joy_speed_scale_for_output}, '
            f'heave_gain={self.heave_gain:.3f}, '
            f'horizontal_output_gain={self.horizontal_output_gain:.3f}, '
            f'yaw_output_gain={self.yaw_output_gain:.3f}, '
            f'vertical_output_gain={self.vertical_output_gain:.3f}, '
            f'rear_vertical_bias={self.rear_vertical_bias:.3f}, '
            f'pitch_torque_gain={self.pitch_torque_gain:.3f}, '
            f'torque_first_allocation={self.torque_first_allocation}, '
            f'slew_rate={self.slew_rate:.3f}/s, '
            f'max_output={self.max_output:.3f}, '
            f'output_scale={self.output_scale:.3f}, '
            f'output_deadband={self.output_deadband:.3f}, '
            f'level_horizontal_compensation='
            f'{self.level_horizontal_compensation_enabled}, '
            f'gain={self.level_horizontal_compensation_gain:.3f}, '
            f'max={self.level_horizontal_compensation_max:.3f}, '
            f'heave_sign={self.level_horizontal_compensation_heave_sign:.3f}, '
            f'min_z={self.level_horizontal_compensation_min_z:.3f}, '
            f'level_comp_spare_only={self.level_horizontal_compensation_uses_spare_only}, '
            f'attitude_priority_horizontal_slowdown='
            f'{self.attitude_priority_horizontal_slowdown_enabled}, '
            f'start={self.attitude_priority_horizontal_slowdown_start:.3f}, '
            f'full={self.attitude_priority_horizontal_slowdown_full:.3f}, '
            f'min_scale={self.attitude_priority_horizontal_min_scale:.3f}, '
            f'surge_pitch_moment_comp={self.surge_pitch_moment_compensation_enabled}, '
            f'surge_pitch_gain={self.surge_pitch_moment_gain:.3f}, '
            f'surge_pitch_max={self.surge_pitch_moment_max:.3f}, '
            f'surge_pitch_min_surge={self.surge_pitch_moment_min_surge:.3f}, '
            f'surge_pitch_target_gate={self.surge_pitch_moment_target_gate_enabled}, '
            f'imu_pitch_hold_comp={self.imu_pitch_hold_compensation_enabled}, '
            f'imu_pitch_hold_gain={self.imu_pitch_hold_gain:.3f}, '
            f'imu_pitch_hold_max={self.imu_pitch_hold_max:.3f}, '
            f'roll_torque_gain={self.roll_torque_gain:.3f}, '
            f'pitch_torque_gain={self.pitch_torque_gain:.3f}, '
            f'yaw_torque_gain={self.yaw_torque_gain:.3f}'
        )

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        self.world_z_from_body = quat_to_rotation_z_row(q.x, q.y, q.z, q.w)
        self.current_roll, self.current_pitch, self.current_yaw = quat_to_rpy(
            q.x, q.y, q.z, q.w
        )
        self.have_imu = True

    def cmd_attitude_callback(self, msg: Vector3):
        if not math.isnan(float(msg.x)):
            self.target_roll = float(msg.x)
        if not math.isnan(float(msg.y)):
            self.target_pitch = float(msg.y)

    def cmd_attitude_trim_callback(self, msg: Vector3):
        if not math.isnan(float(msg.x)):
            self.target_roll = float(msg.x)
        if not math.isnan(float(msg.y)):
            self.target_pitch = float(msg.y)

    def output_scale_callback(self, msg: Float64):
        self.output_scale = float(np.clip(float(msg.data), 0.0, 1.0))

    def joy_speed_scale_callback(self, msg: Float64):
        if self.use_joy_speed_scale_for_output:
            self.output_scale_callback(msg)

    def level_horizontal_heave_compensation(self, fx: float, fy: float) -> float:
        if not self.level_horizontal_compensation_enabled or not self.have_imu:
            return 0.0

        zx, zy, zz = self.world_z_from_body
        min_z = max(1e-6, abs(float(self.level_horizontal_compensation_min_z)))
        if abs(zz) < min_z:
            return 0.0

        comp = -(zx * fx + zy * fy) / zz
        comp *= float(self.level_horizontal_compensation_gain)
        comp *= float(self.level_horizontal_compensation_heave_sign)
        limit = max(0.0, abs(float(self.level_horizontal_compensation_max)))
        return float(np.clip(comp, -limit, limit))

    def attitude_priority_horizontal_scale(self, tx_scaled: float, ty_scaled: float) -> float:
        if not self.attitude_priority_horizontal_slowdown_enabled:
            return 1.0

        demand = max(abs(float(tx_scaled)), abs(float(ty_scaled)))
        start = max(0.0, float(self.attitude_priority_horizontal_slowdown_start))
        full = max(start + 1e-6, float(self.attitude_priority_horizontal_slowdown_full))
        min_scale = float(np.clip(self.attitude_priority_horizontal_min_scale, 0.0, 1.0))

        if demand <= start:
            return 1.0
        if demand >= full:
            return min_scale

        ratio = (demand - start) / (full - start)
        return 1.0 - ratio * (1.0 - min_scale)

    def surge_pitch_moment_compensation(self, fx: float) -> float:
        if not self.surge_pitch_moment_compensation_enabled:
            return 0.0
        if abs(float(fx)) < max(0.0, float(self.surge_pitch_moment_min_surge)):
            return 0.0

        target_pitch = float(self.target_pitch)
        target_abs = abs(target_pitch)
        if self.surge_pitch_moment_target_gate_enabled:
            min_target = max(0.0, float(self.surge_pitch_moment_min_target))
            if target_abs < min_target:
                return 0.0

            # Problem case: moving in the same geometric direction as the trim
            # makes surge thrust create a pitch moment that levels the vehicle.
            # Internal fx is inverted from the pilot surge command.
            if target_pitch * float(fx) >= 0.0:
                return 0.0

        full_target = max(
            max(0.0, float(self.surge_pitch_moment_min_target)) + 1e-6,
            abs(float(self.surge_pitch_moment_full_target))
        )
        target_scale = float(np.clip(target_abs / full_target, 0.0, 1.0))

        comp = float(self.surge_pitch_moment_gain) * float(fx) * target_scale
        limit = max(0.0, abs(float(self.surge_pitch_moment_max)))
        return float(np.clip(comp, -limit, limit))

    def imu_pitch_hold_compensation(self, fx: float) -> float:
        if not self.imu_pitch_hold_compensation_enabled or not self.have_imu:
            return 0.0
        if abs(float(fx)) < max(0.0, float(self.imu_pitch_hold_min_surge)):
            return 0.0

        pitch_error = self.target_pitch - self.current_pitch
        if abs(pitch_error) < max(0.0, float(self.imu_pitch_hold_deadband)):
            return 0.0

        drive = min(1.0, abs(float(fx)))
        comp = float(self.imu_pitch_hold_gain) * pitch_error * drive
        limit = max(0.0, abs(float(self.imu_pitch_hold_max)))
        return float(np.clip(comp, -limit, limit))

    def init_matrices(self):
        # =========================
        # thrusters.yaml / model.sdf 기반
        # 1~4: horizontal (Fx, Fy, Tz)
        # 5~8: vertical   (Fz, Tx, Ty)
        # 이 값이 실제 모델과 어긋나면 heave 시 불필요한 자세 토크가 생긴다.
        # =========================
        thrusters = [
            # horizontal 1~4
            ([0.20, -0.13, 0.0], [0.7071, -0.7071, 0.0]),
            ([0.20,  0.13, 0.0], [0.7071,  0.7071, 0.0]),
            ([-0.20, -0.13, 0.0], [0.7071,  0.7071, 0.0]),
            ([-0.20,  0.13, 0.0], [0.7071, -0.7071, 0.0]),

            # vertical 5~8
            ([0.20, -0.27, 0.0], [0.0, 0.0, 1.0]),
            ([0.20,  0.27, 0.0], [0.0, 0.0, 1.0]),
            ([-0.20, -0.27, 0.0], [0.0, 0.0, 1.0]),
            ([-0.20,  0.27, 0.0], [0.0, 0.0, 1.0]),
        ]

        self.TAM = np.zeros((6, 8), dtype=float)

        for i, (pos, direc) in enumerate(thrusters):
            p = np.array(pos, dtype=float)
            d = normalize(direc)

            # force
            self.TAM[0:3, i] = d

            # torque = r x F
            self.TAM[3:6, i] = np.cross(p, d)

        self.get_logger().info(f'Full TAM:\n{self.TAM}')

        # -------------------------
        # Horizontal submatrix
        # rows: Fx, Fy, Tz
        # cols: thrusters 1~4
        # -------------------------
        self.H = np.array([
            self.TAM[0, 0:4],   # Fx
            self.TAM[1, 0:4],   # Fy
            self.TAM[5, 0:4],   # Tz
        ], dtype=float)

        self.H_pinv = np.linalg.pinv(self.H)

        # -------------------------
        # Vertical submatrix
        # rows: Fz, Tx, Ty
        # cols: thrusters 5~8
        # -------------------------
        self.V = np.array([
            self.TAM[2, 4:8],   # Fz
            self.TAM[3, 4:8],   # Tx
            self.TAM[4, 4:8],   # Ty
        ], dtype=float)

        self.V_pinv = np.linalg.pinv(self.V)

        self.get_logger().info(f'Horizontal matrix H:\n{self.H}')
        self.get_logger().info(f'Vertical matrix V:\n{self.V}')

    def apply_deadband(self, arr):
        out = np.array(arr, dtype=float)
        out[np.abs(out) < self.output_deadband] = 0.0
        return out

    def add_component_with_headroom(self, base, addition):
        base = np.array(base, dtype=float)
        addition = np.array(addition, dtype=float)
        if float(np.max(np.abs(addition))) < 1e-9:
            return base

        if float(np.max(np.abs(base + addition))) <= 1.0:
            return base + addition

        scale = 1.0
        for base_value, add_value in zip(base, addition):
            if abs(add_value) < 1e-9:
                continue
            if add_value > 0.0:
                allowed = (1.0 - base_value) / add_value
            else:
                allowed = (-1.0 - base_value) / add_value
            scale = min(scale, allowed)

        scale = float(np.clip(scale, 0.0, 1.0))
        return base + addition * scale

    def allocate_priority_components(self, components):
        out = np.zeros(4, dtype=float)
        for component in components:
            out = self.add_component_with_headroom(out, component)
        return out

    def apply_slew_rate(self, target):
        target = np.array(target, dtype=float)
        now = self.get_clock().now()

        if self.last_slew_time is None:
            dt = 0.0
        else:
            dt = (now - self.last_slew_time).nanoseconds * 1e-9

        self.last_slew_time = now

        if dt <= 0.0:
            max_delta = 0.0
        else:
            # Cap dt so a callback pause does not allow one large output jump.
            max_delta = max(0.0, self.slew_rate) * min(dt, 0.1)

        delta = target - self.prev_thrust
        delta = np.clip(delta, -max_delta, max_delta)
        out = self.prev_thrust + delta
        self.prev_thrust = out.copy()
        return out

    def callback(self, msg: Wrench):
        # -------------------------
        # 원하는 wrench
        # -------------------------
        fx = float(msg.force.x)*-1
        fy = float(msg.force.y)
        fz = float(msg.force.z)
        tx = float(msg.torque.x)
        ty = float(msg.torque.y)
        tz = float(msg.torque.z)

        # Keep pilot/position horizontal motion level in the world frame while
        # the vehicle is intentionally pitched or rolled for attitude hold.
        level_comp_fz = self.level_horizontal_heave_compensation(fx, fy)

        # gain 적용
        fz_scaled = self.heave_gain * fz
        level_comp_fz_scaled = self.heave_gain * level_comp_fz
        tx_scaled = self.roll_torque_gain * tx
        ty_scaled = (
            self.pitch_torque_gain * ty +
            self.surge_pitch_moment_compensation(fx) +
            self.imu_pitch_hold_compensation(fx)
        )
        tz_scaled = self.yaw_torque_gain * tz

        horizontal_scale = self.attitude_priority_horizontal_scale(tx_scaled, ty_scaled)
        fx *= horizontal_scale
        fy *= horizontal_scale

        # -------------------------
        # horizontal allocation
        #   1~4번 thruster만 사용
        #   Fx, Fy, Tz 담당
        # -------------------------
        horiz_force_cmd = np.array([fx, fy, 0.0], dtype=float)
        horiz_yaw_cmd = np.array([0.0, 0.0, tz_scaled], dtype=float)
        u_h = (
            (self.H_pinv @ horiz_force_cmd) * self.horizontal_output_gain +
            (self.H_pinv @ horiz_yaw_cmd) * self.yaw_output_gain
        )

        # -------------------------
        # vertical allocation
        #   5~8번 thruster만 사용
        #   Fz, Tx, Ty 담당
        # -------------------------
        vert_heave_cmd = np.array([fz_scaled, 0.0, 0.0], dtype=float)
        vert_torque_cmd = np.array([0.0, tx_scaled, ty_scaled], dtype=float)
        vert_level_cmd = np.array([level_comp_fz_scaled, 0.0, 0.0], dtype=float)

        u_v_heave = (self.V_pinv @ vert_heave_cmd) * self.vertical_output_gain
        u_v_torque = (self.V_pinv @ vert_torque_cmd) * self.vertical_output_gain
        u_v_level = (self.V_pinv @ vert_level_cmd) * self.vertical_output_gain

        if self.torque_first_allocation:
            components = [u_v_torque, u_v_heave]
        else:
            components = [u_v_heave, u_v_torque]

        u_v = self.allocate_priority_components(components)
        if self.level_horizontal_compensation_uses_spare_only:
            u_v = self.add_component_with_headroom(u_v, u_v_level)
        else:
            u_v = u_v + u_v_level

        # Rear-heavy trim: calibrated pitch feed-forward. With the current
        # thruster signs/mapping, this raises the rear relative to the front.
        vertical_active = (
            np.linalg.norm([fz_scaled, level_comp_fz_scaled, tx_scaled, ty_scaled]) > 1e-6
        )
        if vertical_active and abs(self.rear_vertical_bias) > 1e-9:
            u_v[0] += self.rear_vertical_bias
            u_v[1] += self.rear_vertical_bias
            u_v[2] -= self.rear_vertical_bias
            u_v[3] -= self.rear_vertical_bias

        # -------------------------
        # combine (decoupled normalization)
        # horizontal(1~4) and vertical(5~8) groups are normalized independently
        # so yaw demand won't scale down vertical heave compensation.
        # -------------------------
        u_h = normalize_group_unit(u_h)
        u_v = normalize_group_unit(u_v)

        thrust = np.zeros(8, dtype=float)
        thrust[0:4] = u_h
        thrust[4:8] = u_v

        # 실제 plugin / can 방향 반대일 경우 대비
        thrust = thrust * self.signs
        output_scale = float(np.clip(self.output_scale, 0.0, 1.0))
        thrust *= output_scale
        max_output = float(np.clip(self.max_output, 0.0, 1.0))
        thrust = np.clip(thrust, -max_output, max_output)

        # -------------------------
        # slew rate + deadband
        # -------------------------
        thrust = self.apply_slew_rate(thrust)
        thrust = self.apply_deadband(thrust)

        # -------------------------
        # publish
        # -------------------------
        out = Float64MultiArray()
        out.data = thrust.tolist()
        self.pub.publish(out)

        # -------------------------
        # debug log
        # -------------------------
        # self.get_logger().info(
        #     f'wrench=[Fx={fx:.3f}, Fy={fy:.3f}, Fz={fz:.3f}, '
        #     f'Tx={tx:.3f}, Ty={ty:.3f}, Tz={tz:.3f}]'
        # )
        # self.get_logger().info(
        #     f'scaled=[Fx={fx:.3f}, Fy={fy:.3f}, Fz={fz_scaled:.3f}, '
        #     f'Tx={tx_scaled:.3f}, Ty={ty_scaled:.3f}, Tz={tz_scaled:.3f}]'
        # )
        # self.get_logger().info(
        #     f'u_h={np.round(u_h, 4).tolist()}, '
        #     f'u_v={np.round(u_v, 4).tolist()}'
        # )
        # self.get_logger().info(
        #     f'thrust(norm)={np.round(thrust, 4).tolist()}'
        # )

    def on_parameter_update(self, params):
        try:
            for p in params:
                if p.name == 'heave_gain':
                    self.heave_gain = float(p.value)
                elif p.name == 'horizontal_output_gain':
                    self.horizontal_output_gain = float(p.value)
                elif p.name == 'yaw_output_gain':
                    self.yaw_output_gain = float(p.value)
                elif p.name == 'vertical_output_gain':
                    self.vertical_output_gain = float(p.value)
                elif p.name == 'rear_vertical_bias':
                    self.rear_vertical_bias = float(p.value)
                elif p.name == 'pitch_torque_gain':
                    self.pitch_torque_gain = float(p.value)
                elif p.name == 'torque_first_allocation':
                    self.torque_first_allocation = bool(p.value)
                elif p.name == 'level_horizontal_compensation_enabled':
                    self.level_horizontal_compensation_enabled = bool(p.value)
                elif p.name == 'level_horizontal_compensation_gain':
                    self.level_horizontal_compensation_gain = float(p.value)
                elif p.name == 'level_horizontal_compensation_max':
                    self.level_horizontal_compensation_max = float(p.value)
                elif p.name == 'level_horizontal_compensation_heave_sign':
                    self.level_horizontal_compensation_heave_sign = float(p.value)
                elif p.name == 'level_horizontal_compensation_min_z':
                    self.level_horizontal_compensation_min_z = float(p.value)
                elif p.name == 'level_horizontal_compensation_uses_spare_only':
                    self.level_horizontal_compensation_uses_spare_only = bool(p.value)
                elif p.name == 'attitude_priority_horizontal_slowdown_enabled':
                    self.attitude_priority_horizontal_slowdown_enabled = bool(p.value)
                elif p.name == 'attitude_priority_horizontal_slowdown_start':
                    self.attitude_priority_horizontal_slowdown_start = float(p.value)
                elif p.name == 'attitude_priority_horizontal_slowdown_full':
                    self.attitude_priority_horizontal_slowdown_full = float(p.value)
                elif p.name == 'attitude_priority_horizontal_min_scale':
                    self.attitude_priority_horizontal_min_scale = float(p.value)
                elif p.name == 'surge_pitch_moment_compensation_enabled':
                    self.surge_pitch_moment_compensation_enabled = bool(p.value)
                elif p.name == 'surge_pitch_moment_gain':
                    self.surge_pitch_moment_gain = float(p.value)
                elif p.name == 'surge_pitch_moment_max':
                    self.surge_pitch_moment_max = float(p.value)
                elif p.name == 'surge_pitch_moment_min_surge':
                    self.surge_pitch_moment_min_surge = float(p.value)
                elif p.name == 'surge_pitch_moment_target_gate_enabled':
                    self.surge_pitch_moment_target_gate_enabled = bool(p.value)
                elif p.name == 'surge_pitch_moment_min_target_deg':
                    self.surge_pitch_moment_min_target = math.radians(float(p.value))
                elif p.name == 'surge_pitch_moment_full_target_deg':
                    self.surge_pitch_moment_full_target = math.radians(float(p.value))
                elif p.name == 'imu_pitch_hold_compensation_enabled':
                    self.imu_pitch_hold_compensation_enabled = bool(p.value)
                elif p.name == 'imu_pitch_hold_gain':
                    self.imu_pitch_hold_gain = float(p.value)
                elif p.name == 'imu_pitch_hold_max':
                    self.imu_pitch_hold_max = float(p.value)
                elif p.name == 'imu_pitch_hold_deadband_deg':
                    self.imu_pitch_hold_deadband = math.radians(float(p.value))
                elif p.name == 'imu_pitch_hold_min_surge':
                    self.imu_pitch_hold_min_surge = float(p.value)
                elif p.name == 'slew_rate':
                    self.slew_rate = float(p.value)
                elif p.name == 'max_output':
                    self.max_output = float(p.value)
                elif p.name == 'output_scale':
                    self.output_scale = float(p.value)
                elif p.name == 'use_joy_speed_scale_for_output':
                    self.use_joy_speed_scale_for_output = bool(p.value)
                elif p.name == 'output_deadband':
                    self.output_deadband = float(p.value)

            self.get_logger().info(
                f'Allocator parameters updated: '
                f'heave_gain={self.heave_gain:.3f}, '
                f'horizontal_output_gain={self.horizontal_output_gain:.3f}, '
                f'yaw_output_gain={self.yaw_output_gain:.3f}, '
                f'vertical_output_gain={self.vertical_output_gain:.3f}, '
                f'rear_vertical_bias={self.rear_vertical_bias:.3f}, '
                f'pitch_torque_gain={self.pitch_torque_gain:.3f}, '
                f'torque_first_allocation={self.torque_first_allocation}, '
                f'level_horizontal_compensation={self.level_horizontal_compensation_enabled}, '
                f'level_horizontal_compensation_gain={self.level_horizontal_compensation_gain:.3f}, '
                f'level_horizontal_compensation_max={self.level_horizontal_compensation_max:.3f}, '
                f'level_horizontal_compensation_heave_sign={self.level_horizontal_compensation_heave_sign:.3f}, '
                f'level_horizontal_compensation_spare_only={self.level_horizontal_compensation_uses_spare_only}, '
                f'attitude_priority_horizontal_slowdown={self.attitude_priority_horizontal_slowdown_enabled}, '
                f'attitude_priority_horizontal_min_scale={self.attitude_priority_horizontal_min_scale:.3f}, '
                f'surge_pitch_moment_comp={self.surge_pitch_moment_compensation_enabled}, '
                f'surge_pitch_moment_gain={self.surge_pitch_moment_gain:.3f}, '
                f'surge_pitch_moment_min_surge={self.surge_pitch_moment_min_surge:.3f}, '
                f'surge_pitch_moment_target_gate={self.surge_pitch_moment_target_gate_enabled}, '
                f'imu_pitch_hold_comp={self.imu_pitch_hold_compensation_enabled}, '
                f'imu_pitch_hold_gain={self.imu_pitch_hold_gain:.3f}, '
                f'slew_rate={self.slew_rate:.3f}/s, '
                f'max_output={self.max_output:.3f}, '
                f'output_scale={self.output_scale:.3f}, '
                f'use_joy_speed_scale_for_output={self.use_joy_speed_scale_for_output}, '
                f'output_deadband={self.output_deadband:.3f}'
            )
            return SetParametersResult(successful=True)
        except Exception as e:
            return SetParametersResult(successful=False, reason=str(e))


def main(args=None):
    rclpy.init(args=args)
    node = AllocatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
