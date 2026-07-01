# Attitude Controller Review

대상 파일: `code_review/code/attitude_controller.py`

## 역할
이 코드는 IMU와 수동 wrench를 받아 ROV의 `roll`, `pitch`, `yaw` 토크를 만드는 자세 제어기입니다.

## 설계 해석
설계상 핵심은 `roll/pitch stabilizer + yaw heading hold`입니다. 즉 단순히 자세 오차만 줄이는 것이 아니라, 조종자의 yaw 조작과 translational motion 중에도 trim attitude와 heading hold를 유지하려는 운용 감각이 강하게 들어가 있습니다.

## 리뷰 초점
리뷰 포인트는 `목표 자세를 어떻게 잡는지`, `IMU를 어떻게 제어 상태로 필터링하는지`, `yaw manual override와 hold가 어떻게 전환되는지`, `translation/heave 중 보호 로직이 어떤 출력을 만드는지`입니다.

## 런타임 동작 해설
런타임에서는 `imu_callback()`이 현재 자세와 각속도를 계속 갱신하고, `control_loop()`가 주기적으로 목표 자세와 현재 자세의 차이를 계산해 토크를 냅니다. 여기에 yaw stick release 이후 heading을 다시 잠그는 로직, translation 중 roll/pitch trim을 유지하는 로직, heave나 큰 body rate 상황에서 출력을 보호하는 로직이 겹쳐서 실제 조종 감각을 만듭니다.

## 핵심 파라미터
- `kp_roll`, `ki_roll`, `kd_roll`: roll 축 복원력, steady-state bias 보상, roll rate damping 강도를 정합니다.
- `kp_pitch`, `ki_pitch`, `kd_pitch`: pitch 축 응답과 감쇠를 정하며 surge 중 nose-up 경향을 얼마나 강하게 잡을지에도 영향을 줍니다.
- `kp_yaw`, `kd_yaw`: heading error를 얼마나 세게 복원할지와 yaw rate를 얼마나 감쇠할지 결정합니다.
- `yaw_manual_override_threshold`: 이 값보다 큰 yaw stick이 들어오면 yaw hold보다 manual yaw를 우선합니다.
- `yaw_hold_settle_time_sec`: yaw stick을 놓은 직후 heading을 다시 잠그기 전에 잠깐 damping만 적용하는 시간입니다.
- `xy_motion_protect_threshold`, `strong_xy_motion_threshold`: translation 중 roll/pitch trim hold를 약하게 만들기 시작하는 기준입니다.
- `rp_scale_when_xy_motion`, `rp_scale_when_strong_xy_motion`: translation 중 roll/pitch torque를 얼마나 줄일지 정합니다.
- `heave_protect_threshold`, `strong_heave_threshold`: 수직 조작 중 yaw hold를 더 보수적으로 만들지 결정하는 기준입니다.
- `large_tilt_disable_deg`: 기체가 너무 크게 기울면 torque 출력을 끊는 안전 게이트입니다.
- `orientation_filter_measurement_alpha`, `orientation_filter_max_correction_rate_deg`: IMU 기반 control attitude filter의 추종 속도와 correction 한계를 정합니다.

## 함수 맵
- `clamp()`
- `vec_norm()`
- `quat_normalize()`
- `quat_conj()`
- `quat_mul()`
- `quat_to_rpy()`
- `rpy_to_quat()`
- `wrap_to_pi()`
- `__init__()`
- `_update_target_quaternion()`
- `_force_level_roll_pitch_target()`
- `_capture_current_attitude_as_target()`
- `_capture_current_yaw_as_target()`
- `_set_control_enabled()`
- `imu_callback()`
- `manual_wrench_callback()`
- `cmd_attitude_callback()`
- `cmd_attitude_trim_callback()`
- `_apply_deadband()`
- `_reset_control_attitude_filter()`
- `_update_control_attitude_filter()`
- `_apply_rp_torque_slew()`
- `_translation_tilt_feedforward()`
- `control_loop()`
- `on_parameter_update()`
- `main()`

## 함수 리뷰

### Quaternion / Angle 유틸리티

**의미**

이 함수들은 제어기 본체보다 먼저, 자세 표현을 다루기 위한 공통 도구입니다. Quaternion을 정규화하고 곱하고 Euler angle로 바꾸는 역할을 맡습니다.

**영향**

이 유틸리티가 안정적이어야 목표 자세 계산과 yaw wrapping이 깨지지 않습니다. 특히 `wrap_to_pi()`는 yaw 오차가 `+π/-π` 경계에서 튀는 문제를 막아줍니다.

**리뷰 메모**

이 계층은 제어식의 기반이라서 작아 보여도 중요합니다. 자세 제어가 갑자기 반대로 튀거나 yaw 오차가 크게 보이는 문제는 여기서 시작되는 경우가 많습니다.

```python
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
```

### `__init__()`

**의미**

노드의 토픽, 게인, 제한값, 보호 로직, yaw hold 정책을 전부 선언하고 상태 변수를 초기화합니다.

**영향**

이 함수가 곧 제어기의 운용 정책 선언부입니다. 어떤 입력을 받고 어떤 보호 로직이 켜지는지, yaw stick을 놓았을 때 어떤 감각으로 복귀하는지가 여기서 결정됩니다.

**리뷰 메모**

파라미터가 많지만 모두 의미가 분명합니다. 다만 수동 입력 freshness를 위한 timestamp/state가 없는 점은 후반 제어 루프에서 stale manual state로 이어질 수 있습니다.

```python
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
```

### 목표 자세 캡처 계열

**의미**

이 함수들은 '지금 무엇을 목표 자세로 볼 것인가'를 정하는 계층입니다. 초기 캡처, level target 유지, yaw release 후 heading hold 재설정이 모두 여기서 일어납니다.

**영향**

사용자가 조작을 놓았을 때 기체가 어느 자세로 돌아갈지 결정합니다. 운용감이 좋은지, trim이 유지되는지, yaw hold가 자연스러운지가 이 계층에 달려 있습니다.

**리뷰 메모**

이 코드의 운용 감각은 꽤 좋습니다. 특히 yaw target을 현재 yaw로 다시 잡는 설계는 실제 조종 감각을 부드럽게 만듭니다.

```python
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
```

### `imu_callback()`

**의미**

IMU quaternion과 body rate를 읽어 현재 자세와 각속도를 최신 상태로 갱신합니다.

**영향**

제어기의 모든 피드백이 여기서 들어옵니다. 처음 IMU를 받으면 초기 target을 캡처하기 때문에, 이 함수는 센서 수신과 target initialization을 동시에 담당합니다.

**리뷰 메모**

초기 target capture와 IMU 상태 갱신이 한 곳에 모여 있어 흐름은 명확합니다. 다만 manual freshness와는 독립이라 이후 control loop가 오래된 manual 입력을 그대로 읽을 수 있습니다.

```python
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
```

### `_update_control_attitude_filter()`

**의미**

IMU 자세를 그대로 쓰지 않고, body rate 적분 예측값과 measurement를 섞어서 `control_roll`, `control_pitch`를 만듭니다.

**영향**

센서 노이즈와 순간 흔들림을 줄이고 제어 대상 자세를 더 부드럽게 만듭니다. 즉 이 함수는 제어 입력의 품질을 개선하는 내부 observer 역할을 합니다.

**리뷰 메모**

이 필터는 attitude hold를 덜 거칠게 만들어 주는 좋은 계층입니다. 또한 `dt`가 비정상이면 reset하도록 만들어 센서 타이밍 문제에 비교적 안전합니다.

```python
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
```

### `_translation_tilt_feedforward()`

**의미**

수동 surge/sway가 있을 때 미리 roll/pitch 토크를 보정해 translation 중 trim 자세 붕괴를 줄이려는 feedforward 함수입니다.

**영향**

이 함수가 켜지면 단순 PID 복원보다 더 적극적으로 자세를 유지하려고 합니다. 특히 기체가 이동하면서 생기는 pitch-up / roll-off 경향을 선제적으로 누를 수 있습니다.

**리뷰 메모**

기능 자체는 고급스럽지만, 튜닝 의존성이 큽니다. gain이 과하면 조종감이 뻣뻣해질 수 있어 실제 운용 로그와 함께 봐야 합니다.

```python
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
```

### `control_loop()`

**의미**

실제 제어법칙이 수행되는 중심 함수입니다. 자세 오차 계산, 적분, rate damping, yaw manual override, translation/heave 보호, slew limit, 최종 토크 publish가 모두 여기에 있습니다.

**영향**

이 함수의 동작이 곧 조종감입니다. 조종자가 translation 중인지, heave를 주는지, yaw stick을 놓았는지에 따라 토크 출력 정책이 바뀝니다.

**리뷰 메모**

구조는 매우 좋고 운용 의도도 분명합니다. 다만 manual wrench freshness가 없어서 `yaw_manual_active`, `heave_protect`, `xy_soft_trim`이 오래 남을 수 있는 리스크가 있습니다.

```python
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
```

### `on_parameter_update()`

**의미**

런타임 파라미터 갱신을 받아 제어기 상태와 게인을 즉시 바꾸는 함수입니다.

**영향**

GUI나 runtime tuning에서 실시간으로 gain과 정책을 바꿀 수 있게 해줍니다. 실험 속도를 크게 올리는 대신, 파라미터 변경 시 즉시 제어 감각이 달라집니다.

**리뷰 메모**

runtime tuning을 적극적으로 지원하는 점은 강점입니다. 동시에 리뷰 관점에서는 파라미터 변화가 control loop에 어떤 영향을 주는지 문서가 꼭 필요하다는 뜻이기도 합니다.

```python
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
```

## 전체 코드

```python
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
```
