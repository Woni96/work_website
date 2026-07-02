# ROV Control Code Review - 함수별 설명 문서

3장. `attitude_controller.py`

IMU 기반 Roll/Pitch/Yaw 자세 유지 토크를 생성하는 자세 제어 노드

이 파일은 IMU로 현재 자세를 읽고 목표 자세와 비교하여 roll/pitch/yaw 토크를 만듭니다. 수심, 위치 유지 중에도 기체 자세가 무너지지 않도록 하는 기반 제어기입니다.

- 파일: `attitude_controller.py`
- 함수 개수: 26
- 주요 역할: IMU 기반 Roll/Pitch/Yaw 자세 유지 토크를 생성하는 자세 제어 노드

3장.1 전역 함수.clamp()

- 위치: `attitude_controller.py:20-23`
- 입력: x, lo, hi
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: 값을 지정된 최소/최대 범위 안으로 제한합니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.
- 내부 동작 흐름:
  - 입력값과 최소/최대 한계를 받습니다.
  - 최솟값보다 작으면 최솟값으로 제한합니다.
  - 최댓값보다 크면 최댓값으로 제한하고, 범위 안이면 그대로 반환합니다.
- 코드 일부:

```python
def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))
```

3장.2 전역 함수.vec_norm()

- 위치: `attitude_controller.py:24-27`
- 입력: x, y, z
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: 3차원 벡터의 크기를 계산합니다. 각속도 크기 판단 등에 사용됩니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.
- 내부 동작 흐름:
  - x, y, z 성분을 읽습니다.
  - 각 성분 제곱합을 계산합니다.
  - 제곱근을 취해 벡터 크기를 반환합니다.
- 코드 일부:

```python
def vec_norm(x: float, y: float, z: float) -> float:
    return math.sqrt(x * x + y * y + z * z)
```

3장.3 전역 함수.quat_normalize()

- 위치: `attitude_controller.py:28-35`
- 입력: q
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: quaternion을 단위 quaternion으로 정규화합니다.
- 왜 사용했는가: ROV 제어에서는 자세 표현과 좌표계 변환이 계속 필요하므로, 반복되는 수학 연산을 함수로 분리한 것입니다.
- 제어 영향: roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.
- 내부 동작 흐름:
  - 입력 quaternion 성분을 분리합니다.
  - 노름을 계산합니다.
  - 노름이 너무 작으면 기본 단위 quaternion을 반환하고, 아니면 정규화 결과를 반환합니다.
- 코드 일부:

```python
def quat_normalize(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / n, y / n, z / n, w / n)
```

3장.4 전역 함수.quat_conj()

- 위치: `attitude_controller.py:36-40`
- 입력: q
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: quaternion의 켤레를 계산합니다. 회전 역변환에 사용됩니다.
- 왜 사용했는가: ROV 제어에서는 자세 표현과 좌표계 변환이 계속 필요하므로, 반복되는 수학 연산을 함수로 분리한 것입니다.
- 제어 영향: roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.
- 내부 동작 흐름:
  - 입력 quaternion 성분을 분리합니다.
  - 벡터부 부호를 반전합니다.
  - 스칼라부는 유지한 채 켤레 quaternion을 반환합니다.
- 코드 일부:

```python
def quat_conj(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x, y, z, w = q
    return (-x, -y, -z, w)
```

3장.5 전역 함수.quat_mul()

- 위치: `attitude_controller.py:41-52`
- 입력: 없음
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: 두 quaternion의 곱을 계산합니다. 자세 오차 또는 벡터 회전에 사용됩니다.
- 왜 사용했는가: ROV 제어에서는 자세 표현과 좌표계 변환이 계속 필요하므로, 반복되는 수학 연산을 함수로 분리한 것입니다.
- 제어 영향: roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.
- 내부 동작 흐름:
  - 두 quaternion의 성분을 각각 분리합니다.
  - Hamilton product 공식을 적용합니다.
  - 곱셈 결과 quaternion을 반환합니다.
- 코드 일부:

```python
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
```

3장.6 전역 함수.quat_to_rpy()

- 위치: `attitude_controller.py:53-70`
- 입력: x, y, z, w
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: quaternion 자세 표현을 roll, pitch, yaw 각도로 변환합니다. 사람이 이해하기 쉬운 자세 오차 및 보상 계산에 사용됩니다.
- 왜 사용했는가: ROV 제어에서는 자세 표현과 좌표계 변환이 계속 필요하므로, 반복되는 수학 연산을 함수로 분리한 것입니다.
- 제어 영향: roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.
- 내부 동작 흐름:
  - quaternion 성분으로 roll을 계산합니다.
  - pitch를 계산하며 asin 범위를 넘어설 경우 안전하게 처리합니다.
  - yaw를 계산해 세 각도를 반환합니다.
- 코드 일부:

```python
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
```

3장.7 전역 함수.rpy_to_quat()

- 위치: `attitude_controller.py:71-85`
- 입력: roll, pitch, yaw
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: roll, pitch, yaw 목표를 quaternion으로 변환합니다.
- 왜 사용했는가: ROV 제어에서는 자세 표현과 좌표계 변환이 계속 필요하므로, 반복되는 수학 연산을 함수로 분리한 것입니다.
- 제어 영향: roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.
- 내부 동작 흐름:
  - roll, pitch, yaw의 half-angle 삼각함수를 계산합니다.
  - quaternion 성분을 조합합니다.
  - 정규화된 목표 quaternion을 반환합니다.
- 코드 일부:

```python
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
```

3장.8 전역 함수.wrap_to_pi()

- 위치: `attitude_controller.py:86-89`
- 입력: angle
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: 각도를 -pi~pi 범위로 정규화합니다. yaw wrap 문제를 방지합니다.
- 왜 사용했는가: ROV 제어에서는 자세 표현과 좌표계 변환이 계속 필요하므로, 반복되는 수학 연산을 함수로 분리한 것입니다.
- 제어 영향: roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.
- 내부 동작 흐름:
  - 입력 각도의 sin과 cos를 계산합니다.
  - `atan2`를 이용해 같은 방향의 대표 각도로 변환합니다.
  - -pi~pi 범위의 각도를 반환합니다.
- 코드 일부:

```python
def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))
```

3장.9 AttitudeController.__init__()

- 위치: `attitude_controller.py:92-368`
- 입력: self
- 출력: 직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다.
- 역할: ROS2 노드의 파라미터, 상태 변수, subscriber, publisher, timer를 초기화합니다. 해당 제어 노드가 시스템에 연결되는 시작점입니다.
- 왜 사용했는가: 노드가 실행되기 전에 필요한 파라미터, 통신 인터페이스, 상태 변수를 모두 준비해야 하기 때문에 사용됩니다.
- 제어 영향: roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.
- 내부 동작 흐름:
  - 노드 이름을 설정합니다.
  - ROS parameter를 선언하고 현재 값을 읽습니다.
  - 제어에 필요한 내부 상태 변수를 초기화합니다.
  - subscriber와 publisher를 생성합니다.
  - timer와 parameter callback을 등록합니다.
  - 초기 설정값을 log로 출력합니다.
- 코드 일부:

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

3장.10 AttitudeController._update_target_quaternion()

- 위치: `attitude_controller.py:369-372`
- 입력: self
- 출력: 내부 상태 갱신이 중심이며, 필요 시 계산 결과를 return합니다.
- 역할: 현재 target roll/pitch/yaw로부터 목표 quaternion을 갱신합니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.
- 내부 동작 흐름:
  - 현재 target roll/pitch/yaw를 읽습니다.
  - `rpy_to_quat()`로 목표 quaternion을 계산합니다.
  - target initialized 상태를 참으로 갱신합니다.
- 코드 일부:

```python
def _update_target_quaternion(self):
    self.q_target = rpy_to_quat(self.target_roll, self.target_pitch, self.target_yaw)
    self.target_initialized = True
```

3장.11 AttitudeController._force_level_roll_pitch_target()

- 위치: `attitude_controller.py:373-379`
- 입력: self
- 출력: 내부 상태 갱신이 중심이며, 필요 시 계산 결과를 return합니다.
- 역할: level target과 trim 값을 이용해 roll/pitch 목표를 강제로 갱신합니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.
- 내부 동작 흐름:
  - level target 사용 여부를 확인합니다.
  - target roll/pitch와 trim 값을 radian으로 변환해 목표값을 갱신합니다.
  - 갱신된 목표로 quaternion을 다시 계산합니다.
- 코드 일부:

```python
def _force_level_roll_pitch_target(self):
    if not self.level_roll_pitch_target:
        return
    self.target_roll = math.radians(self.target_roll_deg + self.roll_trim_deg)
    self.target_pitch = math.radians(self.target_pitch_deg + self.pitch_trim_deg)
    self._update_target_quaternion()
```

3장.12 AttitudeController._capture_current_attitude_as_target()

- 위치: `attitude_controller.py:380-389`
- 입력: self
- 출력: 내부 상태 갱신이 중심이며, 필요 시 계산 결과를 return합니다.
- 역할: 현재 자세를 제어 목표로 캡처합니다. 초기화 또는 제어 재활성화 시 사용됩니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.
- 내부 동작 흐름:
  - 현재 yaw를 wrap 처리해 목표 yaw로 저장합니다.
  - level mode이면 설정된 level/trim 목표를 사용하고, 아니면 현재 control roll/pitch를 사용합니다.
  - 새 목표 quaternion을 계산합니다.
- 코드 일부:

```python
def _capture_current_attitude_as_target(self):
    self.target_yaw = wrap_to_pi(self.yaw)
    if self.level_roll_pitch_target:
        self.target_roll = math.radians(self.target_roll_deg + self.roll_trim_deg)
        self.target_pitch = math.radians(self.target_pitch_deg + self.pitch_trim_deg)
    else:
        self.target_roll = self.control_roll
        self.target_pitch = self.control_pitch
    self._update_target_quaternion()
```

3장.13 AttitudeController._capture_current_yaw_as_target()

- 위치: `attitude_controller.py:390-393`
- 입력: self
- 출력: 내부 상태 갱신이 중심이며, 필요 시 계산 결과를 return합니다.
- 역할: 현재 yaw를 heading hold 목표로 캡처합니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.
- 내부 동작 흐름:
  - 현재 yaw를 wrap 처리합니다.
  - 목표 yaw에 저장합니다.
  - 새 목표 quaternion을 계산합니다.
- 코드 일부:

```python
def _capture_current_yaw_as_target(self):
    self.target_yaw = wrap_to_pi(self.yaw)
    self._update_target_quaternion()
```

3장.14 AttitudeController._set_control_enabled()

- 위치: `attitude_controller.py:394-411`
- 입력: self, enabled
- 출력: 내부 상태 갱신이 중심이며, 필요 시 계산 결과를 return합니다.
- 역할: 제어 enable 상태 변경 시 목표값, 적분항, 출력 상태를 초기화하거나 0 출력합니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.
- 내부 동작 흐름:
  - 이전 enable 상태와 새 상태를 비교합니다.
  - enable 전이 시 필터와 목표 자세, 적분항을 초기화합니다.
  - disable 전이 시 0 torque publish와 상태 정리를 수행합니다.
- 코드 일부:

```python
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
```

3장.15 AttitudeController.imu_callback()

- 위치: `attitude_controller.py:412-435`
- 입력: self, msg
- 출력: 직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다.
- 역할: IMU 메시지를 수신하여 현재 자세, 각속도, 또는 z축 방향 정보를 내부 상태에 저장합니다.
- 왜 사용했는가: ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.
- 제어 영향: roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.
- 내부 동작 흐름:
  - ROS2 메시지를 수신합니다.
  - orientation을 정규화하고 roll/pitch/yaw를 계산합니다.
  - 각속도와 현재 자세 상태를 갱신하고 필요하면 초기 target을 캡처합니다.
- 코드 일부:

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

3장.16 AttitudeController.manual_wrench_callback()

- 위치: `attitude_controller.py:436-438`
- 입력: self, msg
- 출력: 직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다.
- 역할: 조종기 또는 상위 입력에서 들어오는 수동 Wrench 명령을 저장합니다.
- 왜 사용했는가: ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.
- 제어 영향: roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.
- 내부 동작 흐름:
  - ROS2 메시지를 수신합니다.
  - 수동 wrench를 내부 상태에 저장합니다.
  - 이후 control loop가 사용할 최신 manual 입력으로 유지합니다.
- 코드 일부:

```python
def manual_wrench_callback(self, msg: Wrench):
    self.manual_wrench = msg
```

3장.17 AttitudeController.cmd_attitude_callback()

- 위치: `attitude_controller.py:439-463`
- 입력: self, msg
- 출력: 직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다.
- 역할: 외부에서 들어오는 목표 자세 명령을 내부 목표 roll/pitch/yaw 값으로 반영합니다.
- 왜 사용했는가: ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.
- 제어 영향: roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.
- 내부 동작 흐름:
  - ROS2 메시지를 수신합니다.
  - NaN이 아닌 목표 roll/pitch/yaw를 읽습니다.
  - 업데이트가 있으면 목표 quaternion을 다시 계산합니다.
- 코드 일부:

```python
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
```

3장.18 AttitudeController.cmd_attitude_trim_callback()

- 위치: `attitude_controller.py:464-484`
- 입력: self, msg
- 출력: 직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다.
- 역할: trim 형태의 목표 자세를 수신하여 roll/pitch 보정 기준으로 사용합니다.
- 왜 사용했는가: ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.
- 제어 영향: roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.
- 내부 동작 흐름:
  - ROS2 메시지를 수신합니다.
  - NaN이 아닌 trim 값을 읽어 degree 기준 trim 상태를 갱신합니다.
  - 업데이트가 있으면 필요 시 level target을 다시 구성합니다.
- 코드 일부:

```python
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
```

3장.19 AttitudeController._apply_deadband()

- 위치: `attitude_controller.py:485-487`
- 입력: self, x
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: 작은 torque 값을 0으로 만들어 미세 떨림과 불필요한 토크 출력을 줄입니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.
- 내부 동작 흐름:
  - 입력 torque 크기를 확인합니다.
  - `torque_deadband`보다 작으면 0으로 만듭니다.
  - 그 외에는 원래 값을 반환합니다.
- 코드 일부:

```python
def _apply_deadband(self, x: float) -> float:
    return 0.0 if abs(x) < self.torque_deadband else x
```

3장.20 AttitudeController._reset_control_attitude_filter()

- 위치: `attitude_controller.py:488-492`
- 입력: self
- 출력: 내부 상태 갱신이 중심이며, 필요 시 계산 결과를 return합니다.
- 역할: roll/pitch 제어용 필터 상태를 현재 IMU 자세로 초기화합니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.
- 내부 동작 흐름:
  - 현재 roll/pitch를 읽습니다.
  - control roll/pitch 상태를 현재 자세로 맞춥니다.
  - 필터 초기화 완료 플래그를 켭니다.
- 코드 일부:

```python
def _reset_control_attitude_filter(self):
    self.control_roll = self.roll
    self.control_pitch = self.pitch
    self.control_attitude_filter_initialized = True
```

3장.21 AttitudeController._update_control_attitude_filter()

- 위치: `attitude_controller.py:493-516`
- 입력: self, dt
- 출력: 내부 상태 갱신이 중심이며, 필요 시 계산 결과를 return합니다.
- 역할: 각속도 적분 예측과 IMU 측정을 섞어 roll/pitch 제어용 자세 값을 갱신합니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.
- 내부 동작 흐름:
  - 필터 활성 여부와 dt 유효성을 확인합니다.
  - body rate 적분으로 예측값을 계산합니다.
  - measurement와의 차이를 제한된 correction으로 반영해 control roll/pitch를 갱신합니다.
- 코드 일부:

```python
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

3장.22 AttitudeController._apply_rp_torque_slew()

- 위치: `attitude_controller.py:517-528`
- 입력: self, tx_ctrl, ty_ctrl, dt
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: roll/pitch 제어 토크 변화량을 제한합니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.
- 내부 동작 흐름:
  - dt로 허용 가능한 최대 토크 변화량을 계산합니다.
  - 현재 목표 torque를 이전 torque 주변 허용 범위로 clamp합니다.
  - 갱신된 torque를 저장하고 반환합니다.
- 코드 일부:

```python
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
```

3장.23 AttitudeController._translation_tilt_feedforward()

- 위치: `attitude_controller.py:529-543`
- 입력: self, manual_surge, manual_sway
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: 수평 이동 중 목표 roll/pitch trim 유지에 필요한 feed-forward 토크를 계산합니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.
- 내부 동작 흐름:
  - 기능 활성 여부를 확인합니다.
  - surge/sway 입력에서 deadband를 뺀 유효 drive를 계산합니다.
  - 설정 gain과 최대치로 roll/pitch feed-forward torque를 만듭니다.
- 코드 일부:

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

3장.24 AttitudeController.control_loop()

- 위치: `attitude_controller.py:544-754`
- 입력: self
- 출력: 내부 상태 갱신이 중심이며, 필요 시 계산 결과를 return합니다.
- 역할: 자세 오차와 각속도 damping으로 roll/pitch/yaw 토크를 계산하고 발행하는 주 제어 루프입니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.
- 내부 동작 흐름:
  - IMU와 target 초기화 여부를 확인합니다.
  - dt를 계산하고 control_enabled 상태를 확인합니다.
  - roll/pitch 필터를 갱신하고 목표 자세를 준비합니다.
  - 수동 wrench 입력에서 heave, surge, sway, yaw 명령을 읽습니다.
  - roll/pitch/yaw 오차와 각속도 damping으로 제어 토크를 계산합니다.
  - 수동 yaw 조작 중에는 yaw hold를 양보하고, release 시 현재 yaw를 목표로 캡처합니다.
  - 계산된 토크를 limit와 slew-rate 처리 후 attitude torque topic으로 발행합니다.
- 코드 일부:

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

3장.25 AttitudeController.on_parameter_update()

- 위치: `attitude_controller.py:755-871`
- 입력: self, params
- 출력: 파라미터 갱신 결과를 `SetParametersResult`로 반환하면서 내부 상태를 함께 갱신합니다.
- 역할: ROS2 runtime parameter 변경을 노드 내부 변수에 반영합니다.
- 왜 사용했는가: 실제 로봇 테스트 중 gain과 제한값을 노드를 재시작하지 않고 바꾸기 위해 사용됩니다.
- 제어 영향: roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.
- 내부 동작 흐름:
  - 변경 요청된 parameter 목록을 순회합니다.
  - parameter 이름에 맞는 내부 변수를 갱신합니다.
  - 각도 단위 parameter는 필요한 경우 radian으로 변환합니다.
  - 갱신 결과를 log로 남깁니다.
  - 성공 또는 실패 결과를 `SetParametersResult`로 반환합니다.
- 코드 일부:

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

3장.26 전역 함수.main()

- 위치: `attitude_controller.py:872-885`
- 입력: args
- 출력: 직접적인 return 값보다는 노드 실행과 종료 처리가 핵심 출력입니다.
- 역할: rclpy를 초기화하고 노드를 생성한 뒤 spin을 수행합니다.
- 왜 사용했는가: ROS2 노드 생명주기를 시작하고 종료 처리를 안정적으로 수행하기 위해 사용됩니다.
- 제어 영향: 이 함수는 attitude controller 노드가 실제 ROS graph 안에서 동작하기 시작하는 진입점입니다.
- 내부 동작 흐름:
  - `rclpy.init()`으로 ROS2를 초기화합니다.
  - 노드 객체를 생성합니다.
  - `rclpy.spin()`으로 callback 처리를 시작합니다.
  - 종료 시 노드를 정리하고 `rclpy.shutdown()`을 호출합니다.
- 코드 일부:

```python
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
