# Allocator Review

대상 파일: `code_review/code/allocator_node.py`

## 역할
이 코드는 최종 `Wrench(Fx, Fy, Fz, Tx, Ty, Tz)`를 8개 thruster 명령으로 바꾸는 control allocation 노드입니다.

## 설계 해석
설계의 핵심은 `수평 4개(Fx, Fy, Tz)`와 `수직 4개(Fz, Tx, Ty)`를 분리해서 pseudo-inverse와 priority logic으로 다루는 것입니다.

## 리뷰 초점
리뷰 포인트는 `행렬이 실제 기체 배치를 어떻게 표현하는지`, `수평/수직 allocator가 어떤 철학으로 분리되는지`, `compensation 항이 왜 필요한지`, `최종 shaping과 slew-rate가 actuator에 어떤 영향을 주는지`입니다.

## 런타임 동작 해설
런타임에서는 `callback()`가 들어온 wrench를 먼저 내부 축 부호와 gain으로 해석하고, 수평과 수직 그룹으로 나눠 allocation을 수행합니다. 그 뒤 compensation, priority allocation, normalization, deadband, slew-rate를 차례로 적용해 실제 thruster array로 publish합니다. 즉 이 모듈은 '마지막 수학 + 마지막 actuator shaping' 계층입니다.

## 핵심 파라미터
- `horizontal_output_gain`, `vertical_output_gain`, `yaw_output_gain`: 수평/수직 force와 yaw torque가 thruster 출력으로 얼마나 강하게 반영될지 정합니다.
- `heave_gain`: heave 요구를 vertical thruster 출력으로 키우는 기본 gain입니다.
- `pitch_torque_gain`, `rear_vertical_bias`: pitch recovery 성향과 후방 수직 thruster 바이어스를 조절합니다.
- `torque_first_allocation`: 수직 그룹에서 자세 토크를 먼저 만족시키고 남는 헤드룸에 heave를 넣을지 결정합니다.
- `level_horizontal_compensation_*`: 기체가 기울어진 상태에서 수평 이동이 heave 성분을 만들 때 이를 보정하는 파라미터 묶음입니다.
- `attitude_priority_horizontal_slowdown_*`: 자세 토크 요구가 커질수록 horizontal output을 얼마나 줄일지 정합니다.
- `surge_pitch_moment_*`: surge thrust가 만드는 pitch moment를 보상할지와 그 강도를 정합니다.
- `imu_pitch_hold_*`: 현재 pitch와 target pitch 차이를 allocator 차원에서 추가 보상할지 정합니다.
- `slew_rate`: thruster 출력이 한 번에 너무 급하게 바뀌지 않도록 제한합니다.
- `max_output`, `output_scale`, `output_deadband`: 최종 thruster 명령 범위, 전체 출력 크기, deadband 제거 수준을 정합니다.

## 함수 맵
- `normalize()`
- `normalize_group_unit()`
- `quat_to_rotation_z_row()`
- `quat_to_rpy()`
- `__init__()`
- `imu_callback()`
- `cmd_attitude_callback()`
- `cmd_attitude_trim_callback()`
- `output_scale_callback()`
- `joy_speed_scale_callback()`
- `level_horizontal_heave_compensation()`
- `attitude_priority_horizontal_scale()`
- `surge_pitch_moment_compensation()`
- `imu_pitch_hold_compensation()`
- `init_matrices()`
- `apply_deadband()`
- `add_component_with_headroom()`
- `allocate_priority_components()`
- `apply_slew_rate()`
- `callback()`
- `on_parameter_update()`
- `main()`

## 함수 리뷰

### 기본 유틸리티

**의미**

행렬 생성과 IMU 보상 계산에 필요한 벡터/자세 유틸리티입니다.

**영향**

allocation 행렬과 IMU 기반 보상 항의 수학적 기반을 제공합니다.

**리뷰 메모**

작은 함수지만 allocator가 단순 mixer가 아니라 IMU-aware allocation이라는 점을 보여줍니다.

**상세 해설**

allocator에서 이 유틸리티들은 단순 계산 편의 함수가 아니라, 뒤쪽 보상 로직과 allocation geometry가 모두 기대고 있는 수학 기반입니다. `normalize_group_unit()`은 한 그룹 출력이 saturation을 넘지 않도록 정규화하고, `quat_to_rotation_z_row()`는 현재 기체의 기울기 정보를 보상 항 계산에 사용할 수 있게 해 줍니다.

즉 allocator가 고정된 행렬 곱셈기를 넘어, 현재 자세를 아는 분배기처럼 동작하게 만드는 첫 계층입니다.

```python
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
```

### `__init__()`

**의미**

thruster sign, gain, compensation, priority, output shaping, runtime tuning 관련 모든 파라미터를 선언합니다. 즉 allocator가 단순 행렬 연산 노드인지, 실제 기체 거동까지 반영하는 smart output layer인지가 여기서 결정됩니다.

**영향**

allocator가 단순 `TAM * u = τ` 계산기가 아니라 실제 운용형 shaping node라는 점이 여기서 드러납니다.

**리뷰 메모**

파라미터가 많지만 역할이 분명합니다. 다만 `fx = -msg.force.x`처럼 본체 쪽 부호 해석이 하드코딩되어 있어서 문서화가 꼭 필요합니다.

**상세 해설**

지금 보여주신 화면에서 가장 빈약하게 보였던 부분이 바로 이 함수입니다. 실제로는 이 함수가 allocator 전체 성격을 거의 다 설명해 줍니다. 첫 번째 덩어리는 `wrench_cmd_topic`, `thruster_cmd_topic`, `imu_topic` 같은 입출력 연결입니다. 두 번째 덩어리는 `heave_gain`, `horizontal_output_gain`, `yaw_output_gain`, `vertical_output_gain`처럼 force/torque 축별 gain입니다. 세 번째 덩어리는 `torque_first_allocation`, `rear_vertical_bias`, `pitch_torque_gain`처럼 allocation 우선순위와 기체 성향을 반영하는 파라미터입니다.

그리고 이 코드의 개성을 가장 잘 보여주는 것은 뒤쪽 compensation 파라미터들입니다. `level_horizontal_compensation_*`, `attitude_priority_horizontal_slowdown_*`, `surge_pitch_moment_*`, `imu_pitch_hold_*`는 단순한 `TAM pseudo-inverse`를 넘어서 실제 기체가 기울고, 전진하고, 자세를 유지할 때 어떤 출력을 내야 하는지를 allocator 수준에서 조정합니다. 즉 이 함수는 allocator의 철학서에 가깝습니다.

**이 함수와 관련된 파라미터**

- `wrench_cmd_topic`, `thruster_cmd_topic`, `imu_topic`: allocator가 무엇을 입력으로 받고 무엇을 출력할지 정합니다.
- `heave_gain`, `horizontal_output_gain`, `vertical_output_gain`, `yaw_output_gain`: 각 축 요구를 실제 thruster 명령으로 얼마나 크게 반영할지 정하는 기본 gain입니다.
- `torque_first_allocation`: 수직 그룹에서 heave보다 자세 토크를 먼저 만족시킬지 결정합니다.
- `rear_vertical_bias`, `pitch_torque_gain`: 수직 thruster 분배 성향과 pitch 복원 특성을 조정합니다.
- `level_horizontal_compensation_*`: 기체가 기울어진 상태에서 수평 이동이 heave로 새는 문제를 allocator 단계에서 보정합니다.
- `attitude_priority_horizontal_slowdown_*`: 자세 토크 요구가 크면 horizontal 출력을 얼마나 줄일지 정합니다.
- `surge_pitch_moment_*`: surge thrust가 만드는 pitch moment를 allocator 차원에서 보상합니다.
- `imu_pitch_hold_*`: 현재 pitch와 target pitch 차이를 보고 추가적인 hold 보상을 넣습니다.
- `slew_rate`, `max_output`, `output_scale`, `output_deadband`: 최종 actuator-friendly output shaping과 saturation 성격을 결정합니다.

```python
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
```

### IMU / Attitude 입력 함수

**의미**

allocator가 현재 자세, 목표 trim, 출력 scale 상태를 받아 compensation과 output shaping에 반영하는 입력 계층입니다.

**영향**

이 함수들이 있어 allocator는 단순 분배가 아니라 현재 자세와 운용 상태를 고려하는 smart mixer가 됩니다.

**리뷰 메모**

상위 controller와 allocator가 느슨하지만 의미 있게 연결되어 있습니다.

**상세 해설**

이 계층은 allocator가 독립적인 하위 노드이면서도 상위 자세 제어 의도와 조종 상태를 읽을 수 있게 해 줍니다. `imu_callback()`은 현재 pitch/roll을 보상 함수들에 제공하고, `cmd_attitude_callback()` 및 `cmd_attitude_trim_callback()`은 목표 자세 기준을 allocator 보상에 연결합니다. `output_scale_callback()`과 `joy_speed_scale_callback()`은 전체 추진 크기를 런타임에 조절하는 운영 인터페이스입니다.

즉 allocator는 단순 수동 mixer가 아니라, 상위 제어 의도와 현재 기체 자세를 함께 반영하는 output shaping layer입니다.

**이 함수와 관련된 파라미터**

- `cmd_attitude_topic`, `cmd_attitude_trim_topic`: 목표 pitch/trim 기준을 allocator 보상에 전달합니다.
- `output_scale_topic`, `joy_speed_scale_topic`, `use_joy_speed_scale_for_output`: 최종 thruster 출력의 전체 크기를 외부에서 조정합니다.

```python
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
```

### Compensation 함수들

**의미**

수평 이동으로 인한 heave 누수, attitude demand가 클 때 horizontal 약화, surge에 따른 pitch moment, IMU 기반 pitch hold 보상 등 실제 기체 거동을 보정합니다.

**영향**

이 함수들이 켜지면 allocator는 단순 분배기에서 벗어나, 기체 자세와 운동 모드에 따라 더 실용적인 출력을 냅니다.

**리뷰 메모**

이 부분이 가장 실험적이면서도 고급스럽습니다. 동시에 gain 의존성이 큰 영역이므로 기능별 로그와 문서가 꼭 있어야 합니다.

**상세 해설**

이 묶음은 allocator를 단순한 TAM 분배기에서 실제 운용형 shaping node로 바꿔 주는 핵심입니다. `level_horizontal_heave_compensation()`은 기체가 기울어진 상태에서 수평 추력이 수직 성분을 만드는 문제를 완화하고, `attitude_priority_horizontal_scale()`은 자세 토크 요구가 클 때 수평 이동을 일부 희생하게 만듭니다. `surge_pitch_moment_compensation()`과 `imu_pitch_hold_compensation()`은 전진 thrust와 현재 pitch 상태를 반영해 추가적인 수직 보상을 생성합니다.

즉 이 계층은 '요구 wrench'와 '실제 기체가 체감하는 거동' 사이의 간극을 메우는 경험적 보상층입니다. 튜닝 난이도는 있지만, 제대로 맞으면 상위 제어기가 덜 힘들어집니다.

**이 함수와 관련된 파라미터**

- `level_horizontal_compensation_*`: 수평 추진에 따라 생기는 heave leak 보상 정책입니다.
- `attitude_priority_horizontal_slowdown_*`: 자세 토크가 커질수록 horizontal 출력을 얼마나 줄일지 정합니다.
- `surge_pitch_moment_*`: 전진 추력이 만드는 pitch moment 보상 강도와 활성 조건입니다.
- `imu_pitch_hold_*`: 현재/목표 pitch 차이에 따라 추가 보상을 넣는 allocator 내부 hold 파라미터입니다.

```python
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
```

### `init_matrices()`

**의미**

thruster 위치/방향으로 TAM, 수평 H 행렬, 수직 V 행렬, pseudo-inverse를 구성합니다.

**영향**

기체 배치가 이 함수에 담깁니다. 행렬이 틀리면 heave만 줘도 pitch가 생기고, yaw만 줘도 sway가 생깁니다.

**리뷰 메모**

allocator 해석의 핵심입니다. 실제 모델과 이 함수의 thruster geometry가 맞는지 항상 함께 검증해야 합니다.

**상세 해설**

이 함수는 기체의 thruster 위치와 방향을 수학 모델로 고정하는 구간입니다. 수평 4기와 수직 4기의 각 레버암과 thrust 방향이 여기서 TAM과 pseudo-inverse 행렬로 변환됩니다.

따라서 allocator 문제의 상당수는 사실 이 함수에서 시작됩니다. yaw만 줬는데 sway가 섞이거나 heave만 줬는데 pitch가 생긴다면, 게인보다 먼저 이 geometry와 부호 정의가 실제 하드웨어와 일치하는지 검증해야 합니다.

**이 함수와 관련된 파라미터**

- `rear_vertical_bias`, `pitch_torque_gain`: 수직 thruster 분배 행렬이 실제로 어떤 성향을 띨지에 간접적으로 영향을 줍니다.

```python
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
```

### Allocation / Shaping 보조 함수

**의미**

출력 deadband 제거, 남은 헤드룸 안에서 성분 추가, priority allocation, slew-rate 제한을 담당합니다.

**영향**

수치적으로는 같은 wrench라도 actuator 친화성은 크게 달라집니다. 이 함수들이 saturation 시 거동과 응답 속도를 정합니다.

**리뷰 메모**

현재 allocator가 heuristic allocator라는 점이 가장 잘 드러나는 구간입니다. 이 계층 덕분에 practical하지만, 최적화 기반 allocator와는 성격이 다릅니다.

**상세 해설**

이 함수들은 이상적인 연속 wrench를 실제 actuator가 낼 수 있는 명령열로 다듬는 후처리 계층입니다. `add_component_with_headroom()`과 `allocate_priority_components()`는 saturation 가까이에서 어떤 성분을 먼저 살릴지 결정하고, `apply_deadband()`는 너무 작은 명령을 없애 chatter를 줄이며, `apply_slew_rate()`는 한 번에 급격히 바뀌는 thrust를 막습니다.

즉 allocator 품질은 행렬 해석만으로 끝나지 않습니다. 같은 wrench라도 이런 shaping 계층이 다르면 실제 모터 체감, 발열, 응답성, 자세 안정성이 크게 달라집니다.

**이 함수와 관련된 파라미터**

- `slew_rate`: 출력이 시간적으로 얼마나 빨리 변할 수 있는지 제한합니다.
- `output_deadband`: 아주 작은 thrust 명령을 제거합니다.
- `max_output`, `output_scale`: 최종 thrust의 절대 크기와 전체 배율을 정합니다.

```python
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
```

### `callback()`

**의미**

최종 wrench를 받아 부호 해석, scaling, horizontal/vertical allocation, compensation, shaping, saturation, publish까지 한 번에 수행합니다.

**영향**

이 함수가 곧 thruster command의 최종 형태를 결정합니다. 어떤 축 요구가 우선되는지, saturation에서 무엇을 희생하는지가 여기서 결정됩니다.

**리뷰 메모**

코드의 의도는 분명합니다. 다만 `Fx` 하드코딩 반전과 heuristic saturation은 읽는 사람에게 반드시 설명이 필요합니다.

**상세 해설**

이 함수는 allocator의 본체입니다. 먼저 입력 wrench를 읽고 내부 좌표계 기준으로 `fx`, `fy`, `fz`, `tx`, `ty`, `tz`를 해석합니다. 여기서 `fx = -msg.force.x`처럼 축 부호를 내부 convention에 맞게 뒤집는 부분이 있기 때문에, 이 함수는 좌표계 해석의 실제 출발점이기도 합니다.

그 다음 수평 성분(`Fx`, `Fy`, `Tz`)과 수직 성분(`Fz`, `Tx`, `Ty`)을 분리해서 allocation을 수행하고, 보상 항들을 추가한 뒤 priority allocation, normalization, deadband, slew-rate를 순서대로 적용합니다. 즉 이 함수는 단순 수학 공식이 아니라 '요구 wrench를 실제 thruster array가 낼 수 있는 형태로 번역하는 전체 파이프라인'입니다.

**이 함수와 관련된 파라미터**

- `horizontal_output_gain`, `vertical_output_gain`, `yaw_output_gain`, `heave_gain`: 입력 wrench를 thruster 명령 크기로 스케일링하는 데 직접 들어갑니다.
- `torque_first_allocation`: 수직 allocation에서 heave와 자세 토크 중 무엇을 먼저 보장할지 결정합니다.
- `level_horizontal_compensation_*`, `surge_pitch_moment_*`, `imu_pitch_hold_*`: allocation 중간에 추가되는 보상 항의 크기와 활성 조건을 정합니다.
- `slew_rate`, `max_output`, `output_scale`, `output_deadband`: 최종 thruster 출력을 다듬는 마지막 shaping 파라미터입니다.

```python
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
```

### `on_parameter_update()`

**의미**

allocation gain, compensation, slew, output scale을 런타임에 갱신합니다.

**영향**

allocator tuning을 빠르게 반복할 수 있게 해줍니다.

**리뷰 메모**

실험 속도를 크게 올리는 좋은 구조입니다. 이번 리뷰 문서에서 함수-파라미터 연결을 설명하는 이유도 이 함수 때문입니다.

**상세 해설**

allocator는 실제로 현장 튜닝 비중이 매우 높은 모듈입니다. thrust geometry는 고정되어 있어도, gain과 compensation만으로도 체감 조종감이 크게 달라집니다. 이 함수는 그 조정을 런타임에 가능하게 만들어 테스트 반복 속도를 높여 줍니다.

대신 문서가 부족하면 위험합니다. 어떤 파라미터가 allocation 행렬 이전에 적용되는지, 어떤 것은 compensation이고 어떤 것은 최종 shaping인지 구분되지 않으면 잘못된 축을 튜닝할 수 있기 때문입니다.

```python
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
```

## 전체 코드

```python
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
```
