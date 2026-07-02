# ROV Control Code Review - 함수별 설명 문서

2장. `allocator_node.py`

이 코드는 최종 `Wrench(Fx, Fy, Fz, Tx, Ty, Tz)`를 8개 thruster 명령으로 바꾸는 control allocation 노드입니다.

이 파일은 제어기가 계산한 6축 wrench를 실제 8개 스러스터 명령으로 바꾸는 마지막 단계입니다. 제어 성능뿐 아니라 실제 로봇 안전에도 직접 연결됩니다.

- 파일: `allocator_node.py`
- 함수 개수: 22
- 주요 역할: 이 코드는 최종 `Wrench(Fx, Fy, Fz, Tx, Ty, Tz)`를 8개 thruster 명령으로 바꾸는 control allocation 노드입니다.

2장.1 전역 함수.normalize()

- 위치: `allocator_node.py:22-29`
- 입력: v
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: 입력 벡터를 단위 벡터로 정규화합니다. 스러스터 방향 벡터가 정확한 힘 방향만 표현하도록 만들기 위해 사용됩니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: 이 함수의 결과는 같은 노드의 다음 계산 단계 또는 다른 제어 노드의 입력으로 사용됩니다. 따라서 값의 단위, 부호, 좌표계가 전체 ROV 움직임에 직접 영향을 줍니다.
- 내부 동작 흐름:
  - 입력 벡터를 `numpy array`로 변환합니다.
  - 노름이 너무 작으면 0 나눗셈을 피하기 위해 원래 벡터를 그대로 사용합니다.
  - 그 외에는 노름으로 나누어 단위 벡터를 반환합니다.
- 코드 일부:

```python
def normalize(v):
    v = np.array(v, dtype=float)
    n = np.linalg.norm(v)
    if n < 1e-9:
        return v
    return v / n
```

2장.2 전역 함수.normalize_group_unit()

- 위치: `allocator_node.py:30-37`
- 입력: v
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: 스러스터 그룹 출력의 최대 절댓값이 1을 넘으면 같은 비율로 전체를 축소합니다. 출력 포화 범위를 유지하면서 방향성은 보존합니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: 스러스터 출력 분배에 직접 영향을 줍니다. 계산 결과는 최종 thruster command의 크기와 방향을 결정합니다.
- 내부 동작 흐름:
  - 입력 그룹 출력을 배열로 변환합니다.
  - 가장 큰 절댓값을 찾습니다.
  - 1보다 크면 전체를 같은 비율로 축소하고, 아니면 그대로 반환합니다.
- 코드 일부:

```python
def normalize_group_unit(v):
    arr = np.array(v, dtype=float)
    max_abs = float(np.max(np.abs(arr)))
    if max_abs > 1.0:
        arr = arr / max_abs
    return arr
```

2장.3 전역 함수.quat_to_rotation_z_row()

- 위치: `allocator_node.py:38-53`
- 입력: x, y, z, w
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: IMU quaternion에서 body z축이 world 좌표계에서 향하는 방향 성분을 계산합니다. 기체가 기울어진 상태의 heave 보상 계산에 사용됩니다.
- 왜 사용했는가: ROV 제어에서는 자세 표현과 좌표계 변환이 계속 필요하므로, 반복되는 수학 연산을 함수로 분리한 것입니다.
- 제어 영향: 이 함수의 결과는 같은 노드의 다음 계산 단계 또는 다른 제어 노드의 입력으로 사용됩니다. 따라서 값의 단위, 부호, 좌표계가 전체 ROV 움직임에 직접 영향을 줍니다.
- 내부 동작 흐름:
  - 입력 quaternion을 정규화합니다.
  - 정규화가 불가능할 정도로 작으면 기본 z축 `(0, 0, 1)`을 반환합니다.
  - 정규화된 quaternion으로 world 기준 body z축 방향 성분을 계산합니다.
- 코드 일부:

```python
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
```

2장.4 전역 함수.quat_to_rpy()

- 위치: `allocator_node.py:54-79`
- 입력: x, y, z, w
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: quaternion 자세 표현을 roll, pitch, yaw 각도로 변환합니다. 사람이 이해하기 쉬운 자세 오차 및 보상 계산에 사용됩니다.
- 왜 사용했는가: ROV 제어에서는 자세 표현과 좌표계 변환이 계속 필요하므로, 반복되는 수학 연산을 함수로 분리한 것입니다.
- 제어 영향: 이 함수의 결과는 같은 노드의 다음 계산 단계 또는 다른 제어 노드의 입력으로 사용됩니다. 따라서 값의 단위, 부호, 좌표계가 전체 ROV 움직임에 직접 영향을 줍니다.
- 내부 동작 흐름:
  - 입력 quaternion을 정규화합니다.
  - roll, pitch, yaw를 순서대로 계산합니다.
  - pitch는 asin 범위를 넘지 않도록 안전하게 처리합니다.
- 코드 일부:

```python
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

2장.5 AllocatorNode.__init__()

- 위치: `allocator_node.py:82-332`
- 입력: self
- 출력: 직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다.
- 역할: ROS2 노드의 파라미터, 상태 변수, subscriber, publisher, timer를 초기화합니다. 해당 제어 노드가 시스템에 연결되는 시작점입니다.
- 왜 사용했는가: 노드가 실행되기 전에 필요한 파라미터, 통신 인터페이스, 상태 변수를 모두 준비해야 하기 때문에 사용됩니다.
- 제어 영향: 이 함수의 결과는 같은 노드의 다음 계산 단계 또는 다른 제어 노드의 입력으로 사용됩니다. 따라서 값의 단위, 부호, 좌표계가 전체 ROV 움직임에 직접 영향을 줍니다.
- 내부 동작 흐름:
  - 노드 이름을 설정합니다.
  - ROS parameter를 선언하고 현재 값을 읽습니다.
  - 제어에 필요한 내부 상태 변수를 초기화합니다.
  - subscriber와 publisher를 생성합니다.
  - parameter callback을 등록하고 초기 설정값을 로그에 남깁니다.
- 코드 일부:

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

2장.6 AllocatorNode.imu_callback()

- 위치: `allocator_node.py:333-340`
- 입력: self, msg
- 출력: 직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다.
- 역할: IMU 메시지를 수신하여 현재 자세, 각속도, 또는 z축 방향 정보를 내부 상태에 저장합니다.
- 왜 사용했는가: ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.
- 제어 영향: 이 함수의 결과는 같은 노드의 다음 계산 단계 또는 다른 제어 노드의 입력으로 사용됩니다. 따라서 값의 단위, 부호, 좌표계가 전체 ROV 움직임에 직접 영향을 줍니다.
- 내부 동작 흐름:
  - ROS2 IMU 메시지를 수신합니다.
  - orientation에서 현재 world z축 방향과 roll/pitch/yaw를 계산합니다.
  - 내부 자세 상태와 `have_imu` 플래그를 갱신합니다.
- 코드 일부:

```python
def imu_callback(self, msg: Imu):
    q = msg.orientation
    self.world_z_from_body = quat_to_rotation_z_row(q.x, q.y, q.z, q.w)
    self.current_roll, self.current_pitch, self.current_yaw = quat_to_rpy(
        q.x, q.y, q.z, q.w
    )
    self.have_imu = True
```

2장.7 AllocatorNode.cmd_attitude_callback()

- 위치: `allocator_node.py:341-346`
- 입력: self, msg
- 출력: 직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다.
- 역할: 외부에서 들어오는 목표 자세 명령을 내부 목표 roll/pitch 값으로 반영합니다.
- 왜 사용했는가: ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.
- 제어 영향: roll, pitch 자세 유지 토크와 allocator 내부 보상 기준에 영향을 줍니다.
- 내부 동작 흐름:
  - ROS2 메시지를 수신합니다.
  - NaN이 아닌 x/y 값을 읽습니다.
  - 목표 roll/pitch 내부 상태를 갱신합니다.
- 코드 일부:

```python
def cmd_attitude_callback(self, msg: Vector3):
    if not math.isnan(float(msg.x)):
        self.target_roll = float(msg.x)
    if not math.isnan(float(msg.y)):
        self.target_pitch = float(msg.y)
```

2장.8 AllocatorNode.cmd_attitude_trim_callback()

- 위치: `allocator_node.py:347-352`
- 입력: self, msg
- 출력: 직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다.
- 역할: trim 형태의 목표 자세를 수신하여 roll/pitch 보정 기준으로 사용합니다.
- 왜 사용했는가: ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.
- 제어 영향: roll, pitch 자세 유지 토크와 allocator 내부 보상 기준에 영향을 줍니다.
- 내부 동작 흐름:
  - ROS2 메시지를 수신합니다.
  - NaN이 아닌 x/y 값을 읽습니다.
  - trim 기준으로 사용할 목표 roll/pitch를 갱신합니다.
- 코드 일부:

```python
def cmd_attitude_trim_callback(self, msg: Vector3):
    if not math.isnan(float(msg.x)):
        self.target_roll = float(msg.x)
    if not math.isnan(float(msg.y)):
        self.target_pitch = float(msg.y)
```

2장.9 AllocatorNode.output_scale_callback()

- 위치: `allocator_node.py:353-355`
- 입력: self, msg
- 출력: 직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다.
- 역할: 전체 스러스터 출력 스케일을 실시간으로 갱신합니다.
- 왜 사용했는가: ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.
- 제어 영향: 최종 thruster command 전체 크기에 직접 영향을 줍니다.
- 내부 동작 흐름:
  - ROS2 메시지를 수신합니다.
  - 입력 값을 0~1 범위로 clamp합니다.
  - 내부 `output_scale` 상태를 갱신합니다.
- 코드 일부:

```python
def output_scale_callback(self, msg: Float64):
    self.output_scale = float(np.clip(float(msg.data), 0.0, 1.0))
```

2장.10 AllocatorNode.joy_speed_scale_callback()

- 위치: `allocator_node.py:356-359`
- 입력: self, msg
- 출력: 직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다.
- 역할: 조이스틱 속도 스케일을 전체 출력 스케일로 연결할지 결정합니다.
- 왜 사용했는가: 조종기 속도 모드와 allocator 출력 크기를 연동하기 위해 사용됩니다.
- 제어 영향: 조이스틱 기반 운용 시 최종 출력 감도와 최대 추력 수준에 직접 영향을 줍니다.
- 내부 동작 흐름:
  - ROS2 메시지를 수신합니다.
  - `use_joy_speed_scale_for_output`가 켜져 있는지 확인합니다.
  - 활성 상태면 `output_scale_callback()`을 재사용해 출력 스케일을 갱신합니다.
- 코드 일부:

```python
def joy_speed_scale_callback(self, msg: Float64):
    if self.use_joy_speed_scale_for_output:
        self.output_scale_callback(msg)
```

2장.11 AllocatorNode.level_horizontal_heave_compensation()

- 위치: `allocator_node.py:360-374`
- 입력: self, fx, fy
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: 기체가 roll/pitch로 기울어진 상태에서 수평 힘이 수직 방향으로 새는 효과를 보상하기 위한 heave 값을 계산합니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: 수평 이동 중 기체가 의도치 않게 뜨거나 가라앉는 현상에 직접 영향을 줍니다.
- 내부 동작 흐름:
  - 보상 기능이 켜져 있고 IMU가 유효한지 확인합니다.
  - 현재 body z축의 world 성분을 읽습니다.
  - 기울기 때문에 생기는 heave leak를 계산하고 제한값 안으로 clamp합니다.
- 코드 일부:

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
```

2장.12 AllocatorNode.attitude_priority_horizontal_scale()

- 위치: `allocator_node.py:375-391`
- 입력: self, tx_scaled, ty_scaled
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: roll/pitch 토크 요구가 큰 상황에서 수평 이동 명령을 줄여 자세 제어 우선권을 확보합니다.
- 왜 사용했는가: 수평 이동과 자세 복원이 동시에 포화될 때, 어떤 축을 우선할지 분명히 하기 위해 사용됩니다.
- 제어 영향: roll, pitch 복원이 급한 상황에서 surge/sway 응답이 얼마나 희생될지 결정합니다.
- 내부 동작 흐름:
  - slowdown 기능 활성 여부를 확인합니다.
  - 현재 자세 토크 요구 크기를 계산합니다.
  - start/full 구간에 따라 1.0에서 `min_scale`까지 scale을 계산합니다.
- 코드 일부:

```python
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
```

2장.13 AllocatorNode.surge_pitch_moment_compensation()

- 위치: `allocator_node.py:392-420`
- 입력: self, fx
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: 전진/후진 힘이 pitch 모멘트를 만드는 상황을 feed-forward 방식으로 보상합니다.
- 왜 사용했는가: 기체 구조상 surge 추력이 nose-up 또는 nose-down 성향을 만들 수 있어, 이를 allocator 단계에서 미리 상쇄하기 위해 사용됩니다.
- 제어 영향: 전진 시 pitch 흔들림과 수직 스러스터 부담 분배에 직접 영향을 줍니다.
- 내부 동작 흐름:
  - 보상 기능 활성 여부와 최소 surge 조건을 확인합니다.
  - 필요하면 목표 pitch 크기에 따른 gating을 적용합니다.
  - 설정 gain과 한계값을 사용해 추가 pitch 보상량을 계산합니다.
- 코드 일부:

```python
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
```

2장.14 AllocatorNode.imu_pitch_hold_compensation()

- 위치: `allocator_node.py:421-435`
- 입력: self, fx
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: 현재 pitch와 목표 pitch의 차이를 이용해 surge 중 pitch 유지 보상량을 계산합니다.
- 왜 사용했는가: 전진 중 실제 pitch가 목표에서 벗어날 때 상위 controller 이전에 allocator 차원에서 추가 보정을 넣기 위해 사용됩니다.
- 제어 영향: surge 상황에서 pitch hold가 얼마나 단단하게 유지될지에 영향을 줍니다.
- 내부 동작 흐름:
  - 보상 기능과 IMU 유효성을 확인합니다.
  - 최소 surge 조건과 pitch deadband를 검사합니다.
  - pitch error에 gain을 곱하고 한계값 안으로 clamp합니다.
- 코드 일부:

```python
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

2장.15 AllocatorNode.init_matrices()

- 위치: `allocator_node.py:436-499`
- 입력: self
- 출력: 내부 상태 갱신이 중심이며, 필요 시 계산 결과를 return합니다.
- 역할: 스러스터 위치와 방향으로부터 TAM, 수평 allocation 행렬, 수직 allocation 행렬, pseudo-inverse를 생성합니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: 스러스터 출력 분배에 직접 영향을 줍니다. 계산 결과는 최종 thruster command의 크기와 방향을 결정합니다.
- 내부 동작 흐름:
  - 각 스러스터의 위치와 방향을 정의합니다.
  - 전체 TAM과 수평/수직 그룹 행렬을 만듭니다.
  - pseudo-inverse를 계산해 이후 allocation 단계에서 재사용할 수 있게 저장합니다.
- 코드 일부:

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

2장.16 AllocatorNode.apply_deadband()

- 위치: `allocator_node.py:500-504`
- 입력: self, arr
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: 작은 출력값을 0으로 만들어 스러스터 미세 떨림이나 불필요한 명령을 줄입니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: 아주 작은 thruster command를 제거하여 actuator chatter와 불필요한 소비전력을 줄입니다.
- 내부 동작 흐름:
  - 입력을 배열로 변환합니다.
  - `output_deadband`보다 작은 값을 0으로 만듭니다.
  - deadband가 적용된 결과를 반환합니다.
- 코드 일부:

```python
def apply_deadband(self, arr):
    out = np.array(arr, dtype=float)
    out[np.abs(out) < self.output_deadband] = 0.0
    return out
```

2장.17 AllocatorNode.add_component_with_headroom()

- 위치: `allocator_node.py:505-526`
- 입력: self, base, addition
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: 기존 출력에 추가 제어 성분을 더할 때 -1~1 범위를 넘지 않도록 남은 headroom만큼만 추가합니다.
- 왜 사용했는가: 여러 제어 성분을 단순 합산하면 saturation으로 우선순위가 무너질 수 있어, 남은 출력 공간을 계산해 안전하게 합치기 위해 사용됩니다.
- 제어 영향: 어떤 제어 성분이 saturation 상황에서 살아남는지에 직접 영향을 줍니다.
- 내부 동작 흐름:
  - 기존 출력과 추가 성분을 배열로 변환합니다.
  - 추가 성분이 매우 작으면 기존 출력을 그대로 사용합니다.
  - 합산 결과가 한계를 넘으면 scale을 줄여 headroom 안에서만 더합니다.
- 코드 일부:

```python
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
```

2장.18 AllocatorNode.allocate_priority_components()

- 위치: `allocator_node.py:527-532`
- 입력: self, components
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: 여러 제어 성분을 우선순위 순서대로 합성합니다. 먼저 들어온 성분이 출력 공간을 우선 사용합니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: heave, roll, pitch 같은 성분 중 어떤 항이 saturation 시 우선권을 갖는지 결정합니다.
- 내부 동작 흐름:
  - 초기 출력을 0으로 시작합니다.
  - components를 순서대로 순회합니다.
  - `add_component_with_headroom()`으로 우선순위를 유지하며 합성합니다.
- 코드 일부:

```python
def allocate_priority_components(self, components):
    out = np.zeros(4, dtype=float)
    for component in components:
        out = self.add_component_with_headroom(out, component)
    return out
```

2장.19 AllocatorNode.apply_slew_rate()

- 위치: `allocator_node.py:533-555`
- 입력: self, target
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: 이전 출력과 목표 출력 사이의 변화량을 시간 기준으로 제한합니다. 스러스터 명령의 급격한 변화를 줄입니다.
- 왜 사용했는가: 스러스터와 전원 계통에 갑작스러운 명령 변화가 가해지는 것을 줄이기 위해 사용됩니다.
- 제어 영향: 최종 thruster command의 응답 속도와 부드러움에 직접 영향을 줍니다.
- 내부 동작 흐름:
  - 현재 시각과 이전 시각으로 `dt`를 계산합니다.
  - 목표 출력과 이전 출력 차이를 구합니다.
  - 축별 최대 변화량을 제한한 뒤 새 출력을 저장하고 반환합니다.
- 코드 일부:

```python
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

2장.20 AllocatorNode.callback()

- 위치: `allocator_node.py:556-683`
- 입력: self, msg
- 출력: 내부 상태 갱신이 중심이며, 필요 시 계산 결과를 return합니다.
- 역할: 최종 Wrench 명령을 받아 수평/수직 allocation, 보상, normalization, scaling, slew-rate를 거쳐 thruster command를 발행합니다.
- 왜 사용했는가: ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.
- 제어 영향: 스러스터 출력 분배에 직접 영향을 줍니다. 계산 결과는 최종 thruster command의 크기와 방향을 결정합니다.
- 내부 동작 흐름:
  - `/rov/wrench_cmd`에서 force와 torque를 읽습니다.
  - surge, sway, heave, roll, pitch, yaw 성분을 분리합니다.
  - 기울어진 자세에서 수평 이동 시 필요한 heave 보상량을 계산합니다.
  - 수평 스러스터 그룹과 수직 스러스터 그룹으로 나누어 pseudo-inverse allocation을 수행합니다.
  - 토크 우선 또는 heave 우선 정책에 따라 vertical 출력을 합성합니다.
  - 출력 normalization, sign, output_scale, max_output, slew-rate, deadband를 적용합니다.
  - `Float64MultiArray`로 8개 thruster command를 발행합니다.
- 코드 일부:

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

2장.21 AllocatorNode.on_parameter_update()

- 위치: `allocator_node.py:684-788`
- 입력: self, params
- 출력: 파라미터 갱신 결과를 `SetParametersResult`로 반환하면서 내부 상태를 함께 갱신합니다.
- 역할: ROS2 runtime parameter 변경을 노드 내부 변수에 반영합니다.
- 왜 사용했는가: 실제 로봇 테스트 중 gain과 제한값을 노드를 재시작하지 않고 바꾸기 위해 사용됩니다.
- 제어 영향: 이 함수의 결과는 같은 노드의 다음 계산 단계 또는 다른 제어 노드의 입력으로 사용됩니다. 따라서 값의 단위, 부호, 좌표계가 전체 ROV 움직임에 직접 영향을 줍니다.
- 내부 동작 흐름:
  - 변경 요청된 parameter 목록을 순회합니다.
  - parameter 이름에 맞는 내부 변수를 갱신합니다.
  - 각도 단위 parameter는 필요한 경우 radian으로 변환합니다.
  - 갱신 결과를 log로 남기고 `SetParametersResult`를 반환합니다.
- 코드 일부:

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

2장.22 전역 함수.main()

- 위치: `allocator_node.py:789-802`
- 입력: args
- 출력: 직접적인 return 값보다는 노드 실행과 종료 처리가 핵심 출력입니다.
- 역할: rclpy를 초기화하고 노드를 생성한 뒤 spin을 수행합니다.
- 왜 사용했는가: ROS2 노드 생명주기를 시작하고 종료 처리를 안정적으로 수행하기 위해 사용됩니다.
- 제어 영향: 이 함수는 allocator 노드가 실제 ROS graph 안에서 동작하기 시작하는 진입점입니다.
- 내부 동작 흐름:
  - `rclpy.init()`으로 ROS2를 초기화합니다.
  - 노드 객체를 생성합니다.
  - `rclpy.spin()`으로 callback 처리를 시작합니다.
  - 종료 시 노드를 정리하고 `rclpy.shutdown()`을 호출합니다.
- 코드 일부:

```python
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
