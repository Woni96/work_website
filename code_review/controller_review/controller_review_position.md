# ROV Control Code Review - 함수별 설명 문서

5장. `position_controller.py`

DVL 위치/속도와 IMU 자세를 이용해 XY 위치 유지 힘을 생성하는 위치 제어 노드

이 파일은 DVL 위치 또는 속도를 이용해 XY 방향 위치 유지 힘을 만듭니다. 현재 위치를 hold target으로 잡고 위치 오차에 따라 수평 force를 만듭니다.

- 파일: `position_controller.py`
- 함수 개수: 22
- 주요 역할: DVL 위치/속도와 IMU 자세를 이용해 XY 위치 유지 힘을 생성하는 위치 제어 노드

5장.1 전역 함수.clamp()

- 위치: `position_controller.py:16-19`
- 입력: value, lo, hi
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: 값을 지정된 최소/최대 범위 안으로 제한합니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: DVL 기반 위치 추정과 XY position hold 힘에 영향을 준다. 좌표 변환 결과는 전진/좌우 힘 방향을 결정합니다.
- 내부 동작 흐름:
  - 입력값과 최소/최대 한계를 받습니다.
  - 최솟값보다 작으면 최솟값으로 제한합니다.
  - 최댓값보다 크면 최댓값으로 제한하고, 범위 안이면 그대로 반환합니다.
- 코드 일부:

```python
def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))
```

5장.2 전역 함수.quat_normalize()

- 위치: `position_controller.py:20-26`
- 입력: x, y, z, w
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: quaternion을 단위 quaternion으로 정규화합니다.
- 왜 사용했는가: ROV 제어에서는 자세 표현과 좌표계 변환이 계속 필요하므로, 반복되는 수학 연산을 함수로 분리한 것입니다.
- 제어 영향: DVL 기반 위치 추정과 XY position hold 힘에 영향을 준다. 좌표 변환 결과는 전진/좌우 힘 방향을 결정합니다.
- 내부 동작 흐름:
  - 입력 quaternion 노름을 계산합니다.
  - 노름이 너무 작으면 기본 단위 quaternion을 반환합니다.
  - 정규화된 quaternion 성분을 반환합니다.
- 코드 일부:

```python
def quat_normalize(x: float, y: float, z: float, w: float):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)
```

5장.3 전역 함수.quat_multiply()

- 위치: `position_controller.py:27-37`
- 입력: a, b
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: 두 quaternion의 곱을 계산합니다. DVL 벡터 회전 변환에 사용됩니다.
- 왜 사용했는가: ROV 제어에서는 자세 표현과 좌표계 변환이 계속 필요하므로, 반복되는 수학 연산을 함수로 분리한 것입니다.
- 제어 영향: DVL 기반 위치 추정과 XY position hold 힘에 영향을 준다. 좌표 변환 결과는 전진/좌우 힘 방향을 결정합니다.
- 내부 동작 흐름:
  - 두 quaternion 성분을 각각 분리합니다.
  - Hamilton product 공식을 적용합니다.
  - 곱셈 결과 quaternion을 반환합니다.
- 코드 일부:

```python
def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )
```

5장.4 전역 함수.quat_conjugate()

- 위치: `position_controller.py:38-42`
- 입력: q
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: quaternion 역회전에 필요한 켤레를 계산합니다.
- 왜 사용했는가: ROV 제어에서는 자세 표현과 좌표계 변환이 계속 필요하므로, 반복되는 수학 연산을 함수로 분리한 것입니다.
- 제어 영향: DVL 기반 위치 추정과 XY position hold 힘에 영향을 준다. 좌표 변환 결과는 전진/좌우 힘 방향을 결정합니다.
- 내부 동작 흐름:
  - 입력 quaternion 성분을 분리합니다.
  - 벡터부 부호를 반전합니다.
  - 스칼라부는 유지한 채 켤레 quaternion을 반환합니다.
- 코드 일부:

```python
def quat_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)
```

5장.5 전역 함수.quat_from_rpy()

- 위치: `position_controller.py:43-57`
- 입력: roll, pitch, yaw
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: DVL 장착 각도를 quaternion으로 변환합니다.
- 왜 사용했는가: ROV 제어에서는 자세 표현과 좌표계 변환이 계속 필요하므로, 반복되는 수학 연산을 함수로 분리한 것입니다.
- 제어 영향: DVL 기반 위치 추정과 XY position hold 힘에 영향을 준다. 좌표 변환 결과는 전진/좌우 힘 방향을 결정합니다.
- 내부 동작 흐름:
  - roll, pitch, yaw의 half-angle 삼각함수를 계산합니다.
  - quaternion 성분을 조합합니다.
  - 정규화된 quaternion을 반환합니다.
- 코드 일부:

```python
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
```

5장.6 전역 함수.quat_rotate_vector()

- 위치: `position_controller.py:58-63`
- 입력: q, v
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: quaternion으로 3차원 벡터를 회전시킵니다.
- 왜 사용했는가: ROV 제어에서는 자세 표현과 좌표계 변환이 계속 필요하므로, 반복되는 수학 연산을 함수로 분리한 것입니다.
- 제어 영향: DVL 기반 위치 추정과 XY position hold 힘에 영향을 준다. 좌표 변환 결과는 전진/좌우 힘 방향을 결정합니다.
- 내부 동작 흐름:
  - 벡터를 quaternion 형태로 확장합니다.
  - 회전 quaternion과 켤레 quaternion으로 양쪽에서 곱합니다.
  - 회전된 벡터 성분을 반환합니다.
- 코드 일부:

```python
def quat_rotate_vector(q, v):
    qv = (v[0], v[1], v[2], 0.0)
    rotated = quat_multiply(quat_multiply(q, qv), quat_conjugate(q))
    return (rotated[0], rotated[1], rotated[2])
```

5장.7 PositionController.__init__()

- 위치: `position_controller.py:66-216`
- 입력: self
- 출력: 직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다.
- 역할: ROS2 노드의 파라미터, 상태 변수, subscriber, publisher, timer를 초기화합니다. 해당 제어 노드가 시스템에 연결되는 시작점입니다.
- 왜 사용했는가: 노드가 실행되기 전에 필요한 파라미터, 통신 인터페이스, 상태 변수를 모두 준비해야 하기 때문에 사용됩니다.
- 제어 영향: DVL 기반 위치 추정과 XY position hold 힘에 영향을 준다. 좌표 변환 결과는 전진/좌우 힘 방향을 결정합니다.
- 내부 동작 흐름:
  - 노드 이름을 설정합니다.
  - ROS parameter를 선언하고 현재 값을 읽습니다.
  - 제어에 필요한 내부 상태 변수를 초기화합니다.
  - subscriber와 publisher를 생성합니다.
  - parameter callback을 등록합니다.
  - 초기 설정값을 log로 출력합니다.
- 코드 일부:

```python
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
```

5장.8 PositionController._publish_zero_force()

- 위치: `position_controller.py:217-219`
- 입력: self
- 출력: 내부 상태 갱신이 중심이며, 필요 시 계산 결과를 return합니다.
- 역할: position controller 출력 Wrench를 0으로 발행합니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: DVL 기반 위치 추정과 XY position hold 힘에 영향을 준다. 좌표 변환 결과는 전진/좌우 힘 방향을 결정합니다.
- 내부 동작 흐름:
  - 빈 `Wrench()` 메시지를 준비합니다.
  - 출력 publisher를 통해 0 force를 발행합니다.
  - 자동 hold 출력을 즉시 정지시키는 역할을 수행합니다.
- 코드 일부:

```python
def _publish_zero_force(self):
    self.pub_force.publish(Wrench())
```

5장.9 PositionController._has_valid_position_reference()

- 위치: `position_controller.py:220-227`
- 입력: self, now
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: IMU와 DVL 기준 정보가 유효 시간 안에 있는지 확인합니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: DVL 기반 위치 추정과 XY position hold 힘에 영향을 준다. 좌표 변환 결과는 전진/좌우 힘 방향을 결정합니다.
- 내부 동작 흐름:
  - IMU가 유효한지 확인합니다.
  - 마지막 유효 DVL 시각이 있는지 확인합니다.
  - 현재 시각과의 차이가 `valid_timeout_sec` 안인지 판단합니다.
- 코드 일부:

```python
def _has_valid_position_reference(self, now) -> bool:
    if not self.have_imu:
        return False
    if self.last_valid_dvl_time is None:
        return False
    age = (now - self.last_valid_dvl_time).nanoseconds * 1e-9
    return age <= self.valid_timeout_sec
```

5장.10 PositionController._set_control_enabled()

- 위치: `position_controller.py:228-239`
- 입력: self, enabled
- 출력: 내부 상태 갱신이 중심이며, 필요 시 계산 결과를 return합니다.
- 역할: 제어 enable 상태 변경 시 목표값, 적분항, 출력 상태를 초기화하거나 0 출력합니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: DVL 기반 위치 추정과 XY position hold 힘에 영향을 준다. 좌표 변환 결과는 전진/좌우 힘 방향을 결정합니다.
- 내부 동작 흐름:
  - 이전 enable 상태와 새 상태를 비교합니다.
  - enable 전이 시 유효한 위치 참조가 있으면 현재 위치를 target으로 캡처합니다.
  - disable 전이 시 0 force를 발행합니다.
- 코드 일부:

```python
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
```

5장.11 PositionController._capture_current_position_as_target()

- 위치: `position_controller.py:240-244`
- 입력: self
- 출력: 내부 상태 갱신이 중심이며, 필요 시 계산 결과를 return합니다.
- 역할: 현재 추정 위치를 position hold 목표로 저장합니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: DVL 기반 위치 추정과 XY position hold 힘에 영향을 준다. 좌표 변환 결과는 전진/좌우 힘 방향을 결정합니다.
- 내부 동작 흐름:
  - 현재 position x/y를 읽습니다.
  - target x/y에 저장합니다.
  - target initialized 상태를 참으로 갱신합니다.
- 코드 일부:

```python
def _capture_current_position_as_target(self):
    self.target_x = self.position_x
    self.target_y = self.position_y
    self.target_initialized = True
```

5장.12 PositionController._apply_deadband()

- 위치: `position_controller.py:245-247`
- 입력: self, value
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: _apply_deadband 함수는 5장. position_controller.py 내부의 제어 흐름을 구성하는 보조 함수이며, 상태 갱신 또는 계산 단계를 분리하기 위해 사용됩니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: DVL 기반 위치 추정과 XY position hold 힘에 영향을 준다. 좌표 변환 결과는 전진/좌우 힘 방향을 결정합니다.
- 내부 동작 흐름:
  - 입력 force 크기를 확인합니다.
  - `force_deadband`보다 작으면 0으로 만듭니다.
  - 그 외에는 원래 값을 반환합니다.
- 코드 일부:

```python
def _apply_deadband(self, value: float) -> float:
    return 0.0 if abs(value) < self.force_deadband else value
```

5장.13 PositionController._dvl_position_is_finite()

- 위치: `position_controller.py:248-250`
- 입력: self, msg
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: DVL position 메시지의 x/y가 finite 값인지 검사합니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: DVL 기반 위치 추정과 XY position hold 힘에 영향을 준다. 좌표 변환 결과는 전진/좌우 힘 방향을 결정합니다.
- 내부 동작 흐름:
  - position x 값을 finite인지 확인합니다.
  - position y 값을 finite인지 확인합니다.
  - 두 값이 모두 finite일 때만 참을 반환합니다.
- 코드 일부:

```python
def _dvl_position_is_finite(self, msg: PointStamped) -> bool:
    return math.isfinite(float(msg.point.x)) and math.isfinite(float(msg.point.y))
```

5장.14 PositionController.imu_callback()

- 위치: `position_controller.py:251-260`
- 입력: self, msg
- 출력: 직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다.
- 역할: IMU 메시지를 수신하여 현재 자세, 각속도, 또는 z축 방향 정보를 내부 상태에 저장합니다.
- 왜 사용했는가: ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.
- 제어 영향: DVL 기반 위치 추정과 XY position hold 힘에 영향을 준다. 좌표 변환 결과는 전진/좌우 힘 방향을 결정합니다.
- 내부 동작 흐름:
  - ROS2 메시지를 수신합니다.
  - orientation을 정규화해 world-body quaternion을 갱신합니다.
  - yaw rate와 IMU 유효 상태를 저장합니다.
- 코드 일부:

```python
def imu_callback(self, msg: Imu):
    self.q_world_body = quat_normalize(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w,
    )
    self.current_yaw_rate = float(msg.angular_velocity.z)
    self.have_imu = True
```

5장.15 PositionController.manual_wrench_callback()

- 위치: `position_controller.py:261-280`
- 입력: self, msg
- 출력: 직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다.
- 역할: 조종기 또는 상위 입력에서 들어오는 수동 Wrench 명령을 저장합니다.
- 왜 사용했는가: ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.
- 제어 영향: DVL 기반 위치 추정과 XY position hold 힘에 영향을 준다. 좌표 변환 결과는 전진/좌우 힘 방향을 결정합니다.
- 내부 동작 흐름:
  - ROS2 메시지를 수신합니다.
  - manual wrench와 이전 active 상태를 저장합니다.
  - manual XY override 상태를 계산하고 release edge면 현재 위치를 target으로 캡처합니다.
- 코드 일부:

```python
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
```

5장.16 PositionController.armed_callback()

- 위치: `position_controller.py:281-300`
- 입력: self, msg
- 출력: 직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다.
- 역할: armed/disarmed 상태 변화를 받아 제어 목표 및 출력을 안전하게 초기화합니다.
- 왜 사용했는가: ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.
- 제어 영향: DVL 기반 위치 추정과 XY position hold 힘에 영향을 준다. 좌표 변환 결과는 전진/좌우 힘 방향을 결정합니다.
- 내부 동작 흐름:
  - ROS2 메시지를 수신합니다.
  - armed 상태와 이전 상태를 갱신합니다.
  - arm/disarm edge에 따라 현재 위치 target 캡처 또는 0 force publish를 수행합니다.
- 코드 일부:

```python
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
```

5장.17 PositionController.dvl_position_callback()

- 위치: `position_controller.py:301-325`
- 입력: self, msg
- 출력: 직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다.
- 역할: DVL이 제공하는 위치값을 직접 사용해 현재 위치를 갱신하고 position hold 출력을 계산합니다.
- 왜 사용했는가: ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.
- 제어 영향: DVL 기반 위치 추정과 XY position hold 힘에 영향을 준다. 좌표 변환 결과는 전진/좌우 힘 방향을 결정합니다.
- 내부 동작 흐름:
  - ROS2 메시지를 수신합니다.
  - 메시지의 timestamp와 finite 여부를 확인합니다.
  - 유효하면 현재 위치와 최근 DVL 상태를 갱신하고 필요 시 control output을 계산합니다.
- 코드 일부:

```python
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
```

5장.18 PositionController.dvl_callback()

- 위치: `position_controller.py:326-374`
- 입력: self, msg
- 출력: 직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다.
- 역할: DVL 속도 데이터를 body/world frame으로 변환하고 필요 시 적분하여 위치를 추정합니다.
- 왜 사용했는가: ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.
- 제어 영향: DVL 기반 위치 추정과 XY position hold 힘에 영향을 준다. 좌표 변환 결과는 전진/좌우 힘 방향을 결정합니다.
- 내부 동작 흐름:
  - ROS2 메시지를 수신합니다.
  - body frame DVL 속도를 장착 quaternion과 IMU 자세를 이용해 world frame으로 변환합니다.
  - 필요 시 속도를 적분해 위치 추정을 갱신하고 control output을 계산합니다.
- 코드 일부:

```python
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
```

5장.19 PositionController.publish_position_estimate()

- 위치: `position_controller.py:375-383`
- 입력: self, stamp
- 출력: 내부 상태 갱신이 중심이며, 필요 시 계산 결과를 return합니다.
- 역할: 현재 position estimate를 PointStamped로 발행합니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: DVL 기반 위치 추정과 XY position hold 힘에 영향을 준다. 좌표 변환 결과는 전진/좌우 힘 방향을 결정합니다.
- 내부 동작 흐름:
  - PointStamped 메시지를 생성합니다.
  - 현재 frame id와 position x/y를 채웁니다.
  - position estimate topic으로 발행합니다.
- 코드 일부:

```python
def publish_position_estimate(self, stamp):
    msg = PointStamped()
    msg.header.stamp = stamp.to_msg()
    msg.header.frame_id = self.active_position_frame_id
    msg.point.x = self.position_x
    msg.point.y = self.position_y
    msg.point.z = 0.0
    self.pub_position.publish(msg)
```

5장.20 PositionController.publish_control_output()

- 위치: `position_controller.py:384-430`
- 입력: self, now
- 출력: 내부 상태 갱신이 중심이며, 필요 시 계산 결과를 return합니다.
- 역할: position target과 현재 위치/속도 차이로 body frame force 명령을 계산해 발행합니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: DVL 기반 위치 추정과 XY position hold 힘에 영향을 준다. 좌표 변환 결과는 전진/좌우 힘 방향을 결정합니다.
- 내부 동작 흐름:
  - position reference와 armed/control 상태가 유효한지 확인합니다.
  - 목표 위치와 현재 위치의 x/y 오차를 계산합니다.
  - yaw rate와 수동 yaw 입력에 따라 추가 damping을 계산합니다.
  - world frame force를 계산한 뒤 body frame으로 회전 변환합니다.
  - 수동 XY 입력이 active이면 position hold force를 0으로 둡니다.
  - force limit와 deadband를 적용한 뒤 position_force topic으로 발행합니다.
- 코드 일부:

```python
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
```

5장.21 PositionController.on_parameter_update()

- 위치: `position_controller.py:431-503`
- 입력: self, params
- 출력: 파라미터 갱신 결과를 `SetParametersResult`로 반환하면서 내부 상태를 함께 갱신합니다.
- 역할: ROS2 runtime parameter 변경을 노드 내부 변수에 반영합니다.
- 왜 사용했는가: 실제 로봇 테스트 중 gain과 제한값을 노드를 재시작하지 않고 바꾸기 위해 사용됩니다.
- 제어 영향: DVL 기반 위치 추정과 XY position hold 힘에 영향을 준다. 좌표 변환 결과는 전진/좌우 힘 방향을 결정합니다.
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
```

5장.22 전역 함수.main()

- 위치: `position_controller.py:504-517`
- 입력: args
- 출력: 직접적인 return 값보다는 노드 실행과 종료 처리가 핵심 출력입니다.
- 역할: rclpy를 초기화하고 노드를 생성한 뒤 spin을 수행합니다.
- 왜 사용했는가: ROS2 노드 생명주기를 시작하고 종료 처리를 안정적으로 수행하기 위해 사용됩니다.
- 제어 영향: DVL 기반 위치 추정과 XY position hold 힘에 영향을 준다. 좌표 변환 결과는 전진/좌우 힘 방향을 결정합니다.
- 내부 동작 흐름:
  - `rclpy.init()`으로 ROS2를 초기화합니다.
  - 노드 객체를 생성합니다.
  - `rclpy.spin()`으로 callback 처리를 시작합니다.
  - 종료 시 노드를 destroy하고 `rclpy.shutdown()`을 호출합니다.
- 코드 일부:

```python
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
```

## 전체 코드

```python
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
```
