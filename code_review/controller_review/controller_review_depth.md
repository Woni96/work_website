# ROV Control Code Review - 함수별 설명 문서

4장. `depth_controller.py`

수심 센서와 IMU 보정을 이용해 Heave 명령을 생성하는 수심 제어 노드

이 파일은 목표 수심과 현재 수심의 차이를 이용해 상승/하강 명령을 만듭니다. 수동 heave 조작과 자동 depth hold를 자연스럽게 연결합니다.

- 파일: `depth_controller.py`
- 함수 개수: 17
- 주요 역할: 수심 센서와 IMU 보정을 이용해 Heave 명령을 생성하는 수심 제어 노드

4장.1 전역 함수.quat_to_rotation_z_row()

- 위치: `depth_controller.py:27-42`
- 입력: x, y, z, w
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: IMU quaternion에서 body z축이 world 좌표계에서 향하는 방향 성분을 계산합니다. 기체가 기울어진 상태의 heave 보상 계산에 사용됩니다.
- 왜 사용했는가: ROV 제어에서는 자세 표현과 좌표계 변환이 계속 필요하므로, 반복되는 수학 연산을 함수로 분리한 것입니다.
- 제어 영향: 수심 목표 추종, 상승/하강 속도, 수동 heave 조작 이후의 depth hold 동작에 영향을 줍니다.
- 내부 동작 흐름:
  - 입력 quaternion의 노름을 계산합니다.
  - 노름이 너무 작으면 기본 z축 `(0, 0, 1)`을 반환합니다.
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

4장.2 DepthController.__init__()

- 위치: `depth_controller.py:45-287`
- 입력: self
- 출력: 직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다.
- 역할: ROS2 노드의 파라미터, 상태 변수, subscriber, publisher, timer를 초기화합니다. 해당 제어 노드가 시스템에 연결되는 시작점입니다.
- 왜 사용했는가: 노드가 실행되기 전에 필요한 파라미터, 통신 인터페이스, 상태 변수를 모두 준비해야 하기 때문에 사용됩니다.
- 제어 영향: 수심 목표 추종, 상승/하강 속도, 수동 heave 조작 이후의 depth hold 동작에 영향을 줍니다.
- 내부 동작 흐름:
  - 노드 이름을 설정합니다.
  - ROS parameter를 선언하고 현재 값을 읽습니다.
  - 제어에 필요한 내부 상태 변수를 초기화합니다.
  - subscriber와 publisher를 생성합니다.
  - timer 또는 parameter callback을 등록합니다.
  - 초기 설정값을 log로 출력합니다.
- 코드 일부:

```python
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
```

4장.3 DepthController.armed_callback()

- 위치: `depth_controller.py:288-311`
- 입력: self, msg
- 출력: 직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다.
- 역할: armed/disarmed 상태 변화를 받아 제어 목표 및 출력을 안전하게 초기화합니다.
- 왜 사용했는가: ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.
- 제어 영향: 수심 목표 추종, 상승/하강 속도, 수동 heave 조작 이후의 depth hold 동작에 영향을 줍니다.
- 내부 동작 흐름:
  - ROS2 메시지를 수신합니다.
  - armed 상태와 이전 상태를 갱신합니다.
  - arm/disarm edge에 따라 target, 적분항, 출력 상태를 초기화하고 필요시 publish합니다.
- 코드 일부:

```python
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
```

4장.4 DepthController.publish_depth_active()

- 위치: `depth_controller.py:312-316`
- 입력: self, active
- 출력: 내부 상태 갱신이 중심이며, 필요 시 계산 결과를 return합니다.
- 역할: 수심 제어가 실제 활성 상태인지 Bool topic으로 발행합니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: 수심 목표 추종, 상승/하강 속도, 수동 heave 조작 이후의 depth hold 동작에 영향을 줍니다.
- 내부 동작 흐름:
  - 입력 active 상태를 Bool 메시지로 변환합니다.
  - 메시지 필드를 채웁니다.
  - depth active topic으로 발행합니다.
- 코드 일부:

```python
def publish_depth_active(self, active: bool):
    msg = Bool()
    msg.data = bool(active)
    self.depth_active_pub.publish(msg)
```

4장.5 DepthController.depth_control_is_active()

- 위치: `depth_controller.py:317-325`
- 입력: self
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: armed, enable, target, sensor 조건을 확인해 depth control 활성 여부를 판단합니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: 수심 목표 추종, 상승/하강 속도, 수동 heave 조작 이후의 depth hold 동작에 영향을 줍니다.
- 내부 동작 흐름:
  - armed 수신 여부를 확인합니다.
  - armed/control enabled/target initialized/current depth 조건을 차례로 확인합니다.
  - 모든 조건이 만족되면 활성 상태를 반환합니다.
- 코드 일부:

```python
def depth_control_is_active(self) -> bool:
    return (
        self.armed_received and
        self.armed and
        self.control_enabled and
        self.target_initialized and
        self.current_depth is not None
    )
```

4장.6 DepthController.clamp_target_depth()

- 위치: `depth_controller.py:326-333`
- 입력: self
- 출력: 내부 상태 갱신이 중심이며, 필요 시 계산 결과를 return합니다.
- 역할: 목표 수심을 허용 범위 안으로 제한합니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: 수심 목표 추종, 상승/하강 속도, 수동 heave 조작 이후의 depth hold 동작에 영향을 줍니다.
- 내부 동작 흐름:
  - 최소/최대 target depth 한계를 읽습니다.
  - 하한과 상한을 정렬합니다.
  - 현재 target depth를 허용 범위 안으로 clamp합니다.
- 코드 일부:

```python
def clamp_target_depth(self):
    lo = min(float(self.min_target_depth), float(self.max_target_depth))
    hi = max(float(self.min_target_depth), float(self.max_target_depth))
    if self.target_depth < lo:
        self.target_depth = lo
    elif self.target_depth > hi:
        self.target_depth = hi
```

4장.7 DepthController.capture_manual_release_target()

- 위치: `depth_controller.py:334-345`
- 입력: self, reason
- 출력: 내부 상태 갱신이 중심이며, 필요 시 계산 결과를 return합니다.
- 역할: 수동 heave 조작이 끝났을 때 현재 수심 근처를 새 목표로 잡습니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: 수심 목표 추종, 상승/하강 속도, 수동 heave 조작 이후의 depth hold 동작에 영향을 줍니다.
- 내부 동작 흐름:
  - 현재 수심에 release offset을 더해 새 target depth를 계산합니다.
  - target depth를 허용 범위로 제한합니다.
  - target initialized와 적분항을 갱신하고 로그를 남깁니다.
- 코드 일부:

```python
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
```

4장.8 DepthController._set_control_enabled()

- 위치: `depth_controller.py:346-373`
- 입력: self, enabled
- 출력: 내부 상태 갱신이 중심이며, 필요 시 계산 결과를 return합니다.
- 역할: 제어 enable 상태 변경 시 목표값, 적분항, 출력 상태를 초기화하거나 0 출력합니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: 수심 목표 추종, 상승/하강 속도, 수동 heave 조작 이후의 depth hold 동작에 영향을 줍니다.
- 내부 동작 흐름:
  - 이전 enable 상태와 새 상태를 비교합니다.
  - enable 시 현재 수심 기반으로 target을 준비하고 적분항을 초기화합니다.
  - disable 시 0 heave publish와 상태 정리를 수행합니다.
- 코드 일부:

```python
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
```

4장.9 DepthController.cmd_depth_callback()

- 위치: `depth_controller.py:374-379`
- 입력: self, msg
- 출력: 직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다.
- 역할: 외부 목표 수심 명령을 수신해 target_depth에 반영합니다.
- 왜 사용했는가: ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.
- 제어 영향: 수심 목표 추종, 상승/하강 속도, 수동 heave 조작 이후의 depth hold 동작에 영향을 줍니다.
- 내부 동작 흐름:
  - ROS2 메시지를 수신합니다.
  - 입력 수심값을 target depth로 저장합니다.
  - target을 clamp하고 initialized 상태 및 로그를 갱신합니다.
- 코드 일부:

```python
def cmd_depth_callback(self, msg: Float64):
    self.target_depth = msg.data
    self.clamp_target_depth()
    self.target_initialized = True
    self.get_logger().info(f'Updated target depth from /cmd_depth: {self.target_depth:.3f} m')
```

4장.10 DepthController.imu_callback()

- 위치: `depth_controller.py:380-388`
- 입력: self, msg
- 출력: 직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다.
- 역할: IMU 메시지를 수신하여 현재 자세, 각속도, 또는 z축 방향 정보를 내부 상태에 저장합니다.
- 왜 사용했는가: ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.
- 제어 영향: 수심 목표 추종, 상승/하강 속도, 수동 heave 조작 이후의 depth hold 동작에 영향을 줍니다.
- 내부 동작 흐름:
  - ROS2 메시지를 수신합니다.
  - orientation에서 body z축의 world 방향 성분을 계산합니다.
  - 내부 IMU 상태와 `have_imu` 플래그를 갱신합니다.
- 코드 일부:

```python
def imu_callback(self, msg: Imu):
    self.imu_z_row = quat_to_rotation_z_row(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w,
    )
    self.have_imu = True
```

4장.11 DepthController.compensate_depth_sensor_offset()

- 위치: `depth_controller.py:389-402`
- 입력: self, sensor_depth
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: depth 센서가 로봇 중심에서 떨어진 위치에 있을 때 자세에 따른 측정 오차를 보정합니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: 수심 목표 추종, 상승/하강 속도, 수동 heave 조작 이후의 depth hold 동작에 영향을 줍니다.
- 내부 동작 흐름:
  - 보상 기능과 IMU 유효성을 확인합니다.
  - 현재 z축 방향과 센서 오프셋으로 센서의 world z 위치를 계산합니다.
  - raw sensor depth에서 자세 유래 오차를 보정한 depth를 반환합니다.
- 코드 일부:

```python
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
```

4장.12 DepthController.manual_wrench_callback()

- 위치: `depth_controller.py:403-414`
- 입력: self, msg
- 출력: 직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다.
- 역할: 조종기 또는 상위 입력에서 들어오는 수동 Wrench 명령을 저장합니다.
- 왜 사용했는가: ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.
- 제어 영향: 수심 목표 추종, 상승/하강 속도, 수동 heave 조작 이후의 depth hold 동작에 영향을 줍니다.
- 내부 동작 흐름:
  - ROS2 메시지를 수신합니다.
  - manual heave와 수신 시각을 저장합니다.
  - manual active edge를 감지하고 release 시 새 target 캡처를 수행합니다.
- 코드 일부:

```python
def manual_wrench_callback(self, msg: Wrench):
    self.manual_heave = msg.force.z
    self.last_manual_wrench_time = self.get_clock().now()

    self.prev_manual_heave_active = self.manual_heave_active
    self.manual_heave_active = abs(self.manual_heave) > self.manual_heave_override_threshold

    # Edge detect: manual heave released
    if self.prev_manual_heave_active and not self.manual_heave_active:
        if self.current_depth is not None and self.armed:
            self.capture_manual_release_target('Manual heave released')
```

4장.13 DepthController.manual_wrench_is_fresh()

- 위치: `depth_controller.py:415-423`
- 입력: self, now
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: 최근 수동 wrench 입력이 timeout 이내인지 판단합니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: 수심 목표 추종, 상승/하강 속도, 수동 heave 조작 이후의 depth hold 동작에 영향을 줍니다.
- 내부 동작 흐름:
  - timeout 설정이 비활성인지 확인합니다.
  - 마지막 manual 입력 시각이 있는지 확인합니다.
  - 현재 시각과의 차이를 계산해 freshness 여부를 반환합니다.
- 코드 일부:

```python
def manual_wrench_is_fresh(self, now) -> bool:
    if self.manual_wrench_timeout_sec <= 0.0:
        return True
    if self.last_manual_wrench_time is None:
        return False

    age = (now - self.last_manual_wrench_time).nanoseconds * 1e-9
    return age <= self.manual_wrench_timeout_sec
```

4장.14 DepthController.clear_stale_manual_heave()

- 위치: `depth_controller.py:424-434`
- 입력: self, now
- 출력: 내부 상태 갱신이 중심이며, 필요 시 계산 결과를 return합니다.
- 역할: 수동 heave 입력이 오래되면 0으로 정리하고 release target 처리를 수행합니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: 수심 목표 추종, 상승/하강 속도, 수동 heave 조작 이후의 depth hold 동작에 영향을 줍니다.
- 내부 동작 흐름:
  - manual 입력이 아직 fresh한지 확인합니다.
  - stale인데 manual active였다면 release target을 캡처합니다.
  - manual heave와 active 상태를 0/False로 정리합니다.
- 코드 일부:

```python
def clear_stale_manual_heave(self, now):
    if self.manual_wrench_is_fresh(now):
        return

    if self.manual_heave_active and self.current_depth is not None and self.armed:
        self.capture_manual_release_target('Manual heave timed out')

    self.manual_heave = 0.0
    self.prev_manual_heave_active = False
    self.manual_heave_active = False
```

4장.15 DepthController.depth_callback()

- 위치: `depth_controller.py:435-551`
- 입력: self, msg
- 출력: 직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다.
- 역할: 수심 센서 데이터를 받을 때마다 PID 기반 heave 명령을 계산하고 상태 topic을 발행합니다.
- 왜 사용했는가: ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.
- 제어 영향: 수심 목표 추종, 상승/하강 속도, 수동 heave 조작 이후의 depth hold 동작에 영향을 줍니다.
- 내부 동작 흐름:
  - depth 센서값을 읽고 센서 offset 보상을 적용합니다.
  - 수동 heave 입력 timeout을 확인합니다.
  - 초기 target depth가 없으면 현재 수심을 목표로 캡처합니다.
  - 이전 depth와 시간 차이로 depth rate를 계산하고 low-pass filter를 적용합니다.
  - armed, enable, target 조건을 확인하여 depth control 활성 상태를 판단합니다.
  - 수동 heave가 active이면 target depth를 pilot rate 방식으로 이동시킵니다.
  - PID 식으로 raw heave command를 계산하고 sign, limit, upward limit, delta limit를 적용합니다.
  - heave command, target depth, depth error, depth rate를 발행합니다.
- 코드 일부:

```python
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
```

4장.16 DepthController.on_parameter_update()

- 위치: `depth_controller.py:552-616`
- 입력: self, params
- 출력: 파라미터 갱신 결과를 `SetParametersResult`로 반환하면서 내부 상태를 함께 갱신합니다.
- 역할: ROS2 runtime parameter 변경을 노드 내부 변수에 반영합니다.
- 왜 사용했는가: 실제 로봇 테스트 중 gain과 제한값을 노드를 재시작하지 않고 바꾸기 위해 사용됩니다.
- 제어 영향: 수심 목표 추종, 상승/하강 속도, 수동 heave 조작 이후의 depth hold 동작에 영향을 줍니다.
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
```

4장.17 전역 함수.main()

- 위치: `depth_controller.py:617-630`
- 입력: args
- 출력: 직접적인 return 값보다는 노드 실행과 종료 처리가 핵심 출력입니다.
- 역할: rclpy를 초기화하고 노드를 생성한 뒤 spin을 수행합니다.
- 왜 사용했는가: ROS2 노드 생명주기를 시작하고 종료 처리를 안정적으로 수행하기 위해 사용됩니다.
- 제어 영향: 수심 목표 추종, 상승/하강 속도, 수동 heave 조작 이후의 depth hold 동작에 영향을 줍니다.
- 내부 동작 흐름:
  - `rclpy.init()`으로 ROS2를 초기화합니다.
  - 노드 객체를 생성합니다.
  - `rclpy.spin()`으로 callback 처리를 시작합니다.
  - 종료 시 노드를 destroy하고 `rclpy.shutdown()`을 호출합니다.
- 코드 일부:

```python
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
```

## 전체 코드

```python
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
```
