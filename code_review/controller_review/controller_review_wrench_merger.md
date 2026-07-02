# ROV Control Code Review - 함수별 설명 문서

6장. `wrench_merger.py`

수동 입력과 자동 제어 출력을 하나의 최종 Wrench로 병합하는 노드

이 파일은 여러 제어기의 출력을 최종 하나의 Wrench로 합칩니다. 수동 입력이 우선해야 하는 축과 자동 제어가 우선해야 하는 축을 구분하는 역할을 합니다.

- 파일: `wrench_merger.py`
- 함수 개수: 12
- 주요 역할: 수동 입력과 자동 제어 출력을 하나의 최종 Wrench로 병합하는 노드

6장.1 WrenchMerger.__init__()

- 위치: `wrench_merger.py:27-156`
- 입력: self
- 출력: 직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다.
- 역할: ROS2 노드의 파라미터, 상태 변수, subscriber, publisher, timer를 초기화합니다. 해당 제어 노드가 시스템에 연결되는 시작점입니다.
- 왜 사용했는가: 노드가 실행되기 전에 필요한 파라미터, 통신 인터페이스, 상태 변수를 모두 준비해야 하기 때문에 사용됩니다.
- 제어 영향: 수동 조작과 자동 제어의 우선순위를 결정한다. 최종 allocator로 전달되는 wrench가 이 함수의 병합 결과로 정해집니다.
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
    super().__init__('wrench_merger')

    # Parameters
    self.declare_parameter('manual_wrench_topic', '/rov/wrench_manual')
    self.declare_parameter('depth_heave_topic', '/ctrl/depth_heave')
    self.declare_parameter('depth_active_topic', '/ctrl/depth_active')
    self.declare_parameter('position_force_topic', '/ctrl/position_force')
    self.declare_parameter('attitude_torque_topic', '/ctrl/attitude_torque')
    self.declare_parameter('output_wrench_topic', '/rov/wrench_cmd')
    self.declare_parameter('armed_topic', '/rov/armed')
    self.declare_parameter('publish_rate', 50.0)

    # manual heave override threshold
    self.declare_parameter('manual_heave_override_threshold', 0.05)
    self.declare_parameter('manual_xy_override_threshold', 0.05)
    self.declare_parameter('manual_yaw_override_threshold', 0.02)
    self.declare_parameter('manual_wrench_timeout_sec', 0.5)

    self.manual_wrench_topic = self.get_parameter(
        'manual_wrench_topic').get_parameter_value().string_value
    self.depth_heave_topic = self.get_parameter(
        'depth_heave_topic').get_parameter_value().string_value
    self.depth_active_topic = self.get_parameter(
        'depth_active_topic').get_parameter_value().string_value
    self.position_force_topic = self.get_parameter(
        'position_force_topic').get_parameter_value().string_value
    self.attitude_torque_topic = self.get_parameter(
        'attitude_torque_topic').get_parameter_value().string_value
    self.output_wrench_topic = self.get_parameter(
        'output_wrench_topic').get_parameter_value().string_value
    self.armed_topic = self.get_parameter(
        'armed_topic').get_parameter_value().string_value
    self.publish_rate = self.get_parameter(
        'publish_rate').get_parameter_value().double_value
    self.manual_heave_override_threshold = self.get_parameter(
        'manual_heave_override_threshold').get_parameter_value().double_value
    self.manual_xy_override_threshold = self.get_parameter(
        'manual_xy_override_threshold').get_parameter_value().double_value
    self.manual_yaw_override_threshold = self.get_parameter(
        'manual_yaw_override_threshold').get_parameter_value().double_value
    self.manual_wrench_timeout_sec = self.get_parameter(
        'manual_wrench_timeout_sec').get_parameter_value().double_value

    # State
    self.last_manual_wrench = Wrench()
    self.last_manual_wrench_time = None
    self.last_depth_heave = 0.0
    self.depth_active = False
    self.last_position_force = Wrench()
    self.last_attitude_torque = Wrench()

    self.manual_received = False
    self.depth_received = False
    self.depth_active_received = False
    self.position_received = False
    self.attitude_received = False
    self.armed_received = False
    self.armed = False
    self._zero_log_counter = 0
    self._merge_log_counter = 0

    # Subscribers
    self.manual_sub = self.create_subscription(
        Wrench,
        self.manual_wrench_topic,
        self.manual_wrench_callback,
        CONTROL_SUB_QOS
    )

    self.depth_sub = self.create_subscription(
        Float64,
        self.depth_heave_topic,
        self.depth_heave_callback,
        CONTROL_SUB_QOS
    )
    self.depth_active_sub = self.create_subscription(
        Bool,
        self.depth_active_topic,
        self.depth_active_callback,
        CONTROL_SUB_QOS
    )

    self.position_sub = self.create_subscription(
        Wrench,
        self.position_force_topic,
        self.position_force_callback,
        CONTROL_SUB_QOS
    )

    self.attitude_sub = self.create_subscription(
        Wrench,
        self.attitude_torque_topic,
        self.attitude_torque_callback,
        CONTROL_SUB_QOS
    )

    self.armed_sub = self.create_subscription(
        Bool,
        self.armed_topic,
        self.armed_callback,
        ARMED_QOS
    )

    # Publisher
    self.wrench_pub = self.create_publisher(
        Wrench,
        self.output_wrench_topic,
        10
    )

    period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.05
    self.timer = self.create_timer(period, self.publish_merged_wrench)

    self.add_on_set_parameters_callback(self.on_parameter_update)
    self.get_logger().info('WrenchMerger initialized')
    self.get_logger().info(f'  manual_wrench_topic             = {self.manual_wrench_topic}')
    self.get_logger().info(f'  depth_heave_topic               = {self.depth_heave_topic}')
    self.get_logger().info(f'  depth_active_topic              = {self.depth_active_topic}')
    self.get_logger().info(f'  position_force_topic            = {self.position_force_topic}')
    self.get_logger().info(f'  attitude_torque_topic           = {self.attitude_torque_topic}')
    self.get_logger().info(f'  output_wrench_topic             = {self.output_wrench_topic}')
    self.get_logger().info(f'  armed_topic                     = {self.armed_topic}')
    self.get_logger().info(f'  publish_rate                    = {self.publish_rate:.1f} Hz')
    self.get_logger().info(f'  manual_heave_override_threshold = {self.manual_heave_override_threshold:.3f}')
    self.get_logger().info(f'  manual_xy_override_threshold    = {self.manual_xy_override_threshold:.3f}')
    self.get_logger().info(f'  manual_yaw_override_threshold   = {self.manual_yaw_override_threshold:.3f}')
    self.get_logger().info(f'  manual_wrench_timeout_sec       = {self.manual_wrench_timeout_sec:.3f}')
    self.get_logger().info('  initial armed state             = False')
```

6장.2 WrenchMerger.manual_wrench_callback()

- 위치: `wrench_merger.py:157-161`
- 입력: self, msg
- 출력: 직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다.
- 역할: 조종기 또는 상위 입력에서 들어오는 수동 Wrench 명령을 저장합니다.
- 왜 사용했는가: ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.
- 제어 영향: 수동 조작과 자동 제어의 우선순위를 결정한다. 최종 allocator로 전달되는 wrench가 이 함수의 병합 결과로 정해집니다.
- 내부 동작 흐름:
  - ROS2 메시지를 수신합니다.
  - 수동 wrench 값을 내부 상태에 저장합니다.
  - 수신 시각과 manual 수신 플래그를 갱신합니다.
- 코드 일부:

```python
def manual_wrench_callback(self, msg: Wrench):
    self.last_manual_wrench = msg
    self.last_manual_wrench_time = self.get_clock().now()
    self.manual_received = True
```

6장.3 WrenchMerger.depth_heave_callback()

- 위치: `wrench_merger.py:162-165`
- 입력: self, msg
- 출력: 직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다.
- 역할: 수심 제어 노드가 계산한 heave 명령을 저장합니다.
- 왜 사용했는가: ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.
- 제어 영향: 수심 목표 추종, 상승/하강 속도, 수동 heave 조작 이후의 depth hold 동작에 영향을 준다.
- 내부 동작 흐름:
  - ROS2 메시지를 수신합니다.
  - depth controller의 heave 값을 저장합니다.
  - depth 수신 플래그를 갱신합니다.
- 코드 일부:

```python
def depth_heave_callback(self, msg: Float64):
    self.last_depth_heave = msg.data
    self.depth_received = True
```

6장.4 WrenchMerger.depth_active_callback()

- 위치: `wrench_merger.py:166-169`
- 입력: self, msg
- 출력: 직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다.
- 역할: depth controller가 활성 상태인지 저장합니다.
- 왜 사용했는가: ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.
- 제어 영향: 수심 목표 추종, 상승/하강 속도, 수동 heave 조작 이후의 depth hold 동작에 영향을 준다.
- 내부 동작 흐름:
  - ROS2 Bool 메시지를 수신합니다.
  - depth active 상태를 내부 변수에 저장합니다.
  - depth active 수신 플래그를 갱신합니다.
- 코드 일부:

```python
def depth_active_callback(self, msg: Bool):
    self.depth_active = bool(msg.data)
    self.depth_active_received = True
```

6장.5 WrenchMerger.position_force_callback()

- 위치: `wrench_merger.py:170-173`
- 입력: self, msg
- 출력: 직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다.
- 역할: position controller에서 나온 force 명령을 저장합니다.
- 왜 사용했는가: ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.
- 제어 영향: DVL 기반 위치 추정과 XY position hold 힘에 영향을 준다. 좌표 변환 결과는 전진/좌우 힘 방향을 결정합니다.
- 내부 동작 흐름:
  - ROS2 메시지를 수신합니다.
  - position force wrench를 저장합니다.
  - position 수신 플래그를 갱신합니다.
- 코드 일부:

```python
def position_force_callback(self, msg: Wrench):
    self.last_position_force = msg
    self.position_received = True
```

6장.6 WrenchMerger.attitude_torque_callback()

- 위치: `wrench_merger.py:174-177`
- 입력: self, msg
- 출력: 직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다.
- 역할: attitude controller에서 나온 torque 명령을 저장합니다.
- 왜 사용했는가: ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.
- 제어 영향: roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.
- 내부 동작 흐름:
  - ROS2 메시지를 수신합니다.
  - attitude torque wrench를 저장합니다.
  - attitude 수신 플래그를 갱신합니다.
- 코드 일부:

```python
def attitude_torque_callback(self, msg: Wrench):
    self.last_attitude_torque = msg
    self.attitude_received = True
```

6장.7 WrenchMerger.armed_callback()

- 위치: `wrench_merger.py:178-185`
- 입력: self, msg
- 출력: 직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다.
- 역할: armed/disarmed 상태 변화를 받아 제어 목표 및 출력을 안전하게 초기화합니다.
- 왜 사용했는가: ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.
- 제어 영향: 수동 조작과 자동 제어의 우선순위를 결정한다. 최종 allocator로 전달되는 wrench가 이 함수의 병합 결과로 정해집니다.
- 내부 동작 흐름:
  - ROS2 메시지를 수신합니다.
  - armed 상태와 이전 상태를 갱신합니다.
  - 상태 변화가 있으면 로그를 남기고 이후 병합 판단에 사용할 armed 상태를 유지합니다.
- 코드 일부:

```python
def armed_callback(self, msg: Bool):
    prev = self.armed
    self.armed = bool(msg.data)
    self.armed_received = True

    if prev != self.armed:
        self.get_logger().info(f'armed state changed: {self.armed}')
```

6장.8 WrenchMerger.publish_zero_wrench()

- 위치: `wrench_merger.py:186-188`
- 입력: self
- 출력: 내부 상태 갱신이 중심이며, 필요 시 계산 결과를 return합니다.
- 역할: 최종 Wrench를 0으로 발행합니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: 수동 조작과 자동 제어의 우선순위를 결정한다. 최종 allocator로 전달되는 wrench가 이 함수의 병합 결과로 정해집니다.
- 내부 동작 흐름:
  - 빈 `Wrench()` 메시지를 준비합니다.
  - 출력 publisher를 통해 0 wrench를 발행합니다.
  - 비정상 상태나 disarmed 상태에서 안전한 기본 출력을 제공합니다.
- 코드 일부:

```python
def publish_zero_wrench(self):
    self.wrench_pub.publish(Wrench())
```

6장.9 WrenchMerger.manual_wrench_is_fresh()

- 위치: `wrench_merger.py:189-201`
- 입력: self
- 출력: 계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다.
- 역할: 최근 수동 wrench 입력이 timeout 이내인지 판단합니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: 수동 조작과 자동 제어의 우선순위를 결정한다. 최종 allocator로 전달되는 wrench가 이 함수의 병합 결과로 정해집니다.
- 내부 동작 흐름:
  - manual 입력을 한 번이라도 받았는지 확인합니다.
  - timeout 설정과 마지막 수신 시각을 확인합니다.
  - 현재 시각과의 차이를 계산해 freshness 여부를 반환합니다.
- 코드 일부:

```python
def manual_wrench_is_fresh(self) -> bool:
    if not self.manual_received:
        return False
    if self.manual_wrench_timeout_sec <= 0.0:
        return True
    if self.last_manual_wrench_time is None:
        return False

    age = (
        self.get_clock().now() - self.last_manual_wrench_time
    ).nanoseconds * 1e-9
    return age <= self.manual_wrench_timeout_sec
```

6장.10 WrenchMerger.publish_merged_wrench()

- 위치: `wrench_merger.py:202-283`
- 입력: self
- 출력: 내부 상태 갱신이 중심이며, 필요 시 계산 결과를 return합니다.
- 역할: 수동 입력, depth heave, position force, attitude torque를 우선순위 규칙에 따라 병합해 최종 Wrench를 발행합니다.
- 왜 사용했는가: 복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.
- 제어 영향: 수동 조작과 자동 제어의 우선순위를 결정한다. 최종 allocator로 전달되는 wrench가 이 함수의 병합 결과로 정해집니다.
- 내부 동작 흐름:
  - armed 상태를 확인하고 아직 수신 전이거나 disarmed이면 zero wrench를 발행합니다.
  - manual wrench freshness를 확인해 오래된 수동 입력은 0으로 처리합니다.
  - surge/sway는 수동 입력이 threshold를 넘으면 수동값을, 아니면 position force를 사용합니다.
  - heave는 depth_active 상태에 따라 자동 depth heave를 사용하거나 수동 override를 허용합니다.
  - roll/pitch torque는 attitude controller 출력을 사용합니다.
  - yaw는 수동 yaw 입력이 active이면 수동값을, 아니면 attitude yaw torque를 사용합니다.
  - 병합된 최종 Wrench를 `/rov/wrench_cmd`로 발행합니다.
- 코드 일부:

```python
def publish_merged_wrench(self):
    # armed 신호를 아직 못 받았으면 안전하게 0 유지
    if not self.armed_received:
        self.publish_zero_wrench()
        self._zero_log_counter += 1
        if self._zero_log_counter % 100 == 1:
            self.get_logger().warn('armed state not received yet -> publishing zero wrench')
        self.get_logger().debug('armed state not received yet -> publishing zero wrench')
        return

    # disarm이면 어떤 자동제어가 살아있어도 최종 출력은 무조건 0
    if not self.armed:
        self.publish_zero_wrench()
        self._zero_log_counter += 1
        if self._zero_log_counter % 100 == 1:
            self.get_logger().warn('DISARMED -> publishing zero wrench')
        self.get_logger().debug('DISARMED -> publishing zero wrench')
        return

    manual_wrench = self.last_manual_wrench if self.manual_wrench_is_fresh() else Wrench()
    merged = copy.deepcopy(manual_wrench)

    manual_surge = manual_wrench.force.x
    manual_sway = manual_wrench.force.y
    manual_heave = manual_wrench.force.z

    if abs(manual_surge) > self.manual_xy_override_threshold:
        merged.force.x = manual_surge
    else:
        merged.force.x = self.last_position_force.force.x

    if abs(manual_sway) > self.manual_xy_override_threshold:
        merged.force.y = manual_sway
    else:
        merged.force.y = self.last_position_force.force.y

    # BlueROV-style depth hold:
    # when depth control is active, joystick heave is consumed by the
    # depth controller as a climb/descent-rate request. The final heave
    # output stays automatic so releasing the stick immediately holds depth.
    if self.depth_active:
        merged.force.z = self.last_depth_heave + self.last_position_force.force.z
        heave_mode = 'AUTO_DEPTH_HOLD'
    elif abs(manual_heave) > self.manual_heave_override_threshold:
        merged.force.z = manual_heave
        heave_mode = 'MANUAL_HEAVE_OVERRIDE'
    else:
        merged.force.z = self.last_depth_heave + self.last_position_force.force.z
        heave_mode = 'AUTO_DEPTH_PLUS_POSITION_COMP'

    # Attitude auto control overrides roll/pitch torques
    merged.torque.x = self.last_attitude_torque.torque.x
    merged.torque.y = self.last_attitude_torque.torque.y

    # Keep manual yaw command responsive. If user yaw stick is active, manual wins.
    manual_yaw = manual_wrench.torque.z
    if abs(manual_yaw) > self.manual_yaw_override_threshold:
        merged.torque.z = manual_yaw
    else:
        merged.torque.z = self.last_attitude_torque.torque.z

    self.wrench_pub.publish(merged)
    self._merge_log_counter += 1
    if self._merge_log_counter % 100 == 1:
        self.get_logger().info(
            f'merged wrench: heave_mode={heave_mode}, '
            f'manual_heave={manual_heave:.4f}, '
            f'depth_heave={self.last_depth_heave:.4f}, '
            f'out_force_z={merged.force.z:.4f}'
        )

    # self.get_logger().debug(
    #     f'armed={self.armed}, '
    #     f'heave_mode={heave_mode}, '
    #     f'manual_received={self.manual_received}, '
    #     f'depth_received={self.depth_received}, '
    #     f'attitude_received={self.attitude_received}, '
    #     f'manual_heave={manual_heave:.3f}, auto_heave={self.last_depth_heave:.3f}, '
    #     f'fx={merged.force.x:.3f}, fy={merged.force.y:.3f}, fz={merged.force.z:.3f}, '
    #     f'tx={merged.torque.x:.3f}, ty={merged.torque.y:.3f}, tz={merged.torque.z:.3f}'
    # )
```

6장.11 WrenchMerger.on_parameter_update()

- 위치: `wrench_merger.py:284-311`
- 입력: self, params
- 출력: 파라미터 갱신 결과를 `SetParametersResult`로 반환하면서 내부 상태를 함께 갱신합니다.
- 역할: ROS2 runtime parameter 변경을 노드 내부 변수에 반영합니다.
- 왜 사용했는가: 실제 로봇 테스트 중 gain과 제한값을 노드를 재시작하지 않고 바꾸기 위해 사용됩니다.
- 제어 영향: 수동 조작과 자동 제어의 우선순위를 결정한다. 최종 allocator로 전달되는 wrench가 이 함수의 병합 결과로 정해집니다.
- 내부 동작 흐름:
  - 변경 요청된 parameter 목록을 순회합니다.
  - parameter 이름에 맞는 내부 변수를 갱신합니다.
  - 필요하면 publish timer 재생성 여부를 기록합니다.
  - 갱신 결과를 log로 남깁니다.
  - 성공 또는 실패 결과를 `SetParametersResult`로 반환합니다.
- 코드 일부:

```python
def on_parameter_update(self, params):
    try:
        recreate_timer = False

        for p in params:
            if p.name == 'publish_rate':
                self.publish_rate = float(p.value)
                recreate_timer = True
            elif p.name == 'manual_heave_override_threshold':
                self.manual_heave_override_threshold = float(p.value)
            elif p.name == 'manual_xy_override_threshold':
                self.manual_xy_override_threshold = float(p.value)
            elif p.name == 'manual_yaw_override_threshold':
                self.manual_yaw_override_threshold = float(p.value)
            elif p.name == 'manual_wrench_timeout_sec':
                self.manual_wrench_timeout_sec = float(p.value)

        if recreate_timer:
            self.timer.cancel()
            period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.05
            self.timer = self.create_timer(period, self.publish_merged_wrench)

        self.get_logger().info('WrenchMerger parameters updated at runtime')
        return SetParametersResult(successful=True)
    except Exception as e:
        return SetParametersResult(successful=False, reason=str(e))
```

6장.12 전역 함수.main()

- 위치: `wrench_merger.py:312-325`
- 입력: args
- 출력: 직접적인 return 값보다는 노드 실행과 종료 처리가 핵심 출력입니다.
- 역할: rclpy를 초기화하고 노드를 생성한 뒤 spin을 수행합니다.
- 왜 사용했는가: ROS2 노드 생명주기를 시작하고 종료 처리를 안정적으로 수행하기 위해 사용됩니다.
- 제어 영향: 수동 조작과 자동 제어의 우선순위를 결정한다. 최종 allocator로 전달되는 wrench가 이 함수의 병합 결과로 정해집니다.
- 내부 동작 흐름:
  - `rclpy.init()`으로 ROS2를 초기화합니다.
  - 노드 객체를 생성합니다.
  - `rclpy.spin()`으로 callback 처리를 시작합니다.
  - 종료 시 노드를 destroy하고 `rclpy.shutdown()`을 호출합니다.
- 코드 일부:

```python
def main(args=None):
    rclpy.init(args=args)
    node = WrenchMerger()
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
import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64, Bool
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


class WrenchMerger(Node):

    def __init__(self):
        super().__init__('wrench_merger')

        # Parameters
        self.declare_parameter('manual_wrench_topic', '/rov/wrench_manual')
        self.declare_parameter('depth_heave_topic', '/ctrl/depth_heave')
        self.declare_parameter('depth_active_topic', '/ctrl/depth_active')
        self.declare_parameter('position_force_topic', '/ctrl/position_force')
        self.declare_parameter('attitude_torque_topic', '/ctrl/attitude_torque')
        self.declare_parameter('output_wrench_topic', '/rov/wrench_cmd')
        self.declare_parameter('armed_topic', '/rov/armed')
        self.declare_parameter('publish_rate', 50.0)

        # manual heave override threshold
        self.declare_parameter('manual_heave_override_threshold', 0.05)
        self.declare_parameter('manual_xy_override_threshold', 0.05)
        self.declare_parameter('manual_yaw_override_threshold', 0.02)
        self.declare_parameter('manual_wrench_timeout_sec', 0.5)

        self.manual_wrench_topic = self.get_parameter(
            'manual_wrench_topic').get_parameter_value().string_value
        self.depth_heave_topic = self.get_parameter(
            'depth_heave_topic').get_parameter_value().string_value
        self.depth_active_topic = self.get_parameter(
            'depth_active_topic').get_parameter_value().string_value
        self.position_force_topic = self.get_parameter(
            'position_force_topic').get_parameter_value().string_value
        self.attitude_torque_topic = self.get_parameter(
            'attitude_torque_topic').get_parameter_value().string_value
        self.output_wrench_topic = self.get_parameter(
            'output_wrench_topic').get_parameter_value().string_value
        self.armed_topic = self.get_parameter(
            'armed_topic').get_parameter_value().string_value
        self.publish_rate = self.get_parameter(
            'publish_rate').get_parameter_value().double_value
        self.manual_heave_override_threshold = self.get_parameter(
            'manual_heave_override_threshold').get_parameter_value().double_value
        self.manual_xy_override_threshold = self.get_parameter(
            'manual_xy_override_threshold').get_parameter_value().double_value
        self.manual_yaw_override_threshold = self.get_parameter(
            'manual_yaw_override_threshold').get_parameter_value().double_value
        self.manual_wrench_timeout_sec = self.get_parameter(
            'manual_wrench_timeout_sec').get_parameter_value().double_value

        # State
        self.last_manual_wrench = Wrench()
        self.last_manual_wrench_time = None
        self.last_depth_heave = 0.0
        self.depth_active = False
        self.last_position_force = Wrench()
        self.last_attitude_torque = Wrench()

        self.manual_received = False
        self.depth_received = False
        self.depth_active_received = False
        self.position_received = False
        self.attitude_received = False
        self.armed_received = False
        self.armed = False
        self._zero_log_counter = 0
        self._merge_log_counter = 0

        # Subscribers
        self.manual_sub = self.create_subscription(
            Wrench,
            self.manual_wrench_topic,
            self.manual_wrench_callback,
            CONTROL_SUB_QOS
        )

        self.depth_sub = self.create_subscription(
            Float64,
            self.depth_heave_topic,
            self.depth_heave_callback,
            CONTROL_SUB_QOS
        )
        self.depth_active_sub = self.create_subscription(
            Bool,
            self.depth_active_topic,
            self.depth_active_callback,
            CONTROL_SUB_QOS
        )

        self.position_sub = self.create_subscription(
            Wrench,
            self.position_force_topic,
            self.position_force_callback,
            CONTROL_SUB_QOS
        )

        self.attitude_sub = self.create_subscription(
            Wrench,
            self.attitude_torque_topic,
            self.attitude_torque_callback,
            CONTROL_SUB_QOS
        )

        self.armed_sub = self.create_subscription(
            Bool,
            self.armed_topic,
            self.armed_callback,
            ARMED_QOS
        )

        # Publisher
        self.wrench_pub = self.create_publisher(
            Wrench,
            self.output_wrench_topic,
            10
        )

        period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.05
        self.timer = self.create_timer(period, self.publish_merged_wrench)

        self.add_on_set_parameters_callback(self.on_parameter_update)
        self.get_logger().info('WrenchMerger initialized')
        self.get_logger().info(f'  manual_wrench_topic             = {self.manual_wrench_topic}')
        self.get_logger().info(f'  depth_heave_topic               = {self.depth_heave_topic}')
        self.get_logger().info(f'  depth_active_topic              = {self.depth_active_topic}')
        self.get_logger().info(f'  position_force_topic            = {self.position_force_topic}')
        self.get_logger().info(f'  attitude_torque_topic           = {self.attitude_torque_topic}')
        self.get_logger().info(f'  output_wrench_topic             = {self.output_wrench_topic}')
        self.get_logger().info(f'  armed_topic                     = {self.armed_topic}')
        self.get_logger().info(f'  publish_rate                    = {self.publish_rate:.1f} Hz')
        self.get_logger().info(f'  manual_heave_override_threshold = {self.manual_heave_override_threshold:.3f}')
        self.get_logger().info(f'  manual_xy_override_threshold    = {self.manual_xy_override_threshold:.3f}')
        self.get_logger().info(f'  manual_yaw_override_threshold   = {self.manual_yaw_override_threshold:.3f}')
        self.get_logger().info(f'  manual_wrench_timeout_sec       = {self.manual_wrench_timeout_sec:.3f}')
        self.get_logger().info('  initial armed state             = False')

    def manual_wrench_callback(self, msg: Wrench):
        self.last_manual_wrench = msg
        self.last_manual_wrench_time = self.get_clock().now()
        self.manual_received = True

    def depth_heave_callback(self, msg: Float64):
        self.last_depth_heave = msg.data
        self.depth_received = True

    def depth_active_callback(self, msg: Bool):
        self.depth_active = bool(msg.data)
        self.depth_active_received = True

    def position_force_callback(self, msg: Wrench):
        self.last_position_force = msg
        self.position_received = True

    def attitude_torque_callback(self, msg: Wrench):
        self.last_attitude_torque = msg
        self.attitude_received = True

    def armed_callback(self, msg: Bool):
        prev = self.armed
        self.armed = bool(msg.data)
        self.armed_received = True

        if prev != self.armed:
            self.get_logger().info(f'armed state changed: {self.armed}')

    def publish_zero_wrench(self):
        self.wrench_pub.publish(Wrench())

    def manual_wrench_is_fresh(self) -> bool:
        if not self.manual_received:
            return False
        if self.manual_wrench_timeout_sec <= 0.0:
            return True
        if self.last_manual_wrench_time is None:
            return False

        age = (
            self.get_clock().now() - self.last_manual_wrench_time
        ).nanoseconds * 1e-9
        return age <= self.manual_wrench_timeout_sec

    def publish_merged_wrench(self):
        # armed 신호를 아직 못 받았으면 안전하게 0 유지
        if not self.armed_received:
            self.publish_zero_wrench()
            self._zero_log_counter += 1
            if self._zero_log_counter % 100 == 1:
                self.get_logger().warn('armed state not received yet -> publishing zero wrench')
            self.get_logger().debug('armed state not received yet -> publishing zero wrench')
            return

        # disarm이면 어떤 자동제어가 살아있어도 최종 출력은 무조건 0
        if not self.armed:
            self.publish_zero_wrench()
            self._zero_log_counter += 1
            if self._zero_log_counter % 100 == 1:
                self.get_logger().warn('DISARMED -> publishing zero wrench')
            self.get_logger().debug('DISARMED -> publishing zero wrench')
            return

        manual_wrench = self.last_manual_wrench if self.manual_wrench_is_fresh() else Wrench()
        merged = copy.deepcopy(manual_wrench)

        manual_surge = manual_wrench.force.x
        manual_sway = manual_wrench.force.y
        manual_heave = manual_wrench.force.z

        if abs(manual_surge) > self.manual_xy_override_threshold:
            merged.force.x = manual_surge
        else:
            merged.force.x = self.last_position_force.force.x

        if abs(manual_sway) > self.manual_xy_override_threshold:
            merged.force.y = manual_sway
        else:
            merged.force.y = self.last_position_force.force.y

        # BlueROV-style depth hold:
        # when depth control is active, joystick heave is consumed by the
        # depth controller as a climb/descent-rate request. The final heave
        # output stays automatic so releasing the stick immediately holds depth.
        if self.depth_active:
            merged.force.z = self.last_depth_heave + self.last_position_force.force.z
            heave_mode = 'AUTO_DEPTH_HOLD'
        elif abs(manual_heave) > self.manual_heave_override_threshold:
            merged.force.z = manual_heave
            heave_mode = 'MANUAL_HEAVE_OVERRIDE'
        else:
            merged.force.z = self.last_depth_heave + self.last_position_force.force.z
            heave_mode = 'AUTO_DEPTH_PLUS_POSITION_COMP'

        # Attitude auto control overrides roll/pitch torques
        merged.torque.x = self.last_attitude_torque.torque.x
        merged.torque.y = self.last_attitude_torque.torque.y

        # Keep manual yaw command responsive. If user yaw stick is active, manual wins.
        manual_yaw = manual_wrench.torque.z
        if abs(manual_yaw) > self.manual_yaw_override_threshold:
            merged.torque.z = manual_yaw
        else:
            merged.torque.z = self.last_attitude_torque.torque.z

        self.wrench_pub.publish(merged)
        self._merge_log_counter += 1
        if self._merge_log_counter % 100 == 1:
            self.get_logger().info(
                f'merged wrench: heave_mode={heave_mode}, '
                f'manual_heave={manual_heave:.4f}, '
                f'depth_heave={self.last_depth_heave:.4f}, '
                f'out_force_z={merged.force.z:.4f}'
            )

        # self.get_logger().debug(
        #     f'armed={self.armed}, '
        #     f'heave_mode={heave_mode}, '
        #     f'manual_received={self.manual_received}, '
        #     f'depth_received={self.depth_received}, '
        #     f'attitude_received={self.attitude_received}, '
        #     f'manual_heave={manual_heave:.3f}, auto_heave={self.last_depth_heave:.3f}, '
        #     f'fx={merged.force.x:.3f}, fy={merged.force.y:.3f}, fz={merged.force.z:.3f}, '
        #     f'tx={merged.torque.x:.3f}, ty={merged.torque.y:.3f}, tz={merged.torque.z:.3f}'
        # )

    def on_parameter_update(self, params):
        try:
            recreate_timer = False

            for p in params:
                if p.name == 'publish_rate':
                    self.publish_rate = float(p.value)
                    recreate_timer = True
                elif p.name == 'manual_heave_override_threshold':
                    self.manual_heave_override_threshold = float(p.value)
                elif p.name == 'manual_xy_override_threshold':
                    self.manual_xy_override_threshold = float(p.value)
                elif p.name == 'manual_yaw_override_threshold':
                    self.manual_yaw_override_threshold = float(p.value)
                elif p.name == 'manual_wrench_timeout_sec':
                    self.manual_wrench_timeout_sec = float(p.value)

            if recreate_timer:
                self.timer.cancel()
                period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.05
                self.timer = self.create_timer(period, self.publish_merged_wrench)

            self.get_logger().info('WrenchMerger parameters updated at runtime')
            return SetParametersResult(successful=True)
        except Exception as e:
            return SetParametersResult(successful=False, reason=str(e))


def main(args=None):
    rclpy.init(args=args)
    node = WrenchMerger()
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
