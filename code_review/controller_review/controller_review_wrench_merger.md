# Wrench Merger Review

대상 파일: `code_review/code/wrench_merger.py`

## 역할
이 코드는 manual input, depth, position, attitude 출력을 축별 규칙으로 합쳐 최종 `Wrench`를 만드는 supervisory mixer입니다.

## 설계 해석
핵심 설계는 `manual override + auto hold arbitration`입니다. 각 축별로 manual과 auto의 우선순위를 다르게 해석해 최종 명령을 만듭니다.

## 리뷰 초점
리뷰 포인트는 `manual freshness가 어떻게 적용되는지`, `depth active일 때 heave를 어떻게 해석하는지`, `roll/pitch와 yaw의 arbitration 철학이 어떻게 다른지`입니다.

## 런타임 동작 해설
이 모듈은 각 callback에서 manual/depth/position/attitude 입력의 최신값만 저장하고, 실제 merge는 timer 기반 `publish_merged_wrench()`에서 수행합니다. 그래서 입력 주기가 서로 달라도 최종 wrench 주기를 일정하게 유지할 수 있고, armed/disarmed 상태를 마지막 게이트로 적용할 수 있습니다.

## 핵심 파라미터
- `publish_rate`: 최종 wrench를 얼마 주기로 publish할지 정합니다.
- `manual_wrench_timeout_sec`: manual input이 stale이면 zero wrench로 간주하는 기준 시간입니다.
- `manual_xy_override_threshold`: XY 축에서 manual이 auto position보다 우선하는 경계값입니다.
- `manual_heave_override_threshold`: heave 축에서 manual이 auto depth보다 우선하는 경계값입니다.
- `manual_yaw_override_threshold`: yaw 축에서 manual yaw가 attitude yaw hold보다 우선하는 경계값입니다.

## 함수 맵
- `__init__()`
- `manual_wrench_callback()`
- `depth_heave_callback()`
- `depth_active_callback()`
- `position_force_callback()`
- `attitude_torque_callback()`
- `armed_callback()`
- `publish_zero_wrench()`
- `manual_wrench_is_fresh()`
- `publish_merged_wrench()`
- `on_parameter_update()`
- `main()`

## 함수 리뷰

### `__init__()`

**의미**

입력 토픽, armed gating, manual override threshold, publish timer와 마지막 입력 상태들을 준비합니다.

**영향**

이 함수가 곧 이 노드의 arbitration 정책 초기 상태를 만듭니다. 특히 timer 기반 publish는 입력 주기가 달라도 최종 wrench 주기를 일정하게 유지합니다.

**리뷰 메모**

구조가 단순하고 목적이 분명합니다. 이런 중간 supervisory layer가 있으면 상위 제어기와 allocator를 느슨하게 연결할 수 있습니다.

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

### 입력 저장 함수

**의미**

각 제어기 출력을 최신값으로 저장하고 armed 상태를 갱신하는 함수들입니다.

**영향**

이 함수들이 직접 최종 출력은 만들지 않지만, timer 루프가 읽을 최신 상태를 보관합니다.

**리뷰 메모**

timer 기반 merge 구조와 잘 맞습니다. 다만 manual 이외 auto 입력에 freshness timestamp가 없어 마지막 자동 출력이 오래 남을 수 있습니다.

```python
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
```

### `manual_wrench_is_fresh()`

**의미**

manual input이 최근에 들어왔는지 판정하고 stale이면 사실상 zero wrench로 취급하게 만드는 안전 함수입니다.

**영향**

조종기 신호 유실 시 마지막 manual 명령이 latch되는 문제를 막아줍니다.

**리뷰 메모**

현재 코드베이스에서 freshness 처리가 가장 깔끔하게 들어간 부분입니다. 같은 아이디어가 auto inputs에도 확장되면 훨씬 견고해집니다.

```python
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
```

### `publish_merged_wrench()`

**의미**

armed gating, manual freshness 확인, 축별 override 규칙, depth active 정책, yaw arbitration을 적용해 최종 wrench를 publish합니다.

**영향**

이 함수가 전체 제어 시스템의 최종 의사결정자입니다. 같은 upper controller 출력이라도 여기서 어떤 축은 manual이 이기고 어떤 축은 auto가 이깁니다.

**리뷰 메모**

정책은 명료하고 읽기 쉽습니다. 다만 `last_position_force`, `last_depth_heave`, `last_attitude_torque`에도 freshness를 넣지 않으면 상위 노드 장애 시 마지막 값이 계속 살아남습니다.

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

### `on_parameter_update()`

**의미**

publish rate와 override threshold를 런타임에 갱신하고 timer도 재생성합니다.

**영향**

운용자가 merge 주기와 manual override 감도를 실시간 조정할 수 있습니다.

**리뷰 메모**

supervisory node답게 runtime tuning이 단순하고 이해하기 쉽습니다.

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
