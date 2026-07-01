# Depth Controller Review

대상 파일: `code_review/code/depth_controller.py`

## 역할
이 코드는 depth sensor와 IMU, manual heave 입력을 받아 최종 `Fz` 명령을 만드는 수심 유지 제어기입니다.

## 설계 해석
설계의 핵심은 `depth hold + pilot depth-rate`입니다. 즉 stick을 위아래로 움직이면 즉시 추진기 출력만 주는 것이 아니라, 목표 수심을 움직이는 방식으로 조종감을 만듭니다.

## 리뷰 초점
리뷰 포인트는 `target depth를 언제 어떻게 캡처하는지`, `manual heave freshness를 어떻게 지우는지`, `depth sensor offset을 IMU로 어떻게 보정하는지`, `최종 heave shaping이 actuator-friendly한지`입니다.

## 런타임 동작 해설
런타임에서는 `depth_callback()`가 depth sample을 받을 때마다 현재 depth, depth rate, manual heave 상태를 갱신하고 필요하면 target depth를 다시 잡습니다. 이후 PID, pilot depth-rate, saturation, upward limit, slew-rate 제한을 거쳐 최종 `Fz` 명령이 만들어집니다. 즉 이 모듈은 단순 PID가 아니라 상태 전이와 출력 shaping이 함께 들어간 depth hold 시스템입니다.

## 핵심 파라미터
- `kp_depth`, `ki_depth`, `kd_depth`: 수심 오차를 heave 명령으로 바꾸는 핵심 PID 게인입니다.
- `manual_heave_override_threshold`: 이 값보다 큰 manual heave는 조종자가 depth hold를 직접 흔드는 입력으로 해석됩니다.
- `manual_wrench_timeout_sec`: manual heave 입력이 이 시간 이상 갱신되지 않으면 stale로 보고 0으로 복구합니다.
- `pilot_depth_rate_enabled`: manual heave를 직접 추력 명령으로 볼지, 목표 수심의 변화율로 볼지 정합니다.
- `max_pilot_depth_rate`, `pilot_depth_rate_sign`: pilot depth-rate 모드에서 stick 입력이 target depth를 얼마나 빠르게 움직일지 정합니다.
- `manual_heave_release_target_offset`: manual heave를 놓았을 때 현재 depth에 더해 새 target으로 삼는 오프셋입니다.
- `max_heave`, `max_upward_heave`: 최종 heave 출력의 절대 한계와 상승 방향 한계를 정합니다.
- `max_heave_delta_per_cycle`: 한 제어 주기에서 heave가 얼마나 급하게 바뀔 수 있는지 제한합니다.
- `depth_rate_alpha`: depth 미분값에 low-pass filter를 얼마나 강하게 적용할지 정합니다.
- `depth_sensor_offset_x/y/z`, `depth_sensor_offset_compensation_enabled`: 센서 장착 위치 보정과 IMU 기반 compensation 사용 여부를 정합니다.

## 함수 맵
- `quat_to_rotation_z_row()`
- `__init__()`
- `armed_callback()`
- `publish_depth_active()`
- `depth_control_is_active()`
- `clamp_target_depth()`
- `capture_manual_release_target()`
- `_set_control_enabled()`
- `cmd_depth_callback()`
- `imu_callback()`
- `compensate_depth_sensor_offset()`
- `manual_wrench_callback()`
- `manual_wrench_is_fresh()`
- `clear_stale_manual_heave()`
- `depth_callback()`
- `on_parameter_update()`
- `main()`

## 함수 리뷰

### `quat_to_rotation_z_row()`

**의미**

IMU quaternion에서 body z축이 world에서 어디를 보는지 계산하는 보조 함수입니다.

**영향**

depth sensor가 기체 중심에서 떨어져 있을 때, pitch/roll에 의해 측정 depth가 흔들리는 문제를 보정할 수 있게 해줍니다.

**리뷰 메모**

작은 함수지만 depth sensor offset compensation의 핵심 기반입니다. 이 함수가 없으면 IMU를 depth controller가 활용할 방법이 사라집니다.

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

### `__init__()`

**의미**

depth PID, manual override, pilot depth-rate, sensor offset compensation, output shaping 파라미터를 한 번에 선언합니다. 즉 depth controller가 단순한 `error -> heave` 계산기인지, 실제 운용형 hold 시스템인지가 여기서 갈립니다.

**영향**

이 함수가 depth hold의 운용 정책 전체를 결정합니다. 즉 단순 PID가 아니라 실제 조종기, arming, offset compensation, target clamp까지 모두 여기서 준비됩니다.

**리뷰 메모**

구성이 잘 되어 있고 실제 운용형 controller에 가깝습니다. 특히 `manual_wrench_timeout_sec`가 있는 점은 attitude/position보다 안전 측면에서 낫습니다.

**상세 해설**

이 함수는 depth controller를 읽을 때 가장 먼저 봐야 하는 구간입니다. 왜냐하면 depth hold의 감각은 `kp/ki/kd`보다도 `pilot_depth_rate_enabled`, `manual_heave_release_target_offset`, `max_upward_heave`, `depth_rate_alpha` 같은 정책 파라미터들에 크게 좌우되기 때문입니다.

또한 이 함수는 sensor offset compensation 관련 파라미터도 함께 준비합니다. 즉 이 모듈은 단순 PID가 아니라 센서 물리 배치, arm/disarm 동작, manual release 감각, actuator 친화성까지 같이 품고 있는 시스템입니다.

**이 함수와 관련된 파라미터**

- `depth_topic`, `imu_topic`, `cmd_depth_topic`, `manual_wrench_topic`: depth hold에 들어오는 주요 센서/명령 입력입니다.
- `kp_depth`, `ki_depth`, `kd_depth`: 수심 오차를 heave 출력으로 바꾸는 핵심 PID 게인입니다.
- `pilot_depth_rate_enabled`, `max_pilot_depth_rate`, `pilot_depth_rate_sign`: manual heave를 direct thrust가 아니라 target depth rate로 해석할지 결정합니다.
- `manual_heave_override_threshold`, `manual_wrench_timeout_sec`: manual heave 활성 조건과 stale input 해제 정책을 정합니다.
- `manual_heave_release_target_offset`: pilot이 stick을 놓은 뒤 hold target을 현재 depth 기준 어디에 둘지 정합니다.
- `max_heave`, `max_upward_heave`, `max_heave_delta_per_cycle`: 출력 saturation과 actuator-friendly shaping을 담당합니다.
- `depth_sensor_offset_*`, `depth_sensor_offset_compensation_enabled`: 센서 장착 위치 보상에 필요한 파라미터입니다.

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

### Arming / Active 상태 함수

**의미**

이 함수들은 controller가 언제 활성인지, target depth를 허용 범위 안에 둘지, arm/disarm에서 어떤 초기화를 할지 정의합니다.

**영향**

ROV가 arm 될 때 current depth를 target으로 잡고, disarm 시 integrator와 출력 상태를 초기화합니다. 즉 이 계층은 안전성과 target 일관성을 담당합니다.

**리뷰 메모**

상태 전이가 비교적 명확합니다. 실무에서는 `depth_control_is_active()` 같은 함수가 있어야 상위 시스템에서 상태를 해석하기 쉬워집니다.

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
```

### `capture_manual_release_target()`

**의미**

manual heave를 놓았을 때 현재 depth에 release offset을 더해 새 target depth를 잡는 함수입니다.

**영향**

이 함수가 조종자가 stick을 놓은 뒤 depth hold가 어디에서 다시 잠길지를 결정합니다.

**리뷰 메모**

운용 철학은 분명하지만 `current_depth + offset`은 직관과 다를 수 있습니다. 오프셋이 항상 들어가면 '놓은 자리 유지'보다 '조금 이동한 자리 유지'가 되어 사용자 혼란을 줄 수 있습니다.

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

### `compensate_depth_sensor_offset()`

**의미**

IMU로 센서의 world z 위치 변화를 계산해, pitch/roll 때문에 생기는 depth sensor 오차를 보정합니다.

**영향**

기체가 기울어져도 가짜 depth 변화에 과민 반응하지 않게 해 줍니다. 즉 depth hold가 실제 수심 변화를 더 정확히 보게 됩니다.

**리뷰 메모**

이 함수는 실전적인 품질을 크게 올리는 부분입니다. 센서 장착 위치가 중심에서 벗어난 수중체에서는 특히 의미가 큽니다.

```python
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
```

### Manual 입력 freshness 계열

**의미**

manual heave 입력이 최근 입력인지 판정하고, stale이면 안전하게 0으로 되돌리며 필요하면 새 target depth를 다시 잡습니다.

**영향**

조종기 신호가 끊겼을 때 controller가 마지막 nonzero heave를 계속 믿는 문제를 막습니다.

**리뷰 메모**

이 계층은 현재 코드베이스에서 가장 좋은 패턴 중 하나입니다. attitude/position도 이 freshness 전략을 가져오면 전체 일관성이 더 좋아집니다.

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
```

### `depth_callback()`

**의미**

depth sample이 들어올 때마다 target capture, depth rate 추정, PID 계산, pilot depth-rate, saturation, slew-rate, status publish를 수행합니다.

**영향**

이 함수가 곧 depth controller의 본체입니다. PID와 운용 상태 머신이 모두 여기에서 만납니다.

**리뷰 메모**

구조가 좋고 actuator-friendly합니다. 특히 `depth_rate_alpha`, `max_heave_delta_per_cycle`, `max_upward_heave`가 실제 시스템을 거칠지 않게 만들어 줍니다.

**상세 해설**

이 함수 안에는 사실상 depth controller의 모든 핵심이 들어 있습니다. 먼저 현재 depth를 보정하고, manual heave stale 여부를 해제하고, 필요하면 초기 target을 캡처합니다. 그 다음 depth rate를 샘플 차분으로 계산한 뒤 low-pass filtering을 적용하고, 그 결과를 PID의 derivative 입력으로 사용합니다.

그 이후에는 상황에 따라 분기합니다. disarm이면 무조건 0, control disabled면 0, manual heave + pilot depth-rate 모드면 target depth를 움직이고, 그렇지 않으면 정적인 target depth를 향해 PID를 수행합니다. 마지막으로 saturation, upward limit, slew-rate를 차례로 적용해 heave 명령을 publish합니다. 즉 이 함수는 제어 law와 output shaping이 하나의 파이프라인으로 묶인 구조입니다.

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

### `on_parameter_update()`

**의미**

depth controller의 gain, limit, manual policy, compensation 설정을 런타임에 바꿉니다.

**영향**

실험 중 depth 감각과 안전 정책을 바로 수정할 수 있게 해줍니다.

**리뷰 메모**

runtime tuning 친화적이지만, 파라미터가 많을수록 문서가 중요합니다. 이번 리뷰 사이트에서 이 함수가 중요한 이유도 바로 그 때문입니다.

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
