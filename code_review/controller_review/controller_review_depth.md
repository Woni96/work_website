# Depth Controller Code Review

대상 파일: `src/rov_control/rov_control/depth_controller.py`

## 역할
이 노드는 ROV의 수심을 유지하기 위한 제어기입니다.
출력은 `heave force`, 즉 `Fz` 방향 명령입니다.

즉 이 노드는 다음을 계산합니다.

```text
Fz
```

입력은 수심 센서와 IMU, 그리고 수동 heave 입력입니다.

---

## 코드 구조 해설

### 1. 파라미터 선언부
이 부분은 크게 네 그룹입니다.

- 토픽
  - `depth_topic`, `imu_topic`, `cmd_depth_topic`
- PID 게인
  - `kp_depth`, `ki_depth`, `kd_depth`
- 출력 제한
  - `max_heave`, `max_upward_heave`
- 운용 로직
  - `pilot_depth_rate_enabled`
  - `manual_heave_override_threshold`
  - `capture_initial_depth_target`
  - `depth_sensor_offset_compensation_enabled`

즉 이 제어기는 단순 수심 PID만 있는 게 아니라,
조종 입력과 센서 장착 위치까지 고려한 depth hold 시스템입니다.

```python
self.kp_depth = float(self.declare_parameter('kp_depth', 2.0).value)
self.ki_depth = float(self.declare_parameter('ki_depth', 0.0).value)
self.kd_depth = float(self.declare_parameter('kd_depth', 1.0).value)
self.pilot_depth_rate_enabled = bool(
    self.declare_parameter('pilot_depth_rate_enabled', True).value
)
```

---

### 2. 수심 목표(target depth) 관리
이 노드에서 중요한 상태는 `target_depth`입니다.

이 목표는 다음 경우에 설정됩니다.

- 시작 시 현재 수심을 캡처
- arm 순간 현재 수심 캡처
- `/cmd_depth` 명령 수신
- 수동 heave 해제 시 현재 수심 근처로 재설정

즉 이 노드는 고정된 목표 수심만 보는 게 아니라,
운용 이벤트에 따라 hold target을 계속 갱신하는 구조입니다.

```python
def armed_callback(self, msg: Bool):
    self.prev_armed = self.armed
    self.armed = bool(msg.data)
    self.armed_received = True
    if (not self.prev_armed) and self.armed:
        if self.current_depth is not None:
            self.target_depth = self.current_depth
            self.target_initialized = True
            self.prev_heave_cmd = 0.0
            self.depth_integral = 0.0

def capture_manual_release_target(self, reason: str):
    self.target_depth = self.current_depth + self.manual_heave_release_target_offset
    self.clamp_target_depth()
    self.target_initialized = True
    self.depth_integral = 0.0
```

---

### 3. IMU의 역할
이 제어기에서 IMU는 자세제어처럼 직접 PID에는 안 들어갑니다.
대신 depth sensor offset compensation에 사용됩니다.

의미:
- depth 센서가 기체 중심이 아니라 앞/뒤/옆에 달려 있으면,
- 기체가 pitch/roll될 때 센서의 실제 world z 위치가 변합니다.
- 이걸 보정하지 않으면 가짜 수심 변화가 생깁니다.

즉 이 코드는 센서 장착 위치로 인한 기하학적 오차를 IMU로 보정합니다.

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

---

### 4. 수동 heave 입력 처리
이 제어기는 수동 heave를 두 방식으로 봅니다.

- depth hold 비활성 또는 예외 상황: 직접 수동 입력으로 간주
- depth hold 활성 + `pilot_depth_rate_enabled=True`: 목표 수심 변화율 명령으로 해석

즉 조종자가 heave stick을 움직이면,
즉시 추진기를 직접 올리는 것이 아니라

```text
z_target_dot
```

를 바꾸는 방식입니다.

```python
def manual_wrench_callback(self, msg: Wrench):
    self.manual_heave = msg.force.z
    self.last_manual_wrench_time = self.get_clock().now()
    self.prev_manual_heave_active = self.manual_heave_active
    self.manual_heave_active = abs(self.manual_heave) > self.manual_heave_override_threshold
    if self.prev_manual_heave_active and not self.manual_heave_active:
        if self.current_depth is not None and self.armed:
            self.capture_manual_release_target('Manual heave released')
```

---

## 제어 법칙 해설

핵심 제어 식은 다음과 같습니다.

```text
e_z = z_target - z
Fz  = Kp * e_z + Ki * ∫e_z dt - Kd * z_dot
```

그리고 코드에서는 여기에
- 출력 부호(`heave_cmd_sign`)
- 전체 출력 제한
- 상승 방향 제한
- slew-rate 제한
을 적용합니다.

즉 이 코드는 본질적으로 depth PID controller입니다.

```python
error = self.target_depth - depth
raw_cmd = (
    self.kp_depth * error +
    self.ki_depth * self.depth_integral -
    self.kd_depth * depth_rate
)
heave_cmd = self.heave_cmd_sign * raw_cmd
```

---

## 필터의 의미
수심 속도는 직접 센서에서 오는 게 아니라,
수심 샘플 차분으로 계산한 뒤 low-pass filter를 적용합니다.

```text
z_dot_filtered = α * z_dot_raw + (1-α) * z_dot_prev
```

의미:
- depth derivative는 노이즈에 민감함
- 그대로 쓰면 `kd_depth`가 거칠게 반응함
- 그래서 filter를 통해 더 안정적인 depth rate를 만듦

```python
raw_depth_rate = (depth - self.prev_depth) / dt
self.filtered_depth_rate = (
    self.depth_rate_alpha * raw_depth_rate +
    (1.0 - self.depth_rate_alpha) * self.filtered_depth_rate
)
depth_rate = self.filtered_depth_rate
```

---

## saturation의 의미
이 코드에는 여러 단계의 제한이 있습니다.

- `max_heave`
- `max_upward_heave`
- `max_heave_delta_per_cycle`

즉 제어기라기보다 actuator-friendly shaping도 같이 수행합니다.

```python
if heave_cmd > self.max_heave:
    heave_cmd = self.max_heave
elif heave_cmd < -self.max_heave:
    heave_cmd = -self.max_heave

delta = heave_cmd - self.prev_heave_cmd
if delta > self.max_heave_delta_per_cycle:
    heave_cmd = self.prev_heave_cmd + self.max_heave_delta_per_cycle
elif delta < -self.max_heave_delta_per_cycle:
    heave_cmd = self.prev_heave_cmd - self.max_heave_delta_per_cycle
```

---

## 이 코드가 말하는 설계 철학
이 제어기는 단순한 현재 깊이로 돌아가라 PID가 아닙니다.

- 기본은 depth hold
- 수동 heave 입력은 target depth rate로 해석
- 입력을 놓으면 그 자리에서 다시 hold
- 센서 위치 오차는 IMU로 보정
- 출력은 제한과 smoothing을 거침

즉 실무적으로는 pilot-assisted depth hold controller라고 보는 게 가장 정확합니다.

---

## 출력의 의미
최종 출력은 `Float64` heave command입니다.
이 값은 단독으로 스러스터로 가지 않고,
`wrench_merger`에서 최종 `force.z`의 일부로 합쳐집니다.

즉 이 제어기는 수심을 유지하기 위해 필요한 수직 힘만 계산하는 상위 제어기입니다.

---

## 한줄 정리
이 코드는

수심 오차에 대한 PID 제어를 기반으로 하되, 조종기 heave 입력을 목표 수심 변화율로 해석하고,
센서 위치 보정과 출력 제한까지 포함한 실사용형 depth hold 제어기입니다.
