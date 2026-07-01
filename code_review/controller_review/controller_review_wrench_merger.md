# Wrench Merger Code Review

대상 파일: `src/rov_control/rov_control/wrench_merger.py`

## 역할
이 노드는 여러 제어기 출력을 하나의 최종 `Wrench`로 합치는 노드입니다.

즉 이 노드는 직접 제어기를 구현한다기보다,

- 수동 조종 입력
- 위치 제어 출력
- 수심 제어 출력
- 자세 제어 출력

을 축별 규칙에 따라 합쳐서
최종 명령

```text
(Fx, Fy, Fz, Tx, Ty, Tz)
```

를 만듭니다.

즉 구조상 supervisory mixer 또는 command arbitration layer라고 볼 수 있습니다.

---

## 코드 구조 해설

### 1. 입력 소스
이 노드는 다음 입력을 받습니다.

- `manual_wrench_topic`
- `depth_heave_topic`
- `depth_active_topic`
- `position_force_topic`
- `attitude_torque_topic`
- `armed_topic`

이 말은 곧,
이 노드가 전체 제어 아키텍처에서 최종 명령 결정자라는 뜻입니다.

```python
self.create_subscription(Wrench, self.manual_wrench_topic, self.manual_wrench_callback, 10)
self.create_subscription(Float64, self.depth_heave_topic, self.depth_heave_callback, 10)
self.create_subscription(Bool, self.depth_active_topic, self.depth_active_callback, 10)
self.create_subscription(Wrench, self.position_force_topic, self.position_force_callback, 10)
self.create_subscription(Wrench, self.attitude_torque_topic, self.attitude_torque_callback, 10)
```

---

### 2. 내부 상태의 의미
이 노드는 각 입력을 마지막 값으로 저장합니다.

- `last_manual_wrench`
- `last_depth_heave`
- `last_position_force`
- `last_attitude_torque`
- `depth_active`
- `armed`

즉 설계 철학은
각 제어기의 최신 출력을 기억해두고, publish 주기에 맞춰 합쳐서 보낸다
입니다.

```python
def manual_wrench_callback(self, msg: Wrench):
    self.last_manual_wrench = msg
    self.last_manual_wrench_time = self.get_clock().now()
    self.manual_received = True
```

---

## 병합 규칙 해설

### 1. `Fx`, `Fy`
수평 힘은 기본적으로
- 수동 입력이 크면 manual 우선
- 아니면 position controller 출력 사용

즉,

```text
if |manual| > threshold:
    use manual
else:
    use position controller
```

의미:
- 위치 hold는 조종자가 실제로 조작하지 않을 때만 동작
- 조종자가 움직이면 수동 조작권 우선

```python
if abs(manual_surge) > self.manual_xy_override_threshold:
    merged.force.x = manual_surge
else:
    merged.force.x = self.last_position_force.force.x

if abs(manual_sway) > self.manual_xy_override_threshold:
    merged.force.y = manual_sway
else:
    merged.force.y = self.last_position_force.force.y
```

---

### 2. `Fz`
수직 힘은 조금 다르게 동작합니다.

- depth hold 활성 시: 자동 depth 출력 사용
- manual heave가 충분히 크면: manual 우선
- 그 외에는 자동 depth + position 보상 사용

즉 `Fz`는 단순 override가 아니라,
depth hold 상태 머신을 반영한 병합입니다.

```python
if self.depth_active:
    merged.force.z = self.last_depth_heave + self.last_position_force.force.z
    heave_mode = 'AUTO_DEPTH_HOLD'
elif abs(manual_heave) > self.manual_heave_override_threshold:
    merged.force.z = manual_heave
    heave_mode = 'MANUAL_HEAVE_OVERRIDE'
else:
    merged.force.z = self.last_depth_heave + self.last_position_force.force.z
    heave_mode = 'AUTO_DEPTH_PLUS_POSITION_COMP'
```

---

### 3. `Tx`, `Ty`
roll/pitch 토크는 attitude controller 출력을 그대로 사용합니다.

즉 이 두 축은 자동 자세 유지가 우선되는 구조입니다.

```python
merged.torque.x = self.last_attitude_torque.torque.x
merged.torque.y = self.last_attitude_torque.torque.y
```

---

### 4. `Tz`
yaw는 예외입니다.

- manual yaw 입력이 threshold를 넘으면 수동 yaw 우선
- 아니면 attitude controller의 yaw hold 출력 사용

즉 yaw는 자동 hold + 수동 override 구조입니다.

```python
manual_yaw = manual_wrench.torque.z
if abs(manual_yaw) > self.manual_yaw_override_threshold:
    merged.torque.z = manual_yaw
else:
    merged.torque.z = self.last_attitude_torque.torque.z
```

---

## arm/disarm 로직의 의미
이 노드는 최종 출력단 앞에 있기 때문에,
안전 측면에서 매우 중요한 역할을 합니다.

- arm 상태를 아직 못 받았으면 0 출력
- disarm이면 무조건 0 출력

즉 모든 자동제어가 살아 있어도,
최종적으로 이 노드가 출력을 막아버릴 수 있습니다.

이건 구조상 last safety gate에 해당합니다.

```python
if not self.armed_received:
    self.publish_zero_wrench()
    return
if not self.armed:
    self.publish_zero_wrench()
    return
```

---

## publish loop의 의미
이 노드는 각 입력 callback에서 바로 최종 wrench를 내지 않고,
타이머 기반 `publish_merged_wrench()`에서 주기적으로 보냅니다.

의미:
- 입력 주기가 달라도 최종 명령 주기를 일정하게 맞춤
- 각 제어기 출력을 동기화된 형태로 합성
- actuator 입력을 더 안정적으로 전달

즉 이 노드는 단순 merge가 아니라,
command synchronization layer 역할도 합니다.

```python
self.timer = self.create_timer(1.0 / self.output_rate_hz, self.publish_merged_wrench)
```

---

## 시스템 내 위치
전체 제어 구조에서 이 노드는 다음 위치에 있습니다.

```text
Position Controller
Depth Controller
Attitude Controller
Manual Teleop
    -> Wrench Merger
    -> Allocator
    -> Thruster
```

즉 상위 제어기와 allocator 사이의 중간 관리자입니다.

---

## 한줄 정리
이 코드는

수동 조종과 위치/수심/자세 제어 출력을 축별 우선순위 규칙에 따라 합성하여,
최종 ROV wrench 명령을 만드는 supervisory command merger입니다.
