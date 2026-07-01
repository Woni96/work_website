# Attitude Controller Code Review

대상 파일: `src/rov_control/rov_control/attitude_controller.py`

## 역할
이 노드는 ROV의 자세를 유지하기 위한 제어기입니다.
출력은 `torque.x`, `torque.y`, `torque.z`, 즉 `roll / pitch / yaw` 토크입니다.

즉 이 제어기는 다음을 계산합니다.

\[
(T_x, T_y, T_z)
\]

입력은 주로 IMU이며, 조종기에서 들어오는 수동 wrench도 일부 판단에 사용합니다.

---

## 코드 구조 해설

### 1. 수학 유틸리티 함수
파일 앞부분의 `quat_normalize`, `quat_mul`, `quat_to_rpy`, `rpy_to_quat`, `wrap_to_pi`는
자세를 quaternion과 Euler angle 사이에서 변환하기 위한 함수들입니다.

이 제어기의 핵심은 IMU quaternion을 받아서
- 현재 `roll`
- 현재 `pitch`
- 현재 `yaw`
를 계산하고,
이 값과 목표 자세를 비교하는 것입니다.

즉, 앞부분은 제어기 본체라기보다 **자세 표현을 다루기 위한 도구 함수 집합**입니다.

```python
def imu_callback(self, msg: Imu):
    self.q_current = quat_normalize((
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w,
    ))
    self.roll, self.pitch, self.yaw = quat_to_rpy(*self.q_current)
```

---

### 2. 파라미터 선언부 의미
`__init__()` 초반부는 제어기 설정을 선언합니다.

큰 분류로 보면 다음과 같습니다.

- 토픽/주기
  - `imu_topic`, `manual_wrench_topic`, `output_torque_topic`
  - `control_rate_hz`
- PID 게인
  - `kp_roll`, `ki_roll`, `kd_roll`
  - `kp_pitch`, `ki_pitch`, `kd_pitch`
  - `kp_yaw`, `kd_yaw`
- 출력 제한
  - `tx_limit`, `ty_limit`, `tz_limit`
- 보호/운용 로직
  - `xy_motion_protect_threshold`
  - `rp_scale_when_xy_motion`
  - `yaw_hold_enabled`
  - `large_tilt_disable_deg`

즉 이 부분은 단순 초기화가 아니라,
**이 제어기가 어떤 방식으로 동작할지를 결정하는 운용 정책 선언부**입니다.

```python
self.kp_roll = float(self.declare_parameter('kp_roll', 2.5).value)
self.ki_roll = float(self.declare_parameter('ki_roll', 0.0).value)
self.kd_roll = float(self.declare_parameter('kd_roll', 0.8).value)
self.yaw_hold_enabled = bool(self.declare_parameter('yaw_hold_enabled', True).value)
self.large_tilt_disable_rad = math.radians(
    float(self.declare_parameter('large_tilt_disable_deg', 55.0).value)
)
```

---

### 3. 목표 자세 설정 방식
이 코드는 목표 자세를 두 가지 방식으로 다룹니다.

- 초기 상태를 목표로 캡처
- 외부 명령 `/cmd_attitude`, `/cmd_attitude_trim`으로 목표 갱신

특히 `roll/pitch`는 기본적으로 level target을 유지하도록 설계되어 있고,
`yaw`는 hold mode로 따로 관리합니다.

이 의미는,
- roll/pitch는 “기체를 수평 또는 trim 상태로 유지”하려는 목적이 강하고
- yaw는 “조종자가 돌린 뒤 놓으면 현재 방향을 유지”하는 목적이 강하다는 뜻입니다.

즉, 이 제어기는 `3축 자세제어`이지만,
실제로는 **roll/pitch stabilizer + yaw heading hold**에 가깝습니다.

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

---

### 4. IMU callback의 의미
`imu_callback()`에서는
- quaternion을 받아 현재 자세 계산
- body rate `p, q, r` 저장
- 초기 목표 캡처
를 수행합니다.

이 부분의 의미는 간단합니다.

이 제어기는 매번 IMU를 통해
- 현재 어디를 보고 있는지
- 얼마나 빨리 회전 중인지
를 갱신하고,
그 값으로 다음 제어루프를 돌립니다.

즉 IMU는 이 노드의 핵심 피드백 센서입니다.

```python
self.p_rate = msg.angular_velocity.x
self.q_rate = msg.angular_velocity.y
self.r_rate = msg.angular_velocity.z
self.have_imu = True
if self.capture_initial_target and not self.target_initialized:
    self._reset_control_attitude_filter()
    self._capture_current_attitude_as_target()
```

---

### 5. 제어루프의 핵심 구조
`control_loop()`가 실제 제어기의 중심입니다.

이 루프는 다음 순서로 동작합니다.

1. IMU와 목표 자세가 준비되었는지 확인
2. 제어 비활성 상태면 0 토크 출력
3. 현재 자세 오차 계산
4. 각속도 기반 damping 적용
5. PID 토크 계산
6. 보호 로직으로 토크 스케일 조정
7. saturation 및 slew-rate 제한
8. 최종 토크 publish

즉 이 제어기는 전형적인

`자세 오차 -> 토크 계산 -> 보호 -> 출력`

구조입니다.

```python
def control_loop(self):
    if not self.have_imu or not self.target_initialized:
        return
    ...
    if not self.control_enabled:
        self.pub_torque.publish(out)
        return
    self._update_control_attitude_filter(dt)
    self._force_level_roll_pitch_target()
```

---

## 제어 법칙 해설

### Roll / Pitch
코드에서 roll/pitch는 다음 형태입니다.

\[
T_x = K_{p,r} e_r + K_{i,r} \int e_r dt - K_{d,r} p
\]

\[
T_y = K_{p,p} e_p + K_{i,p} \int e_p dt - K_{d,p} q
\]

즉,
- 자세 오차에 비례하는 복원 토크
- 적분으로 steady-state bias 보상
- 각속도로 damping

을 넣는 구조입니다.

이건 **PID 기반 attitude stabilizer**입니다.

```python
tx_ctrl = (
    self.kp_roll * roll_error +
    self.ki_roll * self.roll_integral -
    self.kd_roll * p_rate_ctrl
)
ty_ctrl = (
    self.kp_pitch * pitch_error +
    self.ki_pitch * self.pitch_integral -
    self.kd_pitch * q_rate_ctrl
)
```

---

### Yaw
yaw는 roll/pitch와 조금 다릅니다.

\[
T_z = K_{p,y} e_y - K_{d,y} r
\]

즉 적분 없이 heading error와 yaw rate damping으로 구성됩니다.

이유는 yaw는 조종 입력과 직접 상호작용하기 때문에,
복잡한 적분기보다 **heading hold + manual override** 구조가 더 안정적이기 때문입니다.

즉 yaw는 **PD형 heading hold controller**입니다.

```python
tz_ctrl = self.kp_yaw * yaw_error - self.kd_yaw * r_rate_ctrl
```

---

## 보호 로직의 의미
이 제어기에서 중요한 부분은 단순 PID가 아니라 보호 로직입니다.

### 1. 큰 기울기 차단
`large_tilt_disable_deg`
- 너무 크게 기울면 토크를 0으로 냅니다.
- 의미: 비정상 자세에서 제어기를 무리하게 돌리지 않기 위한 안전장치

### 2. 고속 회전 시 derivative clamp
- body rate가 크면 derivative 항을 제한합니다.
- 의미: 급격한 회전 중 제어기 폭주 방지

### 3. 주행 중 roll/pitch 약화
- `xy_motion_protect_threshold`
- `rp_scale_when_xy_motion`
- 의미: 조종자가 강하게 이동 중일 때 roll/pitch 자세유지를 완화

즉 이 제어기는 “언제나 강하게 자세를 잡는” 제어기가 아니라,
**운용 상태에 따라 자세유지 강도를 조절하는 gain scheduling형 제어기**입니다.

```python
if (
    abs(self.control_roll) > self.large_tilt_disable_rad or
    abs(self.control_pitch) > self.large_tilt_disable_rad
):
    self.pub_torque.publish(out)
    return

rate_limit = max(0.0, self.max_body_rate_for_control)
if rate_limit > 1e-6:
    p_rate_ctrl = clamp(self.p_rate, -rate_limit, rate_limit)
    q_rate_ctrl = clamp(self.q_rate, -rate_limit, rate_limit)
    r_rate_ctrl = clamp(self.r_rate, -rate_limit, rate_limit)
```

---

## Yaw hold 로직의 의미
이 코드의 yaw 부분은 운용성 중심으로 설계되어 있습니다.

- yaw stick이 들어오면 manual yaw 우선
- yaw stick을 놓으면 현재 yaw를 target으로 다시 잡음
- 짧은 settle 구간 동안 yaw rate만 감쇠

이 구조는 실제로 조종자가 느끼기에
- 돌릴 때는 잘 돌고
- 놓으면 현재 방향을 자연스럽게 유지하는
느낌을 주기 위한 것입니다.

즉 이 코드는 단순한 자세제어기라기보다,
**ROV용 heading hold 동작을 구현한 실사용형 attitude controller**입니다.

```python
yaw_manual_active = abs(tz_manual) > self.yaw_manual_override_threshold
yaw_released = self.prev_yaw_manual_active and not yaw_manual_active

if yaw_manual_active:
    self.last_manual_yaw_input_time = now
    self._capture_current_yaw_as_target()
elif yaw_released and self.capture_yaw_target_on_release:
    self.last_manual_yaw_input_time = now
    self._capture_current_yaw_as_target()
```

---

## 출력의 의미
최종 출력은 `geometry_msgs/Wrench`이지만,
실제로 이 노드가 쓰는 것은 `torque.x/y/z` 뿐입니다.

즉 이 노드는 직접 스러스터를 제어하지 않고,
“기체에 이 정도 회전 토크가 필요하다”는 **상위 제어 명령**만 만듭니다.

이 출력은 이후 `wrench_merger`와 `allocator_node`를 거쳐 스러스터 명령이 됩니다.

---

## 한줄 정리
이 코드는

**IMU 자세를 기반으로 roll/pitch는 PID stabilizer, yaw는 heading hold PD로 제어하고, 운용 상황(주행, heave, manual yaw)에 따라 토크를 조절하는 실사용형 자세제어기**입니다.
