# Position Controller Code Review

대상 파일: `src/rov_control/rov_control/position_controller.py`

## 역할
이 노드는 ROV의 평면 위치를 유지하기 위한 제어기입니다.
출력은 `Fx`, `Fy` 중심의 힘 명령입니다.

즉 이 노드는 다음을 계산합니다.

\[
F_x, F_y
\]

입력은 DVL 속도/위치와 IMU입니다.

---

## 코드 구조 해설

### 1. 이 제어기가 보는 좌표계
이 노드는 기본적으로 world 기준 위치를 보고 제어합니다.

- DVL 위치가 있으면 그것을 사용
- 위치가 없으면 DVL 속도를 적분해 위치를 추정
- 제어 계산은 world frame에서 수행
- 최종 출력은 다시 body frame으로 회전

즉 구조는

`world frame에서 위치 오차 계산 -> body frame 힘으로 변환`

입니다.

이 설계는 매우 중요합니다.
왜냐하면 기체가 yaw 되어 있어도,
ROV가 “지구 기준 같은 자리”를 유지하려면 world frame 오차로 계산해야 하기 때문입니다.

```python
force_world = (force_world_x, force_world_y, 0.0)
force_body = quat_rotate_vector(quat_conjugate(self.q_world_body), force_world)
force_body_x = force_body[0]
force_body_y = force_body[1]
```

---

### 2. 파라미터 선언부 의미
주요 그룹은 다음과 같습니다.

- 토픽
  - `dvl_topic`, `dvl_position_topic`, `imu_topic`
- 제어 게인
  - `kp_x`, `kd_x`, `kp_y`, `kd_y`
- 출력 제한
  - `max_force_x`, `max_force_y`, `max_force_z`
- 운용 정책
  - `capture_initial_position_target`
  - `capture_target_on_manual_release`
  - `manual_xy_override_threshold`
  - `valid_timeout_sec`
- DVL 장착 보정
  - `dvl_mount_roll_deg`, `dvl_mount_pitch_deg`, `dvl_mount_yaw_deg`

즉 이 코드는 단순한 위치 PD 제어기이면서,
**센서 유효성, 수동 override, 장착 각도 보정까지 포함한 운영형 hold 제어기**입니다.

```python
self.kp_x = float(self.declare_parameter('kp_x', 1.2).value)
self.kd_x = float(self.declare_parameter('kd_x', 1.6).value)
self.valid_timeout_sec = float(self.declare_parameter('valid_timeout_sec', 0.5).value)
self.capture_target_on_manual_release = bool(
    self.declare_parameter('capture_target_on_manual_release', True).value
)
```

---

### 3. DVL 처리 방식
이 제어기의 핵심은 DVL 처리입니다.

두 모드가 있습니다.

#### (1) DVL position 사용
- `use_dvl_position=True`
- 직접 `x, y` 위치를 사용

#### (2) DVL velocity 적분
- `use_dvl_position=False`
- 속도를 적분해서 위치 추정

즉 이 제어기는
**센서 구성에 따라 absolute position hold 또는 velocity-integrated hold** 둘 다 지원합니다.

```python
def dvl_position_callback(self, msg: PointStamped):
    ...
    if self.use_dvl_position and self._dvl_position_is_finite(msg):
        self.position_x = float(msg.point.x)
        self.position_y = float(msg.point.y)
        self.active_position_frame_id = msg.header.frame_id or self.hold_frame_id
        self.last_valid_dvl_time = now
        if self.capture_initial_position_target and not self.target_initialized:
            self._capture_current_position_as_target()

def dvl_callback(self, msg: DVLData):
    ...
    if msg.velocity_valid and self.have_imu:
        v_dvl = (float(msg.vx), float(msg.vy), float(msg.vz))
        v_body = quat_rotate_vector(self.q_body_dvl, v_dvl)
        v_world = quat_rotate_vector(self.q_world_body, v_body)
        self.velocity_world_x = v_world[0]
        self.velocity_world_y = v_world[1]
```

---

### 4. IMU의 역할
IMU는 여기서 자세제어가 아니라 좌표 변환에 사용됩니다.

- DVL 속도를 body->world 변환
- world에서 계산한 힘을 다시 world->body 변환

즉 IMU는 “기체가 어느 방향을 보고 있는지”를 알려주는 센서입니다.

이 노드는 그 정보를 이용해서
- world frame에서 위치 유지
- body frame에서 힘 출력
을 연결합니다.

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

---

## 제어 법칙 해설

핵심 식은 다음과 같습니다.

\[
e_x = x_{target} - x
\]
\[
e_y = y_{target} - y
\]

\[
F_{world,x} = K_{p,x} e_x - K_{d,x} \dot{x}
\]
\[
F_{world,y} = K_{p,y} e_y - K_{d,y} \dot{y}
\]

즉 위치 오차에 비례하는 복원력과 속도에 비례하는 damping을 더한 구조입니다.

이건 전형적인 **planar PD position hold controller**입니다.

```python
error_x = self.target_x - self.position_x
error_y = self.target_y - self.position_y
force_world_x = self.kp_x * error_x - effective_kd_x * self.velocity_world_x
force_world_y = self.kp_y * error_y - effective_kd_y * self.velocity_world_y
```

---

### 추가 damping
이 코드의 특징은 yaw motion에 따른 damping 증가입니다.

`extra_damping = yaw_rate_damping_gain * abs(current_yaw_rate)`

의미:
- yaw 회전이 클 때는 위치 hold가 불안정해지기 쉬움
- 그래서 derivative gain을 일시적으로 더 키워서 흔들림을 줄임

또 manual yaw가 들어오면 damping boost를 추가합니다.

즉 이 제어기는 단순 PD가 아니라,
**yaw motion-aware PD controller**입니다.

```python
extra_damping = self.yaw_rate_damping_gain * abs(self.current_yaw_rate)
if abs(self.manual_wrench.torque.z) > self.manual_yaw_override_threshold:
    extra_damping += self.manual_yaw_damping_boost

effective_kd_x = self.kd_x + extra_damping
effective_kd_y = self.kd_y + extra_damping
```

---

### world -> body 변환의 의미
위치 오차는 world에서 계산하지만,
실제 추진기는 body 기준으로 힘을 내야 하므로
출력 전에 body frame으로 회전합니다.

즉,

\[
F_{body} = R^T F_{world}
\]

형태입니다.

이게 없으면 기체 yaw가 바뀌었을 때 위치 hold가 어긋납니다.

```python
force_world = (force_world_x, force_world_y, 0.0)
force_body = quat_rotate_vector(quat_conjugate(self.q_world_body), force_world)
force_body_x = force_body[0]
force_body_y = force_body[1]
```

---

## target capture 로직의 의미
이 코드에서 위치 target은 고정 상수가 아닙니다.

다음 상황에서 새로 캡처됩니다.

- 초기 시작 시
- arm 순간
- 수동 XY를 놓았을 때

의미:
- 조종 중에는 위치 hold를 끄고
- 조종을 놓는 순간 “그 자리”를 새로운 hold target으로 삼음

즉 이 구조는 조종성과 자동 hold를 자연스럽게 섞기 위한 것입니다.

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
```

---

## 출력의 의미
최종 출력은 `Wrench`지만,
실제로는 `force.x`, `force.y`, 일부 경우 `force.z`만 사용합니다.

`torque`는 모두 0입니다.

즉 이 노드는 회전 제어를 하지 않고,
**평면 위치를 유지하기 위해 필요한 힘 벡터만 계산하는 제어기**입니다.

이 출력은 나중에 `wrench_merger`에서 다른 제어기 출력과 합쳐집니다.

---

## 설계 관점 요약
이 코드는
- world frame에서 위치 오차를 계산하고
- DVL/IMU를 통해 속도와 좌표계를 보정하고
- PD 제어로 복원력을 만든 뒤
- body frame 힘으로 바꿔서 출력하는 구조입니다.

즉 가장 적절한 표현은

**world-frame planar hold PD controller with body-frame output mapping**

입니다.

---

## 한줄 정리
이 코드는

**DVL와 IMU를 이용해 world 기준 위치 오차를 계산하고, 이를 PD 형태의 평면 복원력으로 만든 뒤, body frame 힘 명령으로 변환하여 출력하는 위치 hold 제어기**입니다.
