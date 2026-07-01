# Allocator Code Review

대상 파일: `src/rov_control/rov_control/allocator_node.py`

## 역할
이 노드는 최종 `Wrench(Fx, Fy, Fz, Tx, Ty, Tz)`를 받아서, 실제 8개 스러스터 명령으로 바꾸는 노드입니다.

즉 이 노드는 다음 변환을 수행합니다.

```text
τ_cmd = [Fx, Fy, Fz, Tx, Ty, Tz]^T
   ->
u     = [u1, u2, u3, u4, u5, u6, u7, u8]^T
```

즉 이 노드는 제어기라기보다
`control allocation + actuator shaping` 노드입니다.

---

## 코드 구조 해설

### 1. `signs` 배열의 의미
`self.signs`는 스러스터 방향 부호를 정의합니다.

즉 allocator가 계산한 추력 방향과 실제 하드웨어/플러그인의 방향이 다를 경우,
이 배열로 맞추겠다는 의미입니다.

이는 제어 수식보다는 하드웨어 인터페이스 계층에 가까운 부분입니다.

```python
self.signs = np.array([1, 1, 1, 1, 1, 1, 1, 1], dtype=float)
...
u = u * self.signs
```

---

### 2. 파라미터 그룹
이 노드 파라미터는 크게 다음 목적을 가집니다.

- 토픽 설정
- scaling / gain
- output shaping
- compensation
- allocation priority

즉 allocator는 단순 pseudo-inverse 계산만 하는 것이 아니라,
실제 기체 특성과 운용 목적을 반영하도록 설계되어 있습니다.

```python
self.heave_gain = float(self.declare_parameter('heave_gain', 1.0).value)
self.roll_torque_gain = float(self.declare_parameter('roll_torque_gain', 1.0).value)
self.pitch_torque_gain = float(self.declare_parameter('pitch_torque_gain', 1.0).value)
self.yaw_torque_gain = float(self.declare_parameter('yaw_torque_gain', 1.0).value)
self.torque_first_allocation = bool(self.declare_parameter('torque_first_allocation', True).value)
self.level_horizontal_compensation_enabled = bool(
    self.declare_parameter('level_horizontal_compensation_enabled', False).value
)
```

---

## 핵심 설계 철학

### 1. 수평/수직 스러스터 그룹 분리
이 코드의 핵심은 8개 스러스터를 두 그룹으로 나누는 것입니다.

#### 수평 그룹 1~4번
담당:
- `Fx`
- `Fy`
- `Tz`

즉,

```text
τ_H = [Fx, Fy, Tz]^T
```

#### 수직 그룹 5~8번
담당:
- `Fz`
- `Tx`
- `Ty`

즉,

```text
τ_V = [Fz, Tx, Ty]^T
```

이 구조는 실제 ROV의 스러스터 배치와 물리적 역할 분리를 반영합니다.

```python
def init_matrices(self):
    thrusters = [
        ([0.20, -0.13, 0.0], [0.7071, -0.7071, 0.0]),
        ([0.20,  0.13, 0.0], [0.7071,  0.7071, 0.0]),
        ([-0.20, -0.13, 0.0], [-0.7071, -0.7071, 0.0]),
        ([-0.20,  0.13, 0.0], [-0.7071,  0.7071, 0.0]),
        ([0.20, -0.27, 0.0], [0.0, 0.0, 1.0]),
        ([0.20,  0.27, 0.0], [0.0, 0.0, 1.0]),
        ([-0.20, -0.27, 0.0], [0.0, 0.0, 1.0]),
        ([-0.20,  0.27, 0.0], [0.0, 0.0, 1.0]),
    ]
    self.TAM = np.zeros((6, 8), dtype=float)
    for i, (pos, direc) in enumerate(thrusters):
        p = np.array(pos, dtype=float)
        d = normalize(direc)
        self.TAM[0:3, i] = d
        self.TAM[3:6, i] = np.cross(p, d)
    self.H = np.array([self.TAM[0, 0:4], self.TAM[1, 0:4], self.TAM[5, 0:4]], dtype=float)
    self.H_pinv = np.linalg.pinv(self.H)
    self.V = np.array([self.TAM[2, 4:8], self.TAM[3, 4:8], self.TAM[4, 4:8]], dtype=float)
    self.V_pinv = np.linalg.pinv(self.V)
```

---

### 2. 수평 allocation
코드에서는 수평 행렬 `H`와 pseudo-inverse `H_pinv`를 사용합니다.

관계는 다음과 같습니다.

```text
τ_H = H u_H
u_H = H⁺ τ_H
```

실제로는 `Fx/Fy` 성분과 `Tz` 성분을 나눠 계산한 뒤 더합니다.

의미:
- 전후/좌우 이동과 yaw 회전을 수평 스러스터 4개가 분담
- yaw gain은 별도로 조절 가능

즉 수평 그룹은 평면 운동 전용 allocator입니다.

```python
horiz_force_cmd = np.array([fx, fy, 0.0], dtype=float)
horiz_yaw_cmd = np.array([0.0, 0.0, tz_scaled], dtype=float)
u_h = (
    (self.H_pinv @ horiz_force_cmd) * self.horizontal_output_gain +
    (self.H_pinv @ horiz_yaw_cmd) * self.yaw_output_gain
)
```

---

### 3. 수직 allocation
수직 그룹은 `V`와 `V_pinv`를 사용합니다.

```text
τ_V = V u_V
u_V = V⁺ τ_V
```

수직 그룹은
- heave
- roll
- pitch
를 담당합니다.

즉 수직 그룹은 depth + attitude recovery 전용 allocator입니다.

```python
vert_heave_cmd = np.array([fz_scaled, 0.0, 0.0], dtype=float)
vert_torque_cmd = np.array([0.0, tx_scaled, ty_scaled], dtype=float)

u_v_heave = (self.V_pinv @ vert_heave_cmd) * self.vertical_output_gain
u_v_torque = (self.V_pinv @ vert_torque_cmd) * self.vertical_output_gain
```

---

## 우선순위 로직의 의미

### `torque_first_allocation`
이 옵션이 켜져 있으면,
수직 그룹에서는

1. 먼저 `Tx`, `Ty`를 만족시키고
2. 남는 여유로 `Fz`를 넣습니다.

즉 allocator 수준에서 이미 stability priority가 반영되어 있습니다.

```python
if self.torque_first_allocation:
    components = [u_v_torque, u_v_heave]
else:
    components = [u_v_heave, u_v_torque]

u_v = self.allocate_priority_components(components)
```

---

## 보상 항 해설

### 1. `level_horizontal_heave_compensation`
기체가 pitch/roll된 상태에서 수평 이동을 하면,
실제 world 기준으로는 상하 성분이 섞입니다.

이걸 보정하기 위해 heave 쪽 보상 항을 추가합니다.

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

### 2. `surge_pitch_moment_compensation`
전진 추진이 만드는 pitch 모멘트를 미리 보상합니다.

### 3. `imu_pitch_hold_compensation`
실제 IMU pitch를 보고 추가 보상을 넣습니다.

```python
def surge_pitch_moment_compensation(self, fx: float) -> float:
    if not self.surge_pitch_moment_compensation_enabled:
        return 0.0
    if abs(float(fx)) < max(0.0, float(self.surge_pitch_moment_min_surge)):
        return 0.0
    ...

def imu_pitch_hold_compensation(self, fx: float) -> float:
    if not self.imu_pitch_hold_compensation_enabled or not self.have_imu:
        return 0.0
    ...
    pitch_error = self.target_pitch - self.current_pitch
    ...
```

### 4. `rear_vertical_bias`
후방 수직 스러스터에 bias를 주어 trim을 조정합니다.

즉 allocator는 단순한 힘 분배기라기보다,
기체의 known coupling을 일부 선보상하는 노드입니다.

---

## 출력 shaping
이 노드는 계산 후 바로 끝나지 않습니다.

- group normalize
- deadband
- overall output scaling
- slew rate

를 적용합니다.

즉 allocator는 최적화 계산기이면서 동시에 actuator conditioner 역할도 합니다.

```python
def apply_slew_rate(self, target):
    max_delta = np.full_like(target, self.max_thrust_delta_per_cycle, dtype=float)
    delta = target - self.prev_thrust
    delta = np.clip(delta, -max_delta, max_delta)
    out = self.prev_thrust + delta
    self.prev_thrust = out.copy()
    return out

u = self.apply_group_normalization(u)
u = self.apply_deadband(u)
u = u * self.output_scale
u = self.apply_slew_rate(u)
```

---

## 시스템 내 위치
전체 제어 구조에서 이 노드는 마지막 상위 단계입니다.

```text
Controllers
-> Merged Wrench
-> Allocator
-> Thruster Command
-> CAN / Simulator
```

즉 이 노드는 고수준 힘/토크 명령을 실제 추진기 명령으로 번역하는 마지막 소프트웨어 계층입니다.

---

## 한줄 정리
이 코드는

고수준 6축 wrench 명령을 수평/수직 스러스터 그룹으로 나누어 pseudo-inverse 기반으로 분배하고,
여기에 기체 보상·우선순위·출력 shaping을 더해 최종 스러스터 명령으로 변환하는 control allocation 노드입니다.
