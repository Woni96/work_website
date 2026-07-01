# Control Code Review

이 문서는 제어 관련 코드만 대상으로 한 리뷰 메모입니다.

검토 범위:
- `src/rov_control/rov_control/attitude_controller.py`
- `src/rov_control/rov_control/depth_controller.py`
- `src/rov_control/rov_control/position_controller.py`
- `src/rov_control/rov_control/wrench_merger.py`
- `src/rov_control/rov_control/allocator_node.py`

기준:
- 런타임 동작 안정성
- 조종 입력/센서 입력 손실 시 안전성
- 제어 구조 일관성
- 유지보수성과 튜닝 리스크

---

## Summary

전체 구조 자체는 명확합니다.

- 상위 제어기들이 `Wrench(Fx, Fy, Fz, Tx, Ty, Tz)`를 생성하고
- `wrench_merger`가 수동/자동 출력을 병합한 뒤
- `allocator_node`가 각 스러스터 명령으로 분배하는 구조입니다.

구조적으로는 괜찮지만, **입력 freshness 관리가 제어기마다 일관되지 않아서 입력 dropout 시 제어 상태가 latch될 수 있는 문제**가 가장 크게 보입니다.

---

## Findings

### 1. `attitude_controller`가 수동 입력 freshness를 관리하지 않음
- Severity: High
- File: `src/rov_control/rov_control/attitude_controller.py:572`
- File: `src/rov_control/rov_control/attitude_controller.py:610`
- File: `src/rov_control/rov_control/attitude_controller.py:674`

**문제**
- `manual_wrench`는 마지막으로 받은 값을 계속 유지합니다.
- 이 노드는 `manual_wrench_timeout_sec` 같은 개념이 없습니다.
- 따라서 조이스틱/teleop 노드가 마지막 nonzero 값을 보낸 뒤 멈추면,
  - `yaw_manual_active`
  - `heave_protect`
  - `xy_motion_protect`
  - `roll/pitch soft trim scaling`
  상태가 계속 유지될 수 있습니다.

**영향**
- 실제 조종 입력은 끊겼는데도 자세제어가 약화된 상태로 남을 수 있습니다.
- yaw hold가 다시 잡혀야 할 타이밍에 계속 manual mode로 남을 수 있습니다.
- 운용 중 “왜 자세제어가 계속 약하지?” 같은 형태의 간헐 문제를 만들 가능성이 큽니다.

**권장사항**
- `wrench_merger`와 같은 방식으로 수동 입력 timestamp를 저장하고 freshness timeout을 적용해야 합니다.
- stale이면 `manual_wrench = Wrench()`로 취급하는 것이 안전합니다.

---

### 2. `position_controller`도 수동 XY 입력이 stale 되어 latch될 수 있음
- Severity: High
- File: `src/rov_control/rov_control/position_controller.py:261`
- File: `src/rov_control/rov_control/position_controller.py:264`
- File: `src/rov_control/rov_control/position_controller.py:417`

**문제**
- `position_controller`는 `manual_xy_active`를 마지막 `manual_wrench` 값으로만 판정합니다.
- freshness timeout이 없습니다.
- 마지막 XY 입력이 nonzero인 상태에서 teleop가 멈추면 `manual_xy_active=True`가 유지될 수 있습니다.

**영향**
- 위치제어 출력이 계속 0으로 눌린 채 유지될 수 있습니다.
- 수동 조작을 이미 놓았는데 position hold가 복귀하지 않는 현상이 생길 수 있습니다.
- release edge 기반 target recapture도 새 메시지가 안 오면 동작하지 않습니다.

**권장사항**
- `last_manual_wrench_time`와 timeout을 추가해서 stale input 해제 로직을 넣는 것이 필요합니다.
- timeout 시 `manual_xy_active=False`로 복구하고, 필요하면 hold target 재캡처도 함께 처리하는 편이 좋습니다.

---

### 3. `wrench_merger`는 자동제어 입력의 freshness를 확인하지 않아 마지막 자동 명령을 계속 재사용함
- Severity: High
- File: `src/rov_control/rov_control/wrench_merger.py:221`
- File: `src/rov_control/rov_control/wrench_merger.py:231`
- File: `src/rov_control/rov_control/wrench_merger.py:243`
- File: `src/rov_control/rov_control/wrench_merger.py:253`

**문제**
- `manual_wrench`만 freshness timeout이 있습니다.
- 반면 아래 자동 입력들은 freshness 체크가 없습니다.
  - `last_position_force`
  - `last_depth_heave`
  - `last_attitude_torque`
  - `depth_active`
- 상위 제어기 노드가 죽거나 publish가 멈춰도 마지막 값을 계속 사용합니다.

**영향**
- 예를 들어 `attitude_controller`가 멈추면 마지막 `Tx/Ty/Tz`가 계속 나갈 수 있습니다.
- `depth_controller`가 멈추면 마지막 `Fz`가 유지될 수 있습니다.
- 장애 전파가 조용히 발생해서 디버깅이 어려워집니다.

**권장사항**
- 각 자동 입력마다 최근 수신 시각을 저장하고 timeout을 적용해야 합니다.
- stale 시 해당 자동 입력만 0으로 떨어뜨리거나, 안전 모드로 전환하는 것이 좋습니다.

---

### 4. `allocator_node`의 `Fx` 부호 반전이 코드 내부에 하드코딩되어 있음
- Severity: Medium
- File: `src/rov_control/rov_control/allocator_node.py:560`

**문제**
- allocator 입력에서 `fx = float(msg.force.x) * -1`로 x축 힘이 무조건 뒤집힙니다.
- 이 반전은 파라미터화되어 있지 않고, 주석만으로도 충분히 설명되어 있지 않습니다.

**영향**
- teleop, position controller, allocator 사이 좌표계 해석이 쉽게 꼬일 수 있습니다.
- 다른 사람 입장에서는 “전진이 왜 여기서 뒤집히지?”를 코드 추적으로만 알아내야 합니다.
- 장비 방향이나 프레임 convention이 바뀌면 allocator 수정이 필요해집니다.

**권장사항**
- 최소한 파라미터(`surge_sign`, `body_x_sign`)로 외부화하는 게 좋습니다.
- 문서화가 필요합니다.

---

### 5. `allocator_node`는 진짜 최적화 allocation이 아니라 축별 분리 후 합산 방식이라 saturation 시 거동이 비직관적일 수 있음
- Severity: Medium
- File: `src/rov_control/rov_control/allocator_node.py:591`
- File: `src/rov_control/rov_control/allocator_node.py:603`
- File: `src/rov_control/rov_control/allocator_node.py:611`
- File: `src/rov_control/rov_control/allocator_node.py:638`

**문제**
- 수평은 `Fx/Fy`와 `Tz`를 분리해서 각각 pseudo-inverse 적용 후 더합니다.
- 수직은 `Fz`와 `Tx/Ty`를 나눠 계산한 뒤 우선순위로 합칩니다.
- 마지막에는 그룹별 normalize를 합니다.

즉 이것은
- 하나의 constrained optimization allocation
- 또는 full 6DOF QP allocation
이 아니라,
**분리 계산 + heuristic saturation/normalization** 구조입니다.

**영향**
- saturation 시 의도한 wrench 비율이 보존되지 않을 수 있습니다.
- heave와 pitch/roll 요구가 동시에 큰 경우 실제 출력이 튜닝에 매우 민감해집니다.
- gain tuning을 많이 건드릴수록 예측이 어려워질 수 있습니다.

**권장사항**
- 지금 구조를 유지해도 되지만, 문서상 “heuristic allocator”임을 명시하는 게 좋습니다.
- 향후 고도화 시에는 weighted least-squares / QP allocator로 가는 것이 자연스럽습니다.

---

### 6. `position_controller`의 world-frame planar force를 body-frame으로 회전한 뒤 `force.z`까지 사용함
- Severity: Medium
- File: `src/rov_control/rov_control/position_controller.py:411`
- File: `src/rov_control/rov_control/position_controller.py:424`

**문제**
- planar 위치 오차에서 만든 world-frame 힘 `[Fx, Fy, 0]`를 body frame으로 회전한 뒤 `force_body_z`를 그대로 출력합니다.
- 기체가 기울어져 있으면 `force_body_z != 0`가 될 수 있습니다.

**영향**
- 평면 위치 hold가 heave 요구를 만들 수 있습니다.
- `wrench_merger`는 이 값을 depth heave와 더하기 때문에, depth/position coupling이 생깁니다.
- 의도된 level compensation일 수도 있지만, 위치제어기 단에서 z를 직접 내는 것은 설계 의도를 모르면 해석이 어렵습니다.

**권장사항**
- 의도된 설계라면 문서화가 필요합니다.
- 의도가 아니라면 `force_body_z=0`로 두고 allocator 쪽 보상만 남기는 편이 더 명확합니다.

---

### 7. `depth_controller`의 release target offset은 항상 현재 depth에 고정 오프셋을 더함
- Severity: Low
- File: `src/rov_control/rov_control/depth_controller.py:334`

**문제**
- manual heave 해제 시 목표 수심을 `current_depth + offset`으로 잡습니다.
- offset이 항상 같은 방향으로 적용됩니다.

**영향**
- 운용자가 기대하는 “놓은 자리 고정”이 아니라 약간 편향된 목표로 재설정될 수 있습니다.
- 튜닝값을 모르는 사용자는 release 후 depth가 미묘하게 다시 움직인다고 느낄 수 있습니다.

**권장사항**
- 이 offset이 필요한 이유를 문서화하거나,
- 기본값을 0으로 두고 필요한 상황에서만 쓰는 것이 더 직관적입니다.

---

## Positive Notes

### 1. 제어 구조가 기능별로 잘 분리되어 있음
- 자세, 수심, 위치, 병합, allocation 단계가 분리되어 있어 추적이 쉽습니다.

### 2. `wrench` 기반 인터페이스가 깔끔함
- 상위 제어기가 모두 `geometry_msgs/Wrench` 또는 scalar heave로 연결되어 있어 확장성이 좋습니다.

### 3. `allocator_node`의 수평/수직 그룹 분리는 실용적임
- `Fx/Fy/Tz`와 `Fz/Tx/Ty`를 분리한 구조는 실제 ROV 하드웨어 배치와 잘 맞습니다.

### 4. `attitude_controller`의 yaw release capture 로직은 운용감 측면에서 좋음
- manual yaw 해제 후 heading hold 복귀가 부드럽게 설계되어 있습니다.

---

## Recommended Priority

### Immediate
- `attitude_controller`에 manual input timeout 추가
- `position_controller`에 manual input timeout 추가
- `wrench_merger`에 auto input timeout 추가

### Short-term
- `allocator_node`의 x축 부호 반전 파라미터화
- `position_controller`의 `force_body_z` 의도 명확화

### Mid-term
- allocator 구조 문서화
- 필요 시 weighted allocation 또는 QP allocator 검토

---

## Review Conclusion

현재 제어 코드는 구조적으로는 잘 나뉘어 있고, 실제 운용형 ROV 제어 구조에 가깝습니다.

다만 가장 큰 문제는 **입력/출력 freshness 관리가 노드별로 일관되지 않아서 stale command가 latch될 수 있다는 점**입니다.
이 부분은 실제 운용 안정성과 디버깅 난이도에 직접 영향을 주므로, 우선순위를 높게 두고 정리하는 것이 좋습니다.
