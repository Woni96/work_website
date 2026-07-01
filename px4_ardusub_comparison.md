# Robster V5 제어기 vs PX4 / ArduSub 비교 정리

작성일: 2026-07-01

이 문서는 현재 워크스페이스의 **실제 사용 중인 ROS 2 Python 제어 파이프라인**을 기준으로, **PX4 UUV(BlueROV2/experimental)** 및 **ArduSub**와의 기능 차이와 파라미터 관점의 차이를 정리한 문서입니다.

비교 대상 기준:
- 현재 코드: `src/rov_control/rov_control/*.py`, `src/rov_bringup/launch/sim.launch.py`
- PX4: 공식 PX4 docs의 UUV/BlueROV2 및 parameter reference
- ArduSub: 공식 ArduSub mode/parameter 문서

공식 문서 확인일: 2026-07-01

---

## 1. 현재 Robster V5 제어 구조 요약

현재 제어기는 **오토파일럿 펌웨어 1개**가 아니라, ROS 2 노드들을 조합한 **분산형 제어 파이프라인**입니다.

구성:
- `joy_wrench_teleop`
  - 수동 조종 입력 생성
  - `/rov/wrench_manual`, `/cmd_attitude`, `/cmd_attitude_trim`, `/rov/armed` 발행
- `depth_controller`
  - depth hold / pilot depth-rate 해석
  - `/ctrl/depth_heave`, `/ctrl/depth_active` 발행
- `position_controller`
  - DVL + IMU 기반 XY hold
  - `/ctrl/position_force`, `/rov/position_estimate` 발행
- `attitude_controller`
  - roll/pitch/yaw stabilization 및 yaw hold
  - `/ctrl/attitude_torque` 발행
- `wrench_merger`
  - manual / depth / position / attitude 출력을 우선순위로 병합
  - `/rov/wrench_cmd` 발행
- `allocator_node`
  - 최종 wrench를 8개 thruster 명령으로 변환
  - `/thruster_cmd` 발행

핵심 특징:
- 수동/자동 축 혼합이 가능함
- heave는 depth hold에 종속된 pilot rate 명령으로도 동작함
- yaw는 수동 override가 매우 강함
- 최종 추력 분배가 전용 allocator에 분리되어 있음
- GUI를 통한 노드별 runtime tuning을 전제하고 있음

---

## 2. 큰 그림 비교

| 항목 | 현재 Robster V5 | PX4 UUV | ArduSub |
|---|---|---|---|
| 아키텍처 | ROS 2 노드 분산형 | 펌웨어 내 통합형 | 펌웨어 내 통합형 |
| 수동/자동 병합 | `wrench_merger`에서 축별 병합 | flight mode/task 내부 처리 | mode/controller 내부 처리 |
| depth hold | 별도 `depth_controller` | UUV position/altitude 계열 | `ALT_HOLD` |
| XY hold | 별도 `position_controller` | UUV position mode | `POSHOLD` |
| attitude hold | 별도 `attitude_controller` | UUV stabilized/acro/position | `STABILIZE`, `ALT_HOLD`, `POSHOLD` |
| thruster allocation | 사용자 정의 allocator | PX4 control allocation | frame/motor matrix 기반 mixer |
| joystick semantics | BlueROV/ArduSub 스타일에 가까움 | `UUV_STICK_MODE`로 UUV 전용 매핑 제공 | 기본적으로 ROV 운용 중심 |
| mission / auto navigation | 현재 코드상 없음 | generic offboard/return, UUV는 experimental | `AUTO`, `GUIDED`, `CIRCLE`, `SURFACE`, `SURFTRAK` |
| failsafe / EKF / arming checks | 최소 수준 | 강함 | 강함 |

요약:
- **현재 제어기 감성은 PX4보다 ArduSub에 더 가깝습니다.**
- 다만 구현 방식은 ArduSub/PX4처럼 통합 펌웨어가 아니라, **ROS 2에서 축별 제어기를 분해해서 재구성한 형태**입니다.

---

## 3. 기능 차이 상세

### 3.1 현재 Robster V5 vs PX4 UUV

PX4는 현재 공식 문서 기준으로 **Submarines/UUV가 experimental** 범주이며, BlueROV2 Heavy 8-thruster 구성을 지원합니다. 또한 UUV 전용 파라미터(`UUV_*`)와 조이스틱 모드(`UUV_STICK_MODE`)를 제공합니다.

현재 코드 대비 PX4와의 차이:

1. **제어 구조 차이**
- 현재 코드는 depth / position / attitude / merge / allocation이 모두 분리됨
- PX4는 내부 flight task + controller + control allocation으로 묶여 있음
- 현재 코드는 특정 축만 enable/disable 하기가 쉽고 디버깅이 직관적임
- PX4는 통합성이 높고 failsafe/mission/offboard 연동이 강함

2. **모드 구조 차이**
- PX4 BlueROV2 문서상 지원 manual/assisted 모드:
  - `Manual`
  - `Stabilized`
  - `Acro`
  - `Altitude`
  - `Position`
- 현재 Robster V5는 모드 스위치형이라기보다
  - arm/disarm
  - 각 controller `control_enabled`
  - manual override threshold
  - `depth_active`
  로 동작 조합을 만듦

3. **thruster allocation 철학 차이**
- 현재 allocator는 `H`/`V` 서브행렬로 수평/수직 thruster를 분리하고,
  heave 우선도/토크 우선도/수평 보상/피치 모멘트 보상 등을 후처리함
- PX4는 `CA_ROTOR*`, `CA_R_REV`, `CA_ROTOR_COUNT` 등으로 rotor geometry와 reversible motor를 정의하는 **일반화된 control allocation** 방식임
- 즉 현재 코드는 **현재 하드웨어에 매우 최적화된 전용 allocator**, PX4는 **재사용성 높은 일반 allocator**에 가까움

4. **안전/시스템 기능 차이**
- 현재 코드는 `armed` gating 중심의 비교적 단순한 안전 구조
- PX4는 prearm check, output state machine, failure handling, offboard/generic return 같은 상위 기능이 더 강함
- 따라서 실해역 운용에서 시스템 레벨 안전 기능은 PX4 쪽이 훨씬 많음

5. **현 구조의 장점**
- ROS 2 기반 실험/튜닝/GUI 연동이 빠름
- 센서/제어/GUI를 원하는 방식으로 바꾸기 쉬움
- 수중 특화 로직(예: heave 보호, XY motion 보호, depth sensor offset 보정)을 자유롭게 넣기 쉬움

6. **현 구조의 약점**
- 상태추정, failsafe, mission, logging, GCS 생태계는 PX4보다 약함
- controller 간 상호작용이 늘어날수록 운영 복잡도가 커짐
- mode abstraction이 약해서 운용 절차를 문서/GUI에 의존하게 됨

### 3.2 현재 Robster V5 vs ArduSub

현재 코드는 **ArduSub 운용 철학을 많이 닮았습니다.**

가까운 부분:
- depth hold에서 manual heave를 **depth rate request**로 해석하는 동작
- 수동 조종이 들어오면 특정 축 자동 hold를 해제/약화하는 구조
- vectored ROV 기반 조이스틱 운용 전제
- position hold / depth hold / stabilized 개념 분리

다른 부분:

1. **ArduSub은 mode 중심 / 현재 코드는 노드 enable 중심**
- ArduSub: `MANUAL`, `ACRO`, `STABILIZE`, `ALT_HOLD`, `POSHOLD`, `AUTO`, `GUIDED`, `SURFACE`, `SURFTRAK`
- 현재 코드: 모드 전환기보다 controller on/off와 manual override 임계값으로 거동이 결정됨

2. **ArduSub은 EKF/mission/failsafe가 내장**
- 현재 코드는 DVL/IMU/depth를 직접 받아 hold를 구성하지만, ArduSub처럼 전체 navigation/autopilot 계층은 없음

3. **현재 position controller는 더 단순함**
- 현재 코드는 XY 위치 hold에 집중된 PD 구조
- ArduSub은 `PSC_*` 계열을 통해 수평/수직 위치-속도-가속도 체인을 더 체계적으로 구성함

4. **현재 attitude controller는 수중 운용용 커스텀 보호로직이 많음**
- `heave_protect_threshold`
- `strong_heave_threshold`
- `xy_motion_protect_threshold`
- `yaw_hold_settle_time_sec`
- `orientation_filter_*`
- 이런 항목은 ArduSub의 전통적 파라미터 이름과 직접 1:1 대응되기보다, 현 플랫폼의 운용 감각을 맞추기 위한 커스텀 로직임

5. **현재 allocator는 ArduSub frame mixer보다 커스텀성이 강함**
- ArduSub은 `FRAME_CONFIG` 기반 frame/motor 구성이 중심
- 현재 allocator는 thrust budget, 수평 레벨 보상, pitch moment 보상 같은 후처리 정책이 더 많이 들어가 있음

---

## 4. 파라미터 세팅 차이

## 4.1 자세 제어(attitude)

### 현재 코드 주요 파라미터
- `kp_roll`, `kd_roll`, `ki_roll`
- `kp_pitch`, `kd_pitch`, `ki_pitch`
- `kp_yaw`, `kd_yaw`
- `tx_limit`, `ty_limit`, `tz_limit`
- `rp_torque_slew_rate`
- `yaw_hold_enabled`
- `yaw_manual_override_threshold`
- `roll_pitch_error_deadband_deg`
- `orientation_filter_enabled`
- `orientation_filter_measurement_alpha`
- `orientation_filter_max_correction_rate_deg`

### PX4 쪽 대응
- `UUV_ROLL_P`, `UUV_ROLL_D`, `UUV_PITCH_P`, `UUV_PITCH_D`, `UUV_YAW_P`, `UUV_YAW_D`
- 또는 일반 multicopter 계열 `MC_ROLL_P`, `MC_PITCH_P`, `MC_YAW_P`, `MC_YAW_WEIGHT`, `MC_ROLLRATE_MAX`, `MC_YAWRATE_MAX`

### ArduSub 쪽 대응
- `ATC_ANG_RLL_P`, `ATC_ANG_PIT_P`, `ATC_ANG_YAW_P`
- `ATC_RAT_YAW_P` 및 다른 `ATC_RAT_*`
- `ATC_RATE_R_MAX`, `ATC_RATE_P_MAX`, `ATC_RATE_Y_MAX`

### 차이 해석
- 현재 코드는 **축별 P/ID + override 정책 + 보호로직**이 하나의 노드 안에 강하게 결합되어 있음
- PX4/ArduSub은 일반적으로 **angle loop / rate loop / saturation**이 더 분리되어 있음
- 현재 `yaw_hold_enabled`, `capture_yaw_target_on_release` 같은 항목은 ArduSub/PX4의 모드 기반 yaw hold보다 **운용 편의성 중심 커스텀 파라미터**에 가까움

---

## 4.2 깊이 제어(depth)

### 현재 코드 주요 파라미터
- `kp_depth`, `ki_depth`, `kd_depth`
- `max_heave`, `max_upward_heave`
- `depth_integral_limit`
- `pilot_depth_rate_enabled`
- `max_pilot_depth_rate`
- `manual_heave_release_target_offset`
- `min_target_depth`, `max_target_depth`
- `depth_sensor_offset_*`
- `depth_sensor_offset_compensation_enabled`

### PX4 쪽 대응
- UUV 전용 파라미터로는 `UUV_GAIN_Z_P`, `UUV_GAIN_Z_D`, `UUV_PGM_VEL`, `UUV_THRUST_SAT` 계열이 더 직접적
- 일반 위치 제어 계열로 보면 `MPC_Z_P`, `MPC_Z_VEL_P_ACC`, `MPC_Z_VEL_I_ACC`, `MPC_Z_VEL_MAX_UP`, `MPC_Z_VEL_MAX_DN`

### ArduSub 쪽 대응
- `PSC_POSZ_P`
- `PSC_VELZ_P`, `PSC_VELZ_I`
- `PILOT_SPEED_UP`, `PILOT_SPEED_DN`, `PILOT_ACCEL_Z`
- `SURFACE_DEPTH`

### 차이 해석
- 현재 depth controller는 **ArduSub와 매우 유사한 조종 감각**을 목표로 함
  - pilot heave → depth-rate 명령
  - stick release → 현재 depth hold
- 다만 파라미터 체계는 ArduSub의 `PSC_*` 식 계층형 이름이 아니라, **기능 설명형 사용자 정의 파라미터**임
- `depth_sensor_offset_compensation_enabled` 같은 항목은 설치 오차를 runtime에서 직접 보정하기 좋아 실제 장비에 유용함

---

## 4.3 위치 제어(position hold)

### 현재 코드 주요 파라미터
- `kp_x`, `kd_x`, `kp_y`, `kd_y`
- `max_force_x`, `max_force_y`, `max_force_z`
- `yaw_rate_damping_gain`
- `manual_yaw_damping_boost`
- `manual_xy_override_threshold`
- `capture_initial_position_target`
- `capture_target_on_manual_release`
- `use_dvl_position`
- `integrate_dvl_velocity_when_position_unavailable`
- `dvl_mount_roll_deg`, `dvl_mount_pitch_deg`, `dvl_mount_yaw_deg`

### PX4 쪽 대응
- `UUV_GAIN_X_P`, `UUV_GAIN_X_D`, `UUV_GAIN_Y_P`, `UUV_GAIN_Y_D`
- `UUV_POS_MODE`, `UUV_POS_STICK_DB`, `UUV_PGM_VEL`
- 일반 position 계열로 보면 `MPC_XY_P`, `MPC_XY_VEL_P_ACC`, `MPC_XY_VEL_I_ACC`, `MPC_XY_VEL_D_ACC`

### ArduSub 쪽 대응
- `PSC_VELXY_P`, `PSC_VELXY_I`, `PSC_VELXY_D`
- waypoint/auto 계열은 `WP_SPD`, `WP_RADIUS_M`, `WP_ACC`

### 차이 해석
- 현재 위치 제어는 **PD 기반 XY hold** 중심이며, DVL 위치 유무에 따라 속도 적분 fallback을 가짐
- PX4/ArduSub은 더 일반적인 navigation stack과 이어져 있음
- 현재 구조는 DVL 장착각 보정과 fallback이 명시적이라 현장 디버깅에 좋지만, 글로벌 navigation/mode 통합은 약함

---

## 4.4 스틱/조종 입력

### 현재 코드 주요 파라미터
- `axis_surge`, `axis_sway`, `axis_heave`, `axis_yaw`
- `scale_surge`, `scale_sway`, `scale_heave`, `scale_yaw`
- `speed_scale`, `axis_speed_step`
- `input_expo`, `input_filter_alpha`
- `hat_attitude_step_deg`, `max_hat_roll_deg`, `max_hat_pitch_deg`

### PX4 쪽 대응
- `UUV_STICK_MODE`
- `UUV_SGM_*`, `UUV_RGM_*`

### ArduSub 쪽 대응
- `JS_GAIN_DEFAULT`, `JS_GAIN_MIN`, `JS_GAIN_MAX`, `JS_GAIN_STEPS`
- `JS_THR_GAIN`

### 차이 해석
- 현재 입력 계층은 ArduSub처럼 **파일럿 운용성**을 중시함
- 하지만 ArduSub처럼 GCS/joystick profile 중심이 아니라, ROS 노드 파라미터로 직접 정의됨
- PX4는 `UUV_STICK_MODE`로 UUV형/legacy형을 크게 나누고, 현재 코드는 더 세밀한 축별/trim별 커스터마이징이 가능함

---

## 4.5 Thruster allocation / frame 설정

### 현재 코드 주요 파라미터
- `heave_gain`
- `horizontal_output_gain`
- `yaw_output_gain`
- `vertical_output_gain`
- `rear_vertical_bias`
- `pitch_torque_gain`
- `torque_first_allocation`
- `slew_rate`
- `max_output`
- `output_scale`
- `output_deadband`
- `level_horizontal_compensation_*`
- `surge_pitch_moment_compensation_*`
- `imu_pitch_hold_compensation_*`

### PX4 쪽 대응
- `CA_ROTOR_COUNT`
- `CA_R_REV`
- `CA_ROTOR*_AX/AY/AZ`
- `CA_ROTOR*_PX/PY/PZ`
- `CA_ROTOR*_CT`, `CA_ROTOR*_KM`
- rotor별 `CA_R*_SLEW`

### ArduSub 쪽 대응
- `FRAME_CONFIG`
- `MOT_THST_EXPO`
- `RC_SPEED`
- `MOT_YAW_HEADROOM`

### 차이 해석
- 현재 allocator는 **frame geometry + 운용 정책이 섞여 있는 커스텀 allocator**임
- PX4는 geometry를 파라미터화해 allocator가 일반화돼 있음
- ArduSub은 frame 종류(`FRAME_CONFIG`)와 모터/출력 계열 파라미터가 중심이라, 현재 코드처럼 세밀한 후처리 로직은 직접 노출되지 않음

---

## 5. 현재 기본 세팅값 관점에서 눈에 띄는 차이

현재 `sim.launch.py` 기준 기본값:
- allocator
  - `output_scale = 0.25`
  - `heave_gain = 1.4`
  - `horizontal_output_gain = 2.5`
  - `vertical_output_gain = 3.0`
  - `yaw_output_gain = 0.35`
- attitude
  - `kp_roll = 0.50`, `ki_roll = 0.04`, `kd_roll = 0.0`
  - `kp_pitch = 0.45`, `ki_pitch = 0.04`, `kd_pitch = 0.08`
  - `kp_yaw = 0.4`, `kd_yaw = 0.12`
- depth
  - `kp_depth = 2.0`, `ki_depth = 0.25`, `kd_depth = 0.8`
  - `pilot_depth_rate_enabled = True`
  - `max_pilot_depth_rate = 0.35`
- position
  - `kp_x = 0.8`, `kd_x = 0.6`, `kp_y = 0.8`, `kd_y = 0.6`
  - 시뮬레이터에서는 `control_enabled = False`
  - `use_dvl_position = False`

해석:
- 현재 값은 **고전적 autopilot 기본값 복제**라기보다, 실기/시뮬레이터 감각에 맞춘 실용 세팅**에 가깝습니다.
- 특히 allocator gain과 `output_scale`은 PX4/ArduSub에서 흔히 보는 “frame + motor model + autopilot saturation”보다, **실제 하드웨어 반응 맞춤형 scaling** 비중이 큽니다.
- depth controller는 ArduSub식 depth-hold 조작감과 가장 유사합니다.
- position controller는 아직 sim에서 기본 off라서, 현재 기본 운용은 full position-hold보다 **manual + attitude/depth stabilization** 쪽에 더 가깝습니다.

---

## 6. 결론

### 6.1 PX4와 비교하면
- 현재 구조는 PX4보다 **더 실험적이지만 더 유연한 ROS형 구조**입니다.
- PX4 UUV는 공식 문서상 여전히 **experimental support** 범주이며, BlueROV2 Heavy 지원과 `UUV_*` 파라미터를 제공합니다.
- 그러나 시스템 통합, allocator 일반화, 상위 모드/자동화/안전 기능은 PX4가 더 강합니다.
- 현재 코드는 **플랫폼 맞춤형 수중 제어 로직 커스터마이징**에서는 더 직접적입니다.

### 6.2 ArduSub와 비교하면
- 현재 제어 철학은 **ArduSub에 더 가깝습니다.**
- 특히 depth hold 조작감, stick release 후 hold, vectored ROV 중심 운용이 그렇습니다.
- 다만 ArduSub은 mode/EKF/failsafe/mission 체계가 강하고, 현재 코드는 이를 ROS 노드 조합으로 부분 재현한 구조입니다.

### 6.3 한 줄 요약
- **운용 감각:** ArduSub에 가까움
- **파라미터 구조:** 사용자 정의 커스텀 구조
- **추력 배분 철학:** PX4보다 전용기체 최적화형
- **시스템 통합도:** PX4/ArduSub < 현재 코드 아님, 오히려 현재 코드가 더 낮음

---

## 7. 실무적으로 파라미터 옮길 때의 대응표

| 현재 코드 | PX4에서 볼 파라미터 | ArduSub에서 볼 파라미터 |
|---|---|---|
| `kp_roll`, `kp_pitch`, `kp_yaw` | `UUV_ROLL_P`, `UUV_PITCH_P`, `UUV_YAW_P` 또는 `MC_*` | `ATC_ANG_*`, `ATC_RAT_*` |
| `kd_roll`, `kd_pitch`, `kd_yaw` | `UUV_*_D` | `ATC_RAT_*_D` |
| `kp_depth` | `UUV_GAIN_Z_P` 또는 `MPC_Z_P` | `PSC_POSZ_P` |
| `ki_depth`, `kd_depth` | `MPC_Z_VEL_I_ACC`, `MPC_Z_VEL_P_ACC` 계열이 더 가까움 | `PSC_VELZ_I`, `PSC_VELZ_P` |
| `max_pilot_depth_rate` | `MPC_Z_VEL_MAX_UP/DN` 또는 UUV 수직 입력 gain | `PILOT_SPEED_UP`, `PILOT_SPEED_DN` |
| `kp_x`, `kd_x`, `kp_y`, `kd_y` | `UUV_GAIN_X/Y_*` 또는 `MPC_XY_P`, `MPC_XY_VEL_*` | `PSC_VELXY_*` |
| `output_scale`, `max_output` | actuator / control allocation saturation | motor output / thrust scaling |
| `torque_first_allocation` | allocator policy에 가까운 custom 개념 | 직접 1:1 대응 없음 |
| `manual_*_override_threshold` | mode/stick deadband 로직에 흩어짐 | mode/pilot control 내부 로직 + 일부 gain |
| `FRAME`/geometry 관련 없음(코드 내부) | `CA_ROTOR*`, `CA_R_REV`, `CA_ROTOR_COUNT` | `FRAME_CONFIG` |

---

## 8. 참고 링크

### 현재 코드
- `src/rov_control/rov_control/allocator_node.py`
- `src/rov_control/rov_control/attitude_controller.py`
- `src/rov_control/rov_control/depth_controller.py`
- `src/rov_control/rov_control/position_controller.py`
- `src/rov_control/rov_control/wrench_merger.py`
- `src/rov_bringup/launch/sim.launch.py`

### PX4 공식 문서
- https://docs.px4.io/main/en/frames_sub/
- https://docs.px4.io/main/en/frames_sub/bluerov2
- https://docs.px4.io/main/en/advanced_config/parameter_reference
- https://docs.px4.io/main/en/flight_modes_mc/

### ArduSub 공식 문서
- https://ardupilot.org/sub/docs/modes.html
- https://ardupilot.org/sub/docs/parameters-Sub-stable-V4.5.7.html

---

## 9. 메모

이 문서의 비교 중 일부는 직접 1:1 이름 대응이 없는 부분에 대해 **기능상 가장 가까운 파라미터를 대응시킨 추론**을 포함합니다.
특히:
- 현재 커스텀 allocator 보정 파라미터
- manual override threshold 계열
- yaw hold settle / heave protect / xy protect 계열
은 PX4/ArduSub의 단일 파라미터와 정확히 일치하지 않고, 여러 mode/controller 로직에 분산되어 있는 경우가 많습니다.

---

## 10. 별첨 - ArduSub vs 현재 제어기 파라미터 1:1 비교표

이 별첨은 **ArduSub 파라미터와 현재 Robster V5 제어기 파라미터를 기능 기준으로 묶어서** 비교한 표입니다.

주의:
- 현재 제어기는 ROS 2 기반 커스텀 제어기라서 ArduSub과 **완전한 1:1 대응이 없는 항목**이 있습니다.
- 그런 경우에는 가장 기능이 가까운 ArduSub 항목을 대응시켰습니다.
- 현재 제어기 파라미터 이름은 코드 선언 기준이고, 현재 기본값은 `sim.launch.py` 기준 기본 세팅을 우선 적었습니다.

### 10.1 자세 제어(Attitude / Stabilize / Yaw Hold)

| 기능 항목 | 현재 제어기 파라미터 | 현재 기본값 | 현재 파라미터 의미 | ArduSub 대응 파라미터 | ArduSub 의미 |
|---|---|---:|---|---|---|
| Roll 각도 비례 이득 | `kp_roll` | 0.50 | roll 오차에 비례해서 복원 토크를 키움 | `ATC_ANG_RLL_P` | roll 각도 오차를 목표 roll rate로 바꾸는 비례 이득 |
| Roll 적분 이득 | `ki_roll` | 0.04 | roll 지속 오차를 누적 보정 | `ATC_RAT_RLL_I` | roll rate 오차의 누적 오차 보정 |
| Roll 미분/감쇠 | `kd_roll` | 0.0 | roll rate 기반 감쇠 | `ATC_RAT_RLL_D` | roll rate 변화에 대한 감쇠 |
| Pitch 각도 비례 이득 | `kp_pitch` | 0.45 | pitch 오차에 비례한 복원 토크 | `ATC_ANG_PIT_P` | pitch 각도 오차를 목표 pitch rate로 변환 |
| Pitch 적분 이득 | `ki_pitch` | 0.04 | pitch 지속 오차 누적 보정 | `ATC_RAT_PIT_I` | pitch rate 오차 적분 보정 |
| Pitch 미분/감쇠 | `kd_pitch` | 0.08 | pitch rate 감쇠 | `ATC_RAT_PIT_D` | pitch rate 변화 감쇠 |
| Yaw hold 비례 이득 | `kp_yaw` | 0.4 | yaw 오차에 비례한 yaw 토크 | `ATC_ANG_YAW_P` | yaw 각도 오차를 목표 yaw rate로 변환 |
| Yaw 감쇠 이득 | `kd_yaw` | 0.12 | yaw rate에 대한 감쇠 | `ATC_RAT_YAW_D` | yaw rate 변화 감쇠 |
| Roll 토크 제한 | `tx_limit` | 0.22 | allocator로 넘기기 전 roll 토크 상한 | `ATC_RATE_R_MAX` | roll rate 최대치 제한 |
| Pitch 토크 제한 | `ty_limit` | 0.22 | allocator로 넘기기 전 pitch 토크 상한 | `ATC_RATE_P_MAX` | pitch rate 최대치 제한 |
| Yaw 토크 제한 | `tz_limit` | 0.08 | allocator로 넘기기 전 yaw 토크 상한 | `ATC_RATE_Y_MAX` | yaw rate 최대치 제한 |
| Roll/Pitch 토크 slewing | `rp_torque_slew_rate` | 0.8 | roll/pitch 출력 변화 속도 제한 | `ATC_RAT_RLL_SMAX`, `ATC_RAT_PIT_SMAX` | rate loop 출력 변화 급격성 제한 |
| Yaw hold 사용 여부 | `yaw_hold_enabled` | True | yaw stick을 놓으면 현재 heading 유지 | mode 동작에 내장 | `STABILIZE`/`ALT_HOLD`/`POSHOLD`에서 yaw hold 성격 반영 |
| 수동 yaw override 기준 | `yaw_manual_override_threshold` | 0.02 | 이 값 이상 yaw stick 입력이면 자동 yaw hold 해제 | RC 입력 deadband/조종 로직 | yaw stick이 들어오면 수동 yaw가 우선됨 |
| yaw release 후 목표 재캡처 | `capture_yaw_target_on_release` | True | yaw stick을 놓는 순간 현재 yaw를 새 hold 목표로 사용 | mode 동작에 내장 | 조종 입력 해제 후 heading 유지 동작 |
| yaw settle 시간 | `yaw_hold_settle_time_sec` | 0.25 | yaw 수동→자동 전환 시 짧은 안정화 시간 | 직접 1:1 없음 | 내부 제어 전환 로직에 분산 |
| yaw 오차 deadband | `yaw_error_deadband_deg` | 0.5 | 아주 작은 yaw 오차는 무시 | 직접 1:1 없음 | 소오차 무시는 내부 제어/입력 해석에 분산 |
| 큰 자세 오차 차단 | `large_tilt_disable_deg` | 85.0 | 너무 크게 기울면 자세 제어 정지 | `ANGLE_MAX`와 mode 제한들 | 큰 기울기 상황의 제어 한계 관리 |
| body rate 제한 | `max_body_rate_for_control` | 1.5 | 회전 속도가 너무 크면 controller 억제 | `ATC_RATE_*_MAX` 계열 | 축별 rate 제한 |
| 토크 deadband | `torque_deadband` | 1e-4 | 너무 작은 토크 명령은 0 처리 | 직접 1:1 없음 | 모터/제어 미세 출력 억제는 내부적으로 처리 |
| 자세 오차 deadband | `roll_pitch_error_deadband_deg` | 1.0 | 작은 roll/pitch 오차는 무시 | 직접 1:1 없음 | 소오차 무시 로직에 가까움 |
| 적분 한계 | `roll_integral_limit` | 0.20 | roll/pitch 적분항 windup 제한 | `ATC_RAT_RLL_IMAX`, `ATC_RAT_PIT_IMAX` | 적분항 최대치 제한 |
| 자세 필터 사용 | `orientation_filter_enabled` | True | IMU 자세값 보정 필터 사용 | EKF/필터 내부 설정 | 자세 추정 품질 관련 파라미터 |
| 필터 측정 반영 비율 | `orientation_filter_measurement_alpha` | 0.08 | 측정값을 얼마나 빨리 반영할지 | EKF 게인류 | 추정값-센서값 혼합 비율 개념 |
| 필터 최대 보정 속도 | `orientation_filter_max_correction_rate_deg` | 45.0 | 필터 보정량의 초당 상한 | EKF 제한 파라미터 | 추정 보정 속도 제한 |
| heave 중 자세 억제 시작 | `heave_protect_threshold` | 0.08 | 상하 조작이 시작되면 roll/pitch 제어를 줄임 | 직접 1:1 없음 | ArduSub는 mode/조종 감각에 흡수되어 있음 |
| 강한 heave 중 자세 억제 | `strong_heave_threshold` | 0.20 | 상하 조작이 크면 더 강하게 자세 제어 억제 | 직접 1:1 없음 | 현재 제어기 고유 보호 로직 |
| XY 이동 중 자세 억제 시작 | `xy_motion_protect_threshold` | 0.20 | 전후/좌우 이동이 생기면 자세 제어 약화 | 직접 1:1 없음 | 현재 제어기 고유 보호 로직 |
| 강한 XY 이동 중 자세 억제 | `strong_xy_motion_threshold` | 0.40 | 이동이 크면 자세 제어를 더 강하게 줄임 | 직접 1:1 없음 | 현재 제어기 고유 보호 로직 |
| 이동 중 자세 스케일 | `rp_scale_when_xy_motion` | 0.85 | 이동 중 roll/pitch 제어 축소 비율 | 직접 1:1 없음 | 현재 제어기 고유 보호 로직 |
| 강한 이동 중 자세 스케일 | `rp_scale_when_strong_xy_motion` | 0.70 | 강한 이동 중 자세 제어 축소 비율 | 직접 1:1 없음 | 현재 제어기 고유 보호 로직 |

### 10.2 깊이 제어(Depth Hold / Alt Hold)

| 기능 항목 | 현재 제어기 파라미터 | 현재 기본값 | 현재 파라미터 의미 | ArduSub 대응 파라미터 | ArduSub 의미 |
|---|---|---:|---|---|---|
| 깊이 위치 P | `kp_depth` | 2.0 | 목표 깊이와 현재 깊이 오차에 대한 비례 출력 | `PSC_POSZ_P` | 깊이 위치 오차를 수직 속도 목표로 변환 |
| 깊이 적분 I | `ki_depth` | 0.25 | 지속적인 depth 오차 보정 | `PSC_VELZ_I` | 수직 속도 제어 적분 보정 |
| 깊이 감쇠 D | `kd_depth` | 0.8 | depth rate 기반 감쇠 | `PSC_VELZ_P`, `PSC_VELZ_D`에 가까움 | 수직 속도/가속도 제어 감쇠 |
| 수직 출력 상한 | `max_heave` | 2.0 | 전체 heave 출력 최대치 | `PILOT_SPEED_UP`, `PILOT_SPEED_DN`, `THR_*` 계열과 조합 | 수직 이동 속도/출력 한계 |
| 상승 방향 출력 상한 | `max_upward_heave` | 0.8 | 상승 방향만 별도로 더 낮게 제한 | `PILOT_SPEED_UP` | 상승 속도 상한 |
| 상승 출력 부호 | `upward_heave_cmd_sign` | -1.0 | 상승 방향 추진 부호 정의 | frame/motor 방향 설정 | 수직 추진 방향 정의 |
| 적분 한계 | `depth_integral_limit` | 2.0 | depth I항 windup 제한 | `PSC_VELZ_IMAX` | 수직 적분 최대치 제한 |
| 수동 heave override 기준 | `manual_heave_override_threshold` | 0.05 | stick 입력이 이보다 크면 수동 개입으로 판단 | pilot input deadband | 조종 입력 유효 판단 기준 |
| 수동 입력 timeout | `manual_wrench_timeout_sec` | 0.5 | 일정 시간 입력이 없으면 stale로 처리 | RC failsafe/input timeout | 조종 입력 신선도 판단 |
| release 후 목표 offset | `manual_heave_release_target_offset` | 0.10 | stick 해제 후 현재 depth를 약간 보정해 hold 목표 설정 | 직접 1:1 없음 | 현재 제어기 커스텀 로직 |
| 초기 목표 자동 캡처 | `capture_initial_depth_target` | True | 시작 시 현재 depth를 자동 목표로 사용 | mode 진입 동작 | `ALT_HOLD` 진입 시 현재 depth hold |
| pilot depth-rate 모드 사용 | `pilot_depth_rate_enabled` | True | heave stick을 depth rate command로 해석 | `ALT_HOLD` 조작 개념 | stick으로 상승/하강 속도를 지시 |
| 최대 pilot depth-rate | `max_pilot_depth_rate` | 0.35 | 파일럿이 줄 수 있는 최대 depth rate | `PILOT_SPEED_UP`, `PILOT_SPEED_DN` | 파일럿 수직 속도 한계 |
| pilot depth-rate 부호 | `pilot_depth_rate_sign` | -1.0 | stick 방향과 depth rate 부호 관계 | RC 방향 / frame 방향 | 상승·하강 입력 부호 정의 |
| 최소 목표 깊이 | `min_target_depth` | 0.0 | 목표 depth 하한 | `SURFACE_DEPTH` 관련 | 수면 접근 제한 기준 |
| 최대 목표 깊이 | `max_target_depth` | 100.0 | 목표 depth 상한 | fence / mission depth 제한 | 너무 깊은 목표 방지 |
| heave 출력 부호 | `heave_cmd_sign` | -1.0 | allocator로 넘기는 heave 부호 | frame/motor 방향 설정 | 수직 추력 방향 |
| depth rate LPF | `depth_rate_alpha` | 0.10 | depth 변화율 저역통과 필터 계수 | 직접 1:1 없음 | 속도 추정 필터 계열에 가까움 |
| 출력 slew 제한 | `max_heave_delta_per_cycle` | 0.8 | depth 제어 출력 변화 속도 제한 | 직접 1:1 없음 | 수직 출력 급변 억제 |
| depth sensor X 오프셋 | `depth_sensor_offset_x` | -0.3 | depth 센서 장착 위치의 X 오차 보정 | 직접 1:1 없음 | 센서 설치 위치 보정 |
| depth sensor Y 오프셋 | `depth_sensor_offset_y` | 0.0 | depth 센서 Y 위치 보정 | 직접 1:1 없음 | 센서 설치 위치 보정 |
| depth sensor Z 오프셋 | `depth_sensor_offset_z` | 0.0 | depth 센서 Z 위치 보정 | 직접 1:1 없음 | 센서 설치 위치 보정 |
| 센서 오프셋 보정 사용 | `depth_sensor_offset_compensation_enabled` | True | 자세 변화 시 센서 위치 보정 반영 | 직접 1:1 없음 | 현재 제어기 커스텀 로직 |

### 10.3 위치 제어(Position Hold / PosHold)

| 기능 항목 | 현재 제어기 파라미터 | 현재 기본값 | 현재 파라미터 의미 | ArduSub 대응 파라미터 | ArduSub 의미 |
|---|---|---:|---|---|---|
| X 위치 비례 이득 | `kp_x` | 0.8 | 목표 X와 현재 X 오차에 대한 비례 힘 | `PSC_VELXY_P`에 기능상 근접 | 수평 위치/속도 제어의 비례 이득 |
| X 감쇠 이득 | `kd_x` | 0.6 | X 방향 속도에 대한 감쇠 | `PSC_VELXY_D` | 수평 속도 변화 감쇠 |
| Y 위치 비례 이득 | `kp_y` | 0.8 | 목표 Y와 현재 Y 오차에 대한 비례 힘 | `PSC_VELXY_P`에 기능상 근접 | 수평 위치/속도 제어 비례 이득 |
| Y 감쇠 이득 | `kd_y` | 0.6 | Y 방향 속도에 대한 감쇠 | `PSC_VELXY_D` | 수평 속도 변화 감쇠 |
| X 출력 상한 | `max_force_x` | 0.8 | X축 자동 보정 힘 한계 | `PSC_ACC_XY`, `WPNAV_SPEED` 계열과 조합 | 수평 보정 강도/가속도 한계 |
| Y 출력 상한 | `max_force_y` | 0.8 | Y축 자동 보정 힘 한계 | `PSC_ACC_XY`, `WPNAV_SPEED` 계열과 조합 | 수평 보정 강도/가속도 한계 |
| Z 출력 상한 | `max_force_z` | 0.8 | 필요 시 위치 제어가 사용하는 Z 보정 한계 | 수직 제어 파라미터와 조합 | 수직 보정 한계 |
| force deadband | `force_deadband` | 1e-4 | 작은 자동 위치 보정은 0 처리 | 직접 1:1 없음 | 작은 모터 떨림 억제 |
| yaw rate damping | `yaw_rate_damping_gain` | 1.2 | yaw 회전 중 XY hold를 더 강하게 감쇠 | 직접 1:1 없음 | 회전 중 position hold 흔들림 완화 |
| 수동 yaw 중 추가 damping | `manual_yaw_damping_boost` | 0.6 | yaw stick 입력 중 XY hold 감쇠 강화 | 직접 1:1 없음 | 현재 제어기 커스텀 로직 |
| 수동 yaw 판단 기준 | `manual_yaw_override_threshold` | 0.05 | yaw 수동 개입 판정 기준 | RC deadband / pilot input | yaw 수동 입력 판단 |
| 수동 XY override 기준 | `manual_xy_override_threshold` | 0.05 | surge/sway 입력이 이보다 크면 position hold 해제 | pilot stick deadband | 조종 입력이 hold를 덮음 |
| 초기 hold 목표 캡처 | `capture_initial_position_target` | True | 시작 시 현재 XY를 hold 목표로 설정 | `POSHOLD` 진입 동작 | 현재 위치를 잡고 유지 |
| 수동 해제 후 목표 재캡처 | `capture_target_on_manual_release` | True | stick을 놓으면 그 위치를 새 hold 목표로 사용 | `POSHOLD` 조종 감각과 유사 | stick release 후 위치 유지 |
| 참조 유효시간 | `valid_timeout_sec` | 0.5 | DVL/위치 참조가 오래되면 자동 hold 중지 | EKF/GPS/DVL health 개념 | 센서 유효성 판단 |
| hold 기준 프레임 | `hold_frame_id` | `dvl_odom` | 위치 hold 좌표계 이름 | EKF frame 내부 | 위치 추정 기준 프레임 |
| DVL position 직접 사용 | `use_dvl_position` | False | DVL position topic을 직접 사용할지 | EK3/DVL integration 설정 | DVL 기반 위치 사용 여부 |
| DVL velocity 적분 fallback | `integrate_dvl_velocity_when_position_unavailable` | True | position이 없으면 velocity 적분으로 hold 추정 | 직접 1:1 없음 | 현재 제어기 fallback |
| DVL 장착 roll 보정 | `dvl_mount_roll_deg` | 0.0 | DVL 설치 각도 roll 보정 | 센서 orientation 설정 | 센서 장착각 보정 |
| DVL 장착 pitch 보정 | `dvl_mount_pitch_deg` | 0.0 | DVL 설치 각도 pitch 보정 | 센서 orientation 설정 | 센서 장착각 보정 |
| DVL 장착 yaw 보정 | `dvl_mount_yaw_deg` | 0.0 | DVL 설치 각도 yaw 보정 | 센서 orientation 설정 | 센서 장착각 보정 |
| 위치 제어 사용 여부 | `control_enabled` | False (sim 기본) | position hold controller 활성화 여부 | mode 전환 | `POSHOLD` 사용 여부 |

### 10.4 수동 입력 / 조종 감각

| 기능 항목 | 현재 제어기 파라미터 | 현재 기본값 | 현재 파라미터 의미 | ArduSub 대응 파라미터 | ArduSub 의미 |
|---|---|---:|---|---|---|
| 전후진 축 매핑 | `axis_surge` | 4 | 조이스틱 어떤 축을 surge로 쓸지 | joystick mapping / button function | 조종기 축 매핑 |
| 좌우 이동 축 매핑 | `axis_sway` | 3 | sway 입력 축 | joystick mapping | 조종기 축 매핑 |
| 상하 이동 축 매핑 | `axis_heave` | 1 | heave 입력 축 | joystick mapping | 조종기 축 매핑 |
| yaw 축 매핑 | `axis_yaw` | 0 | yaw 입력 축 | joystick mapping | 조종기 축 매핑 |
| surge 스케일 | `scale_surge` | -1.0 또는 노드 기본 1.0 | 전후진 입력 크기와 방향 조정 | `JS_GAIN_*` | 조종기 입력 게인 |
| sway 스케일 | `scale_sway` | 1.0 | 좌우 입력 크기 조정 | `JS_GAIN_*` | 조종기 입력 게인 |
| heave 스케일 | `scale_heave` | 2.0 | 상하 입력 크기 조정 | `JS_THR_GAIN` | heave/throttle 입력 게인 |
| yaw 스케일 | `scale_yaw` | 0.5 | yaw 입력 크기 조정 | `JS_GAIN_*` | yaw 입력 게인 |
| roll 스케일 | `scale_roll` | 0.3 | 수동 roll 토크 입력 크기 | joystick gain | 수동 roll 조종 강도 |
| pitch 스케일 | `scale_pitch` | 0.3 | 수동 pitch 토크 입력 크기 | joystick gain | 수동 pitch 조종 강도 |
| 기본 speed scale | `speed_scale` | 0.50 | 전체 조종 감도 기본값 | `JS_GAIN_DEFAULT` | 기본 조종 gain |
| speed step 축 | `axis_speed_step` | 7 | 조종 중 speed scale 단계 변경 축 | `JS_GAIN_STEPS` 개념 | gain 단계 변경 |
| heave deadzone | `heave_deadzone` | 0.18 | heave stick 작은 입력 무시 | RC deadzone | stick deadband |
| 입력 expo | `input_expo` | 1.6 | stick 중앙부를 더 부드럽게 | expo 관련 RC tuning | stick 감도 곡선 |
| 입력 필터 | `input_filter_alpha` | 0.35 | stick 입력 low-pass 필터 | 직접 1:1 없음 | 입력 smoothing |
| hat 자세 step | `hat_attitude_step_deg` | 2.0 또는 노드 기본 5.0 | hat 입력 1회당 trim 변화량 | 직접 1:1 없음 | trim 입력 증분 |
| 최대 hat roll | `max_hat_roll_deg` | 25.0 | hat 기반 roll trim 최대값 | 직접 1:1 없음 | trim 한계 |
| 최대 hat pitch | `max_hat_pitch_deg` | 25.0 | hat 기반 pitch trim 최대값 | 직접 1:1 없음 | trim 한계 |

### 10.5 수동/자동 병합(Wrench Merge)

| 기능 항목 | 현재 제어기 파라미터 | 현재 기본값 | 현재 파라미터 의미 | ArduSub 대응 파라미터 | ArduSub 의미 |
|---|---|---:|---|---|---|
| 병합 주기 | `publish_rate` | 50.0 | manual/auto 출력 병합 주기 | mode/controller loop rate | 내부 모드 갱신 주기 |
| 수동 heave override 기준 | `manual_heave_override_threshold` | 0.05 | heave stick이 크면 auto heave보다 manual 우선 | pilot input handling | 수직 조종 우선권 판단 |
| 수동 XY override 기준 | `manual_xy_override_threshold` | 0.05 | surge/sway가 크면 auto position보다 manual 우선 | pilot input handling | 수평 조종 우선권 판단 |
| 수동 yaw override 기준 | `manual_yaw_override_threshold` | 0.02 | yaw 입력이 크면 auto yaw보다 manual 우선 | pilot input handling | yaw 조종 우선권 판단 |
| 수동 입력 freshness | `manual_wrench_timeout_sec` | 0.5 | 오래된 manual command는 무효 처리 | RC timeout / failsafe | 조종 입력 유효성 판단 |

설명:
- 이 영역은 ArduSub에서 보통 **독립 파라미터 집합으로 드러나지 않고 mode 로직 내부**에 포함됩니다.
- 현재 제어기는 이 동작을 `wrench_merger` 노드에서 명시적으로 분리해서 다룹니다.

### 10.6 Thruster allocation / 출력 분배

| 기능 항목 | 현재 제어기 파라미터 | 현재 기본값 | 현재 파라미터 의미 | ArduSub 대응 파라미터 | ArduSub 의미 |
|---|---|---:|---|---|---|
| heave gain | `heave_gain` | 1.4 | 수직 추력 명령의 전체 배율 | `FRAME_CONFIG` + motor scaling 조합 | frame별 수직 추진 배분 |
| 수평 출력 gain | `horizontal_output_gain` | 2.5 | 수평 thruster 그룹 출력 배율 | frame/motor mixer 내부 | 수평 추진 강도 반영 |
| yaw 출력 gain | `yaw_output_gain` | 0.35 | yaw 성분 배율 | `MOT_YAW_HEADROOM`과 부분적으로 유사 | yaw 권한 확보 |
| 수직 출력 gain | `vertical_output_gain` | 3.0 | 수직 thruster 그룹 출력 배율 | frame/motor scaling | 수직 추진 강도 반영 |
| 후방 수직 bias | `rear_vertical_bias` | 0.0 | 후방 수직 thruster 가중치 보정 | 직접 1:1 없음 | 기체별 모터 편향 보정 |
| pitch torque gain | `pitch_torque_gain` | 1.5 | pitch 토크 성분 배율 | frame/motor mixer 내부 | pitch 제어 권한 |
| 토크 우선 배분 | `torque_first_allocation` | True | heave보다 attitude torque를 먼저 예산 배분 | 직접 1:1 없음 | 현재 제어기 고유 정책 |
| 출력 slew rate | `slew_rate` | 4.0 | thruster 명령 변화 속도 제한 | `MOT_SLEWRATE` 계열 | 모터 출력 급변 억제 |
| 최대 출력 | `max_output` | 1.0 | thruster 명령 절대 상한 | 모터 출력 한계 | 출력 saturation 상한 |
| 전체 출력 스케일 | `output_scale` | 0.25 | 전체 thruster 출력 축소 배율 | joystick gain/motor scaling 조합 | 전체 출력 레벨 제한 |
| 출력 deadband | `output_deadband` | 0.02 | 작은 thruster command는 0 처리 | `MOT_THST_EXPO`와는 목적 다름 | 미세 떨림 억제 |
| 수평 레벨 보상 사용 | `level_horizontal_compensation_enabled` | True | 기체 기울기 시 수평 thruster 보정 | 직접 1:1 없음 | 현재 제어기 커스텀 보상 |
| 수평 레벨 보상 gain | `level_horizontal_compensation_gain` | 1.0 | 수평 보상 강도 | 직접 1:1 없음 | 기울기 보상 강도 |
| 수평 레벨 보상 최대치 | `level_horizontal_compensation_max` | 0.5 | 수평 보상 최대량 | 직접 1:1 없음 | 보상 saturation |
| 수평 레벨 보상 최소 z | `level_horizontal_compensation_min_z` | 0.35 | 일정 이상 기울어져야 보상 시작 | 직접 1:1 없음 | 보상 활성 조건 |
| spare thrust만 사용 | `level_horizontal_compensation_uses_spare_only` | True | 남는 추력 범위 안에서만 보상 | 직접 1:1 없음 | 현재 제어기 고유 정책 |
| surge-pitch 보상 사용 | `surge_pitch_moment_compensation_enabled` | False | 전진 시 pitch 모멘트 보상 | 직접 1:1 없음 | 현재 제어기 고유 보상 |
| surge-pitch 보상 gain | `surge_pitch_moment_gain` | -0.12 | 전진에 따른 pitch 보상량 | 직접 1:1 없음 | 보상 크기 |
| IMU pitch hold 보상 사용 | `imu_pitch_hold_compensation_enabled` | False | pitch 자세를 allocator 단에서 추가 보정 | 직접 1:1 없음 | 현재 제어기 커스텀 보상 |
| IMU pitch hold gain | `imu_pitch_hold_gain` | 0.25 | pitch 보상 강도 | 직접 1:1 없음 | 보상 이득 |

### 10.7 Arm / Enable / 운용 상태

| 기능 항목 | 현재 제어기 파라미터 | 현재 기본값 | 현재 파라미터 의미 | ArduSub 대응 파라미터 | ArduSub 의미 |
|---|---|---:|---|---|---|
| 자세 제어 enable | `control_enabled` (`attitude_controller`) | True | 자세 제어기 자체 활성화 | mode 선택 | 자세 안정화 사용 여부 |
| 깊이 제어 enable | `control_enabled` (`depth_controller`) | True | depth hold 사용 여부 | `ALT_HOLD` 모드 | depth hold 사용 여부 |
| 위치 제어 enable | `control_enabled` (`position_controller`) | False (sim) | position hold 사용 여부 | `POSHOLD` 모드 | XY hold 사용 여부 |
| arm 상태 토픽 | `armed_topic` | `/rov/armed` | 전체 출력 허용/차단 상태 | arming state | 모터 출력 허용 여부 |

설명:
- ArduSub은 보통 **모드 전환**이 곧 controller 활성 조합을 결정합니다.
- 현재 제어기는 모드 전환기보다 **노드별 `control_enabled`와 `armed` 토픽**으로 기능 조합을 만듭니다.

### 10.8 요약

| 비교 축 | 현재 제어기 특징 | ArduSub 특징 |
|---|---|---|
| 파라미터 naming | 기능 설명형 커스텀 이름 | 오토파일럿 체계형 이름 (`ATC_*`, `PSC_*`, `JS_*`, `FRAME_*`) |
| 자세 제어 | angle/rate/보호로직이 한 노드에 강하게 결합 | angle loop / rate loop / mode 구조가 더 체계적으로 분리 |
| 깊이 제어 | ArduSub식 조종 감각과 매우 유사 | mode 기반 depth hold가 내장 |
| 위치 제어 | DVL 기반 XY hold에 집중 | navigation/mode 체계 안에 포함 |
| 수동/자동 병합 | `wrench_merger`로 명시적 구현 | mode/controller 내부 로직으로 흡수 |
| 추력 분배 | 전용 8-thruster 커스텀 allocator | frame 기반 일반화된 mixer |
| 현장 튜닝 감각 | 직관적이고 빠름 | 체계적이지만 시스템 전체 영향이 큼 |


### 10.9 ArduSub에는 있고 현재 제어기에는 없는 항목

이 표는 **ArduSub 쪽에는 명시적 기능/파라미터/모드 체계가 있는데, 현재 Robster V5 제어기에는 없거나 매우 약한 부분**을 정리한 것입니다.

| ArduSub 측 항목 | ArduSub 파라미터/모드 예시 | ArduSub에서 하는 일 | 현재 제어기 상태 | 비고 |
|---|---|---|---|---|
| 운용 모드 체계 | `MANUAL`, `ACRO`, `STABILIZE`, `ALT_HOLD`, `POSHOLD`, `AUTO`, `GUIDED`, `CIRCLE`, `SURFACE`, `SURFTRAK` | 조종/자동화 상태를 명확한 mode로 관리 | 없음 | 현재는 `armed` + 각 controller `control_enabled` 조합으로 비슷한 동작을 만듦 |
| 완전한 미션 운항 | `AUTO`, waypoint 관련 파라미터들 | waypoint 따라가기, 자동 경로 실행 | 없음 | 현재 코드에는 미션 실행기 없음 |
| 외부 유도 모드 | `GUIDED` | GCS/컴패니언에서 목표점 지시 | 부분적 | ROS에서 topic 명령은 가능하지만 ArduSub식 guided mode 상태기는 없음 |
| 수면 복귀 모드 | `SURFACE` | 강제로 상승해 수면으로 복귀 | 없음 | 현재는 depth target을 올려서 유사 동작만 가능 |
| 바닥 추종 모드 | `SURFTRAK` | 바닥 지형을 따라 일정 고도 유지 | 없음 | 현재는 depth hold만 있고 seabed tracking 없음 |
| 내장 failsafe 체계 | battery / leak / RC / comm failsafe 파라미터군 | 이상 상황에서 자동 감속, hold, surface, disarm 등 수행 | 매우 약함 | 현재는 사실상 `armed` gating 중심 |
| 누수 감지 처리 | `FS_LEAK_ENABLE` 계열 | leak sensor 입력 시 자동 대응 | 없음 | 누수 센서 처리 파이프라인 없음 |
| 배터리 failsafe | `BATT_*`, `FS_BATT_*` | 저전압/저용량 시 자동 대응 | 없음 | 전원 상태 기반 안전 동작 없음 |
| 통신 두절 failsafe | `FS_GCS_*`, RC failsafe 계열 | 조종기/GCS 손실 시 자동 대응 | 매우 약함 | manual topic timeout은 있으나 시스템 failsafe는 아님 |
| EKF 기반 통합 상태추정 | `EK3_*`, `AHRS_*` | IMU/압력/DVL/GPS 등 융합하여 자세/속도/위치 추정 | 없음 | 현재는 controller별로 센서를 직접 사용 |
| 내장 heading/position/navigation stack | `EK3_*`, `PSC_*`, `WPNAV_*` | 위치/속도/경로 제어를 공통 stack으로 수행 | 약함 | 현재는 depth/position/attitude가 분리된 로컬 제어기 |
| GCS 중심 파라미터/상태 관리 | Mission Planner / QGroundControl 연동 | 파라미터 저장, calibration, mode 변경, 로그 확인 | 약함 | 현재는 ROS2 파라미터 + GUI 수동 조작 |
| 센서 캘리브레이션 체계 | accel/gyro/compass calibration 파라미터/절차 | 기체 레벨 센서 보정 | 없음 | 현재 코드에는 캘리브레이션 워크플로우가 없음 |
| arming check 체계 | `ARMING_*` 계열 | arm 전 센서/전원/상태 체크 | 거의 없음 | 현재는 arm 토픽만 맞으면 출력 가능 |
| RC 입력 표준화 | `RCx_*`, `BTNx_*`, joystick 관련 설정 | 다양한 조종기 입력을 표준 체계로 관리 | 부분적 | 현재는 teleop 노드 내부 파라미터에 직접 묶임 |
| 모터/프레임 preset 체계 | `FRAME_CONFIG` | 표준 ROV frame에 맞춰 mixer 적용 | 약함 | 현재는 allocator 내부에 현재 기체 geometry가 직접 박혀 있음 |
| 출력 선형화/모터 특성 표준화 | `MOT_THST_EXPO`, `MOT_SPIN_*`, `MOT_PWM_*` | 모터 반응 곡선/최소출력/PWM 범위 정의 | 없음 또는 매우 약함 | 현재는 `output_scale`, gain, deadband 위주 |
| 데이터플래시 로그 체계 | `LOG_*` | 비행/잠항 로그 저장 및 사후 분석 | 없음 | ROS bag으로 대체는 가능하지만 내장 체계는 없음 |
| 자동 튜닝/표준 튜닝 절차 | autotune 관련 기능/절차 | 제어기 튜닝 지원 | 없음 | 현재는 수동 튜닝 중심 |
| 지오펜스/운용 한계 | fence 계열 파라미터 | 깊이/영역 제한 | 없음 | 현재는 `min_target_depth`, `max_target_depth` 정도만 있음 |
| 절대 위치 센서 연동 확장 | GPS, beacon, rangefinder, vision 계열 | 다양한 외부 항법 센서 통합 | 약함 | 현재는 주로 IMU/DVL/depth 중심 |
| 시스템 상태 기반 자동 행동 | 각종 FS/mission/mode logic | 상태에 따라 모드 자동 전환 | 없음 | 현재는 operator/GUI가 직접 개입해야 함 |

### 10.10 현재 제어기에 일부는 있지만 ArduSub보다 약한 항목

| 항목 | 현재 제어기 상태 | ArduSub 대비 차이 |
|---|---|---|
| 입력 신선도 관리 | `manual_wrench_timeout_sec` 있음 | 단순 stale 입력 무시 수준, 시스템 failsafe는 아님 |
| depth limit | `min_target_depth`, `max_target_depth` 있음 | fence/mission/모드 연동 제한 체계는 아님 |
| arm/disarm | `/rov/armed` 있음 | pre-arm health check, arming policy는 거의 없음 |
| position hold | DVL 기반 hold 있음 | EKF/navigation stack 통합이 없음 |
| GUI 기반 파라미터 조정 | 가능 | ArduSub의 GCS 기반 표준화/저장/캘리브레이션 체계보다 약함 |
| 추력 제한 | `output_scale`, `max_output`, `deadband` 있음 | PWM/ESC/motor 특성 수준의 표준 출력 파라미터는 부족 |

### 10.11 한 줄 결론

- **현재 제어기는 “제어 자체”는 잘 쪼개서 구현되어 있지만, ArduSub가 갖는 상위 시스템 기능(mode/failsafe/EKF/mission/GCS)은 많이 비어 있습니다.**
- 즉, **저수준 제어 커스터마이징은 현재 코드가 강하고, 시스템 완성도는 ArduSub가 훨씬 강합니다.**

