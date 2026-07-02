# Blue Robotics(BlueROV2/ArduSub) vs 현재 ROV 기능 비교 정리

작성일: 2026-07-02

이 문서는 현재 워크스페이스의 `px4_ardusub_comparison` 문서와 분석용 코드 사본을 기준으로,
**Blue Robotics BlueROV2 + Navigator + ArduSub 생태계**와
**현재 개발 중인 ROV 제어기**의 기능적 차이를 실무 관점에서 정리한 메모입니다.

비교 기준:
- 현재 ROV: 워크스페이스 내 `code_review/code/*.py`, `px4_ardusub_comparison/px4_ardusub_comparison.md`
- Blue Robotics: BlueROV2 공식 제품 페이지
- ArduSub: ArduPilot Sub 주변장치 문서

---

## 1. 한 줄 요약

- **현재 ROV는 제어기 커스터마이징이 강점**인 구조입니다.
- **Blue Robotics는 하드웨어/주변장치/운용 소프트웨어 생태계가 강점**입니다.
- 즉, 현재 ROV는 **원하는 제어 로직을 빠르게 실험하기 좋고**,
  BlueROV2/ArduSub는 **센서, 조명, 그리퍼, 소나, failsafe까지 포함한 완성형 운용 플랫폼**에 가깝습니다.

---

## 2. 전체 성격 차이

| 항목 | 현재 ROV | Blue Robotics / ArduSub |
|---|---|---|
| 제어 구조 | ROS 2 노드 분산형 | 통합형 오토파일럿 + BlueOS + GCS |
| 강점 | 축별 제어기 분리, 커스텀 로직 삽입 쉬움 | 하드웨어 확장, 운용 기능, 주변장치 연동 풍부 |
| 튜닝 방식 | 노드별 파라미터 직접 조정 | ArduSub/BlueOS/QGC/Mission Planner 기반 |
| 운용 철학 | 기체 맞춤형 실험/개발 | 범용 플랫폼 + 옵션 확장 |
| 생태계 | 현재 프로젝트 중심 | 카메라/조명/그리퍼/소나/센서 공식 생태계 |

---

## 3. 현재 ROV에서 문서상 확인되는 기능

현재 워크스페이스 기준으로 확인되는 기능은 아래와 같습니다.

### 3.1 제어 기능

- 수동 조종 입력 생성
- 자세 안정화 (`roll/pitch/yaw`)
- yaw hold
- depth hold
- pilot depth-rate 입력 해석
- DVL + IMU 기반 XY hold
- 수동/자동 wrench 병합
- 8개 thruster 출력 할당

### 3.2 센서/입력

문서와 코드에서 직접 확인되는 것은 주로 아래입니다.

- `IMU`
- `depth sensor`
- `DVL`
- joystick / gamepad 입력

### 3.3 현재 구조의 특징

- depth sensor offset compensation이 있음
- DVL position이 없을 때 velocity 적분 fallback이 있음
- thruster allocator가 현재 기체 geometry에 강하게 최적화되어 있음
- 노드별 `control_enabled`로 기능 조합 가능

---

## 4. Blue Robotics 쪽에서 강한 부분

BlueROV2/ArduSub 생태계는 단순 제어기보다 **주변장치와 운용성**에서 훨씬 강합니다.

### 4.1 기본/내장 성격

- BlueROV2는 6-thruster 기본 구성
- Heavy kit 적용 시 8-thruster 확장 가능
- 저조도 전방 카메라 기본 제공
- 카메라 틸트 메커니즘 제공
- Navigator 기반 제어
- BlueOS 기반 운용
- ArduSub 기반 vehicle control

### 4.2 Navigator / 시스템 확장

공식 페이지 기준 Navigator는 아래 성격을 가집니다.

- `IMU` 내장
- `magnetometer(heading)` 내장
- `leak sensor` 내장
- `16 outputs` 제공
- serial / I2C 확장 포트 제공
- thrusters / lights / grippers / 기타 액세서리 연결 가능

### 4.3 주변장치 생태계

ArduSub 문서와 Blue Robotics 제품 구조상 아래 장비군이 잘 정리되어 있습니다.

- external LEDs / lights
- grippers
- cameras / gimbals
- sonar / rangefinders
- temperature sensors
- companion computers
- telemetry / control accessories

---

## 5. 센서 관점 차이

### 5.1 현재 ROV

현재 문서 기준으로는 센서 사용이 **제어 목적 중심**입니다.

- `IMU`: 자세 안정화, allocator 보정
- `depth sensor`: depth hold
- `DVL`: XY hold, 속도/위치 추정

즉, 센서가 **각 제어 노드에 직접 연결**되어 있습니다.

### 5.2 Blue Robotics / ArduSub

Blue Robotics 쪽은 센서가 **플랫폼 확장성 중심**으로 구성됩니다.

- 기본 운용용 센서: IMU, compass/magnetometer, leak sensing
- 추가 장착 가능 센서: sonar, rangefinder, temperature, external pressure/depth 계열, camera 계열 등
- 센서들이 단일 제어 노드에 바로 묶이기보다, 오토파일럿/BlueOS/주변장치 체계로 연결됨

### 5.3 차이 해석

- 현재 ROV는 **실제 제어 성능 실험**에 유리함
- Blue Robotics는 **센서 종류 다양화와 장비 추가**가 훨씬 쉬움

---

## 6. LED / 조명 관점 차이

이 부분은 사용자가 궁금해할 가능성이 높아서 따로 정리합니다.

### 6.1 Blue Robotics

공식 BlueROV2 페이지 기준:

- 조명 옵션은 **2-light 또는 4-light 구성**
- 최대 밝기는 **up to 6,000 lumens**

즉, **패키지 옵션 기준으로는 최대 4개 조명**까지 명확히 보입니다.

또한 Navigator는 `16 outputs`가 있으므로, 하드웨어적으로는 lights/grippers 등 액세서리 제어 여지가 큽니다.
다만 **BlueROV2 기본 상품 옵션으로 명시된 것은 2개 또는 4개 조명 구성**입니다.

### 6.2 현재 ROV

현재 워크스페이스 문서/코드 기준:

- LED 제어 노드가 보이지 않음
- light 출력 개수 정의가 보이지 않음
- 조명 밝기, dimming, 개수 제한에 대한 명시가 없음

즉 현재 자료 기준으로는:

- **LED를 지원하지 않는다고 단정할 수는 없지만**
- **문서상 확인 가능한 LED 기능은 아직 없음**

### 6.3 실무 해석

- Blue Robotics는 조명을 **제품 옵션**으로 바로 고를 수 있음
- 현재 ROV는 조명은 아마 별도 하드웨어/드라이버/출력 매핑을 추가해야 할 가능성이 큼

---

## 7. Thruster / 출력 관점 차이

### 7.1 현재 ROV

- allocator가 **8개 thruster 출력**을 전제로 함
- 수평/수직 thruster를 분리해서 계산함
- 수평 보상, pitch 보상, heave 보상 등 기체 맞춤 로직이 많음

즉, 현재 ROV의 allocator는 **현재 기체에 매우 특화된 전용 설계**입니다.

### 7.2 BlueROV2

- 기본 6 thruster
- Heavy kit 시 8 thruster
- ArduSub/Navigator 기반 출력 사용
- 일반화된 frame/mixer/출력 생태계 사용

### 7.3 차이 해석

- 현재 ROV: **출력 품질과 제어감 튜닝**에 강함
- BlueROV2: **기본 프레임 + 확장성 + 표준화**에 강함

---

## 8. 카메라 / 소나 / 그리퍼 관점 차이

### 8.1 Blue Robotics

공식 페이지와 ArduSub 주변장치 구조상 아래가 명확합니다.

- 전방 저조도 HD 카메라 기본 제공
- 카메라 틸트 제공
- 외부 카메라 확장 가능
- 그리퍼 확장 가능
- 스캐닝 소나 확장 가능
- 다양한 센서/카메라/액세서리 카테고리가 이미 정리되어 있음

### 8.2 현재 ROV

현재 워크스페이스 기준으로는:

- 카메라 처리 파이프라인 문서 없음
- 그리퍼 제어 노드 없음
- 소나/스캐닝 소나 연동 구조 없음

즉, 현재 자료 기준으로는 이 부분이 **제어기 바깥 영역**으로 남아 있습니다.

---

## 9. 안전 / 운용 기능 차이

이 부분은 Blue Robotics/ArduSub가 현재 ROV보다 훨씬 강합니다.

### 9.1 Blue Robotics / ArduSub 쪽에 있는 기능

- `AUTO`
- `GUIDED`
- `SURFACE`
- `SURFTRAK`
- battery failsafe
- leak failsafe
- RC / GCS / 통신 failsafe
- EKF 기반 상태추정
- GCS 기반 파라미터/로그/모드 관리

### 9.2 현재 ROV

현재 자료 기준:

- `armed` gating 중심
- 노드별 `control_enabled` 조합 중심
- mission 모드 없음
- leak 처리 없음
- battery failsafe 없음
- EKF 통합 상태추정 없음

### 9.3 차이 해석

- Blue Robotics/ArduSub는 **운용 안정성**
- 현재 ROV는 **제어 실험 자유도**

쪽에 더 무게가 실려 있습니다.

---

## 10. 항목별 요약표

| 항목 | Blue Robotics / ArduSub | 현재 ROV |
|---|---|---|
| 기본 IMU | 있음 | 있음 |
| depth hold | 있음 | 있음 |
| DVL 기반 XY hold | 확장/통합 가능 | 있음 |
| magnetometer / heading 센서 | 있음 | 현재 자료상 없음 |
| leak sensor | 있음 | 현재 자료상 없음 |
| battery failsafe | 있음 | 현재 자료상 없음 |
| camera 기본 제공 | 있음 | 현재 자료상 없음 |
| camera tilt | 있음 | 현재 자료상 없음 |
| gripper 확장 | 있음 | 현재 자료상 없음 |
| sonar 확장 | 있음 | 현재 자료상 없음 |
| lights 옵션 | 2개 또는 4개 | 현재 자료상 확인 안 됨 |
| lights 밝기 | 최대 6,000 lumens | 현재 자료상 확인 안 됨 |
| accessory outputs | 16 outputs | 현재 자료상 명시 없음 |
| thruster 구성 | 6기본 / 8확장 | 8출력 전제 |
| mission mode | 있음 | 없음 |
| surface / failsafe 체계 | 강함 | 약함 |
| 제어기 커스터마이징 | 중간 | 매우 강함 |

---

## 11. 결론

현재 ROV와 Blue Robotics의 가장 큰 차이는 아래처럼 정리할 수 있습니다.

### 현재 ROV가 더 강한 점

- 제어 로직을 마음대로 바꾸기 쉬움
- DVL/depth/attitude를 기체 특성에 맞게 세밀하게 튜닝하기 쉬움
- allocator가 현재 8-thruster 구조에 최적화되어 있음

### Blue Robotics가 더 강한 점

- 카메라, LED, 그리퍼, 소나, leak sensor 같은 **주변장치 생태계**
- ArduSub/BlueOS 기반 **운용 모드와 failsafe**
- 확장 포트와 출력 수가 명확해서 **하드웨어 확장 계획 세우기 쉬움**

### 실무적으로 보면

- **제어 알고리즘 개발/실험 플랫폼**으로는 현재 ROV가 좋음
- **완성형 운용 플랫폼**으로는 BlueROV2/ArduSub가 훨씬 정리되어 있음

---

## 12. 참고

로컬 참고 문서:
- `px4_ardusub_comparison/px4_ardusub_comparison.md`
- `code_review/code/depth_controller.py`
- `code_review/code/position_controller.py`
- `code_review/code/allocator_node.py`

외부 참고 문서:
- BlueROV2 공식 페이지: `https://bluerobotics.com/store/rov/bluerov2/`
- ArduPilot Sub Peripheral Hardware: `https://ardupilot.org/sub/docs/common-optional-hardware.html`
