# Control Code Review

전체 제어 흐름

이 코드 구조는 여러 제어기가 각자 필요한 힘 또는 토크를 계산하고, `WrenchMerger`가 이를 하나의 `Wrench`로 병합한 뒤,
`Allocator`가 8개 스러스터 명령으로 변환하는 방식입니다.

단계 설명

- 수동 입력: 조종기 또는 상위 제어 입력이 `/rov/wrench_manual` 형태로 들어옵니다.
- 자세 제어: `attitude_controller.py`가 IMU 기반 `roll`, `pitch`, `yaw` 토크를 계산합니다.
- 수심 제어: `depth_controller.py`가 목표 수심과 현재 수심 차이로 heave 명령을 계산합니다.
- 위치 제어: `position_controller.py`가 DVL 기반 위치/속도 정보를 이용해 XY force를 계산합니다.
- 병합: `wrench_merger.py`가 수동 입력과 자동 제어 출력을 우선순위 규칙으로 합칩니다.
- 할당: `allocator_node.py`가 최종 wrench를 8개 스러스터 출력으로 변환합니다.

## 코드 리뷰 페이지
- `Attitude Controller Review`
  - Markdown: `code_review/controller_review/controller_review_attitude.md`
  - HTML: `code_review/controller_review/attitude_controller_review.html`
- `Depth Controller Review`
  - Markdown: `code_review/controller_review/controller_review_depth.md`
  - HTML: `code_review/controller_review/depth_controller_review.html`
- `Position Controller Review`
  - Markdown: `code_review/controller_review/controller_review_position.md`
  - HTML: `code_review/controller_review/position_controller_review.html`
- `Wrench Merger Review`
  - Markdown: `code_review/controller_review/controller_review_wrench_merger.md`
  - HTML: `code_review/controller_review/wrench_merger_review.html`
- `Allocator Review`
  - Markdown: `code_review/controller_review/controller_review_allocator.md`
  - HTML: `code_review/controller_review/allocator_node_review.html`
