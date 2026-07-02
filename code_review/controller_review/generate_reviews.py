from __future__ import annotations

import html
import re
import textwrap
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


BASE_DIR = Path(__file__).resolve().parent
CODE_DIR = BASE_DIR.parent / "code"


@dataclass(frozen=True)
class ReviewItem:
    heading: str
    source_names: tuple[str, ...]
    meaning: str
    impact: str
    review: str
    details: str = ""
    parameter_notes: tuple[tuple[str, str], ...] = ()


@dataclass(frozen=True)
class ModuleConfig:
    key: str
    title: str
    source_filename: str
    markdown_filename: str
    html_filename: str
    role_summary: str
    design_summary: str
    review_focus: str
    runtime_summary: str
    parameters: tuple[tuple[str, str], ...]
    items: tuple[ReviewItem, ...]


MODULES: tuple[ModuleConfig, ...] = (
    ModuleConfig(
        key="attitude",
        title="Attitude Controller Review",
        source_filename="attitude_controller.py",
        markdown_filename="controller_review_attitude.md",
        html_filename="attitude_controller_review.html",
        role_summary=(
            "이 코드는 IMU와 수동 wrench를 받아 ROV의 `roll`, `pitch`, `yaw` 토크를 만드는 자세 제어기입니다."
        ),
        design_summary=(
            "설계상 핵심은 `roll/pitch stabilizer + yaw heading hold`입니다. "
            "즉 단순히 자세 오차만 줄이는 것이 아니라, 조종자의 yaw 조작과 translational motion 중에도 "
            "trim attitude와 heading hold를 유지하려는 운용 감각이 강하게 들어가 있습니다."
        ),
        review_focus=(
            "리뷰 포인트는 `목표 자세를 어떻게 잡는지`, `IMU를 어떻게 제어 상태로 필터링하는지`, "
            "`yaw manual override와 hold가 어떻게 전환되는지`, `translation/heave 중 보호 로직이 어떤 출력을 만드는지`입니다."
        ),
        runtime_summary=(
            "런타임에서는 `imu_callback()`이 현재 자세와 각속도를 계속 갱신하고, "
            "`control_loop()`가 주기적으로 목표 자세와 현재 자세의 차이를 계산해 토크를 냅니다. "
            "여기에 yaw stick release 이후 heading을 다시 잠그는 로직, translation 중 roll/pitch trim을 유지하는 로직, "
            "heave나 큰 body rate 상황에서 출력을 보호하는 로직이 겹쳐서 실제 조종 감각을 만듭니다."
        ),
        parameters=(
            ("`kp_roll`, `ki_roll`, `kd_roll`", "roll 축 복원력, steady-state bias 보상, roll rate damping 강도를 정합니다."),
            ("`kp_pitch`, `ki_pitch`, `kd_pitch`", "pitch 축 응답과 감쇠를 정하며 surge 중 nose-up 경향을 얼마나 강하게 잡을지에도 영향을 줍니다."),
            ("`kp_yaw`, `kd_yaw`", "heading error를 얼마나 세게 복원할지와 yaw rate를 얼마나 감쇠할지 결정합니다."),
            ("`yaw_manual_override_threshold`", "이 값보다 큰 yaw stick이 들어오면 yaw hold보다 manual yaw를 우선합니다."),
            ("`yaw_hold_settle_time_sec`", "yaw stick을 놓은 직후 heading을 다시 잠그기 전에 잠깐 damping만 적용하는 시간입니다."),
            ("`xy_motion_protect_threshold`, `strong_xy_motion_threshold`", "translation 중 roll/pitch trim hold를 약하게 만들기 시작하는 기준입니다."),
            ("`rp_scale_when_xy_motion`, `rp_scale_when_strong_xy_motion`", "translation 중 roll/pitch torque를 얼마나 줄일지 정합니다."),
            ("`heave_protect_threshold`, `strong_heave_threshold`", "수직 조작 중 yaw hold를 더 보수적으로 만들지 결정하는 기준입니다."),
            ("`large_tilt_disable_deg`", "기체가 너무 크게 기울면 torque 출력을 끊는 안전 게이트입니다."),
            ("`orientation_filter_measurement_alpha`, `orientation_filter_max_correction_rate_deg`", "IMU 기반 control attitude filter의 추종 속도와 correction 한계를 정합니다."),
        ),
        items=(
            ReviewItem(
                heading="Quaternion / Angle 유틸리티",
                source_names=("clamp", "vec_norm", "quat_normalize", "quat_conj", "quat_mul", "quat_to_rpy", "rpy_to_quat", "wrap_to_pi"),
                meaning=(
                    "이 함수들은 제어기 본체보다 먼저, 자세 표현을 다루기 위한 공통 도구입니다. "
                    "Quaternion을 정규화하고 곱하고 Euler angle로 바꾸는 역할을 맡습니다."
                ),
                impact=(
                    "이 유틸리티가 안정적이어야 목표 자세 계산과 yaw wrapping이 깨지지 않습니다. "
                    "특히 `wrap_to_pi()`는 yaw 오차가 `+π/-π` 경계에서 튀는 문제를 막아줍니다."
                ),
                review=(
                    "이 계층은 제어식의 기반이라서 작아 보여도 중요합니다. "
                    "자세 제어가 갑자기 반대로 튀거나 yaw 오차가 크게 보이는 문제는 여기서 시작되는 경우가 많습니다."
                ),
                details=(
                    "이 유틸리티 묶음은 attitude controller가 어떤 좌표 표현을 믿고 있는지 보여주는 기반입니다. "
                    "`quat_normalize()`는 센서 quaternion의 수치 안정성을 확보하고, `quat_mul()`과 `quat_conj()`는 목표 자세와 현재 자세의 관계를 계산할 수 있게 해 줍니다. "
                    "`quat_to_rpy()`와 `rpy_to_quat()`는 사람이 이해하기 쉬운 roll/pitch/yaw와 내부 quaternion 표현을 오가는 다리 역할을 합니다.\n\n"
                    "특히 `wrap_to_pi()`는 yaw hold에서 필수적입니다. heading target이 179도이고 현재 yaw가 -179도일 때, 이 함수를 거치지 않으면 오차를 358도로 해석해 잘못된 큰 토크가 나올 수 있습니다. "
                    "즉 이 계층은 '작은 보조 함수'가 아니라 제어기의 오차 정의와 좌표계 일관성을 지키는 기반 계층입니다."
                ),
            ),
            ReviewItem(
                heading="`__init__()`",
                source_names=("__init__",),
                meaning=(
                    "노드의 토픽, 게인, 제한값, 보호 로직, yaw hold 정책을 전부 선언하고 상태 변수를 초기화합니다. "
                    "즉 이 함수는 단순 constructor가 아니라 이 controller가 어떤 철학으로 동작할지 정의하는 설정 진입점입니다."
                ),
                impact=(
                    "이 함수가 곧 제어기의 운용 정책 선언부입니다. "
                    "어떤 입력을 받고 어떤 보호 로직이 켜지는지, yaw stick을 놓았을 때 어떤 감각으로 복귀하는지가 여기서 결정됩니다."
                ),
                review=(
                    "파라미터가 많지만 모두 의미가 분명합니다. 특히 이 함수 하나만 읽어도 "
                    "이 자세 제어기가 `단순 자세 PID`가 아니라 `manual yaw override`, `trim hold`, `translation 보호`, `orientation filtering`을 가진 "
                    "운용형 controller라는 점을 알 수 있습니다. 다만 수동 입력 freshness를 위한 timestamp/state가 없는 점은 "
                    "후반 제어 루프에서 stale manual state로 이어질 수 있습니다."
                ),
                details=(
                    "이 함수에서 먼저 보아야 할 것은 파라미터가 어떤 덩어리로 나뉘는지입니다. "
                    "첫 번째 덩어리는 `imu_topic`, `manual_wrench_topic`, `output_torque_topic` 같은 입출력 토픽입니다. "
                    "두 번째 덩어리는 `kp_*`, `ki_*`, `kd_*`로 대표되는 기본 제어 게인입니다. "
                    "세 번째 덩어리는 `tx_limit`, `ty_limit`, `tz_limit`, `rp_torque_slew_rate` 같은 출력 shaping / safety 관련 파라미터입니다. "
                    "마지막으로 이 코드의 성격을 가장 잘 보여주는 것은 `yaw_hold_enabled`, `capture_yaw_target_on_release`, "
                    "`xy_motion_protect_threshold`, `translation_tilt_ff_*`, `orientation_filter_*` 같은 운용 정책 파라미터입니다.\n\n"
                    "즉 이 함수는 단순히 숫자를 로드하는 함수가 아니라, 기체를 어떤 감각으로 조종할 것인지 선언하는 정책 테이블입니다. "
                    "그래서 이 함수 설명이 빈약하면 전체 코드 해석이 얕아질 수밖에 없습니다."
                ),
                parameter_notes=(
                    ("`imu_topic`, `manual_wrench_topic`, `output_torque_topic`", "센서 입력, pilot 입력, 최종 torque 출력이 어디를 통해 흐르는지 정합니다."),
                    ("`kp_roll/pitch/yaw`, `ki_roll/pitch`, `kd_roll/pitch/yaw`", "자세 오차에 대한 복원력과 damping의 기본 강도를 정하는 핵심 제어 파라미터입니다."),
                    ("`tx_limit`, `ty_limit`, `tz_limit`", "각 축 torque saturation 상한입니다. 기체를 세우는 힘보다 actuator 보호를 우선할 때 중요합니다."),
                    ("`rp_torque_slew_rate`", "roll/pitch torque가 한 주기에서 얼마나 급하게 바뀔 수 있는지 제한합니다."),
                    ("`yaw_hold_enabled`, `capture_yaw_target_on_release`, `yaw_hold_settle_time_sec`", "yaw stick을 놓은 뒤 heading hold가 어떤 감각으로 복귀할지 결정합니다."),
                    ("`xy_motion_protect_threshold`, `strong_xy_motion_threshold`, `rp_scale_when_*`", "translation 중 자세 hold를 얼마나 약하게 또는 강하게 유지할지 결정합니다."),
                    ("`orientation_filter_*`", "IMU 자세를 control attitude로 쓸 때 얼마나 부드럽게 필터링할지 정합니다."),
                ),
            ),
            ReviewItem(
                heading="목표 자세 캡처 계열",
                source_names=("_update_target_quaternion", "_force_level_roll_pitch_target", "_capture_current_attitude_as_target", "_capture_current_yaw_as_target"),
                meaning=(
                    "이 함수들은 '지금 무엇을 목표 자세로 볼 것인가'를 정하는 계층입니다. "
                    "초기 캡처, level target 유지, yaw release 후 heading hold 재설정이 모두 여기서 일어납니다."
                ),
                impact=(
                    "사용자가 조작을 놓았을 때 기체가 어느 자세로 돌아갈지 결정합니다. "
                    "운용감이 좋은지, trim이 유지되는지, yaw hold가 자연스러운지가 이 계층에 달려 있습니다."
                ),
                review=(
                    "이 코드의 운용 감각은 꽤 좋습니다. 특히 yaw target을 현재 yaw로 다시 잡는 설계는 실제 조종 감각을 부드럽게 만듭니다."
                ),
                details=(
                    "이 함수들은 모두 'target attitude를 언제, 무엇으로 재정의할 것인가'에 답하는 계층입니다. "
                    "`_capture_current_attitude_as_target()`은 현재 기체 자세를 그대로 목표로 잠그고, `_capture_current_yaw_as_target()`은 yaw만 현재값으로 다시 잡습니다. "
                    "`_force_level_roll_pitch_target()`은 roll/pitch를 level 또는 trim 기준으로 되돌리는 강한 정책 함수입니다.\n\n"
                    "운용 관점에서 중요한 점은, 이 계층이 단순한 초기화 코드가 아니라 control loop가 참조하는 목표 상태를 직접 바꾼다는 것입니다. "
                    "따라서 기체가 '왜 지금 이 자세를 목표로 삼고 있는지'를 이해하려면 PID 게인보다 먼저 이 계층을 봐야 합니다."
                ),
                parameter_notes=(
                    ("`capture_initial_target`", "첫 IMU 수신 시 현재 자세를 목표로 잡을지 결정합니다."),
                    ("`level_roll_pitch_target`, `target_roll_deg`, `target_pitch_deg`", "roll/pitch target을 항상 수평 또는 지정된 trim 기준으로 강제할지 정합니다."),
                    ("`roll_trim_deg`, `pitch_trim_deg`", "완전한 level 대신 운용상 필요한 기준 기울기를 target에 더합니다."),
                    ("`capture_yaw_target_on_release`", "manual yaw를 놓은 뒤 현재 heading을 다시 잠글지 정합니다."),
                ),
            ),
            ReviewItem(
                heading="`imu_callback()`",
                source_names=("imu_callback",),
                meaning=(
                    "IMU quaternion과 body rate를 읽어 현재 자세와 각속도를 최신 상태로 갱신합니다."
                ),
                impact=(
                    "제어기의 모든 피드백이 여기서 들어옵니다. "
                    "처음 IMU를 받으면 초기 target을 캡처하기 때문에, 이 함수는 센서 수신과 target initialization을 동시에 담당합니다."
                ),
                review=(
                    "초기 target capture와 IMU 상태 갱신이 한 곳에 모여 있어 흐름은 명확합니다. "
                    "다만 manual freshness와는 독립이라 이후 control loop가 오래된 manual 입력을 그대로 읽을 수 있습니다."
                ),
                details=(
                    "이 함수는 attitude controller의 센서 입력 관문입니다. quaternion에서 현재 roll/pitch/yaw를 해석하고, body rate를 읽어 control loop가 사용할 최신 피드백 상태를 갱신합니다. "
                    "또한 첫 IMU가 들어왔을 때 target attitude를 초기화하는 역할도 함께 담당합니다.\n\n"
                    "즉 이 함수는 단순한 센서 수신기가 아니라, '제어를 시작할 준비가 되었는가'를 판단하는 초기화 관문이기도 합니다. "
                    "IMU 데이터가 흔들리거나 지연되면 이후 제어 오차 계산 전체가 흔들리므로, 디버깅 시에는 토크 출력보다 먼저 이 함수가 갱신하는 상태를 확인해야 합니다."
                ),
            ),
            ReviewItem(
                heading="`_update_control_attitude_filter()`",
                source_names=("_reset_control_attitude_filter", "_update_control_attitude_filter"),
                meaning=(
                    "IMU 자세를 그대로 쓰지 않고, body rate 적분 예측값과 measurement를 섞어서 `control_roll`, `control_pitch`를 만듭니다."
                ),
                impact=(
                    "센서 노이즈와 순간 흔들림을 줄이고 제어 대상 자세를 더 부드럽게 만듭니다. "
                    "즉 이 함수는 제어 입력의 품질을 개선하는 내부 observer 역할을 합니다."
                ),
                review=(
                    "이 필터는 attitude hold를 덜 거칠게 만들어 주는 좋은 계층입니다. "
                    "또한 `dt`가 비정상이면 reset하도록 만들어 센서 타이밍 문제에 비교적 안전합니다."
                ),
                details=(
                    "이 함수는 IMU measurement를 곧바로 제어 오차에 넣지 않고, 예측과 보정을 섞어 조금 더 매끈한 `control attitude`를 만듭니다. "
                    "body rate를 적분해 예측한 자세를 만들고, measurement와의 차이를 제한된 correction rate로만 따라가게 해 순간적인 센서 튐이 바로 torque 튐으로 이어지지 않게 합니다.\n\n"
                    "결국 이 계층은 observer에 가까운 역할을 합니다. 측정값의 진실성은 유지하되, 제어 대상은 조금 더 조종 친화적인 상태로 바꿔 쓰는 구조입니다."
                ),
                parameter_notes=(
                    ("`orientation_filter_enabled`", "필터를 켤지, IMU 자세를 그대로 쓸지 결정합니다."),
                    ("`orientation_filter_measurement_alpha`", "measurement correction 비중을 정합니다."),
                    ("`orientation_filter_max_correction_rate_deg`", "한 주기에서 measurement 쪽으로 얼마나 빠르게 끌려갈지 제한합니다."),
                ),
            ),
            ReviewItem(
                heading="`_translation_tilt_feedforward()`",
                source_names=("_translation_tilt_feedforward",),
                meaning=(
                    "수동 surge/sway가 있을 때 미리 roll/pitch 토크를 보정해 translation 중 trim 자세 붕괴를 줄이려는 feedforward 함수입니다."
                ),
                impact=(
                    "이 함수가 켜지면 단순 PID 복원보다 더 적극적으로 자세를 유지하려고 합니다. "
                    "특히 기체가 이동하면서 생기는 pitch-up / roll-off 경향을 선제적으로 누를 수 있습니다."
                ),
                review=(
                    "기능 자체는 고급스럽지만, 튜닝 의존성이 큽니다. "
                    "gain이 과하면 조종감이 뻣뻣해질 수 있어 실제 운용 로그와 함께 봐야 합니다."
                ),
                details=(
                    "이 함수는 translation 명령이 들어왔을 때 발생할 것으로 예상되는 기체 기울어짐을 PID가 뒤늦게 복구하기 전에 미리 보상하는 계층입니다. "
                    "예를 들어 전진 thrust가 들어가면 nose-up 경향이 있다면, surge 입력만 보고 바로 pitch 반대 토크를 조금 넣어 줄 수 있습니다.\n\n"
                    "이런 feedforward는 잘 맞으면 조종감이 훨씬 단단해지지만, 잘못 맞으면 사용자가 의도하지 않은 자동 개입처럼 느껴질 수 있습니다. "
                    "그래서 이 함수는 수학적으로보다 운용감 측면에서 더 신중하게 설명되어야 하는 구간입니다."
                ),
                parameter_notes=(
                    ("`translation_tilt_ff_enabled`", "translation 기반 선행 보상을 사용할지 정합니다."),
                    ("`translation_pitch_ff_gain`, `translation_roll_ff_gain`", "surge/sway 입력이 각각 얼마나 pitch/roll 보상으로 연결될지 정합니다."),
                    ("`translation_tilt_ff_max`", "feedforward 토크 최대치를 제한합니다."),
                    ("`translation_tilt_ff_deadband`", "작은 조작에서는 feedforward가 개입하지 않도록 합니다."),
                ),
            ),
            ReviewItem(
                heading="`control_loop()`",
                source_names=("control_loop",),
                meaning=(
                    "실제 제어법칙이 수행되는 중심 함수입니다. "
                    "자세 오차 계산, 적분, rate damping, yaw manual override, translation/heave 보호, slew limit, 최종 토크 publish가 모두 여기에 있습니다."
                ),
                impact=(
                    "이 함수의 동작이 곧 조종감입니다. "
                    "조종자가 translation 중인지, heave를 주는지, yaw stick을 놓았는지에 따라 토크 출력 정책이 바뀝니다."
                ),
                review=(
                    "구조는 매우 좋고 운용 의도도 분명합니다. "
                    "다만 manual wrench freshness가 없어서 `yaw_manual_active`, `heave_protect`, `xy_soft_trim`이 오래 남을 수 있는 리스크가 있습니다."
                ),
                details=(
                    "이 함수는 실제로 여러 개의 작은 상태 머신이 겹쳐진 형태입니다. "
                    "먼저 IMU/target/control_enabled 상태를 보고 제어를 수행할지 결정합니다. "
                    "그 다음 roll/pitch는 angle error + integral + rate damping으로 계산하고, yaw는 heading hold 상태인지 manual yaw 상태인지에 따라 완전히 다른 정책을 탑니다. "
                    "여기에 translation 중 trim 유지 약화, heave 중 yaw 보호, large tilt disable, body-rate limit, slew-rate 제한이 순서대로 겹칩니다.\n\n"
                    "그래서 이 함수는 단순 PID 함수가 아니라, 실제 조종 감각을 조합하는 중앙 orchestration 함수라고 보는 편이 정확합니다. "
                    "문제 분석 시에도 '게인이 이상하다'보다 '지금 어떤 보호 모드가 켜졌는가'를 먼저 봐야 하는 이유가 여기 있습니다."
                ),
            ),
            ReviewItem(
                heading="`on_parameter_update()`",
                source_names=("on_parameter_update",),
                meaning=(
                    "런타임 파라미터 갱신을 받아 제어기 상태와 게인을 즉시 바꾸는 함수입니다."
                ),
                impact=(
                    "GUI나 runtime tuning에서 실시간으로 gain과 정책을 바꿀 수 있게 해줍니다. "
                    "실험 속도를 크게 올리는 대신, 파라미터 변경 시 즉시 제어 감각이 달라집니다."
                ),
                review=(
                    "runtime tuning을 적극적으로 지원하는 점은 강점입니다. "
                    "동시에 리뷰 관점에서는 파라미터 변화가 control loop에 어떤 영향을 주는지 문서가 꼭 필요하다는 뜻이기도 합니다."
                ),
                details=(
                    "이 함수는 단순히 숫자를 바꾸는 관리 함수가 아닙니다. 실제로는 제어 정책을 런타임에 갈아끼우는 진입점입니다. "
                    "예를 들어 yaw hold 민감도, roll/pitch torque 제한, translation 보호 강도, orientation filter 성향이 모두 비행 중 즉시 달라질 수 있습니다.\n\n"
                    "그래서 이 함수가 있는 모듈은 문서가 특히 중요합니다. 파라미터 이름만 나열해서는 충분하지 않고, 어떤 값이 어느 함수의 어떤 분기를 바꾸는지까지 연결해 줘야 안전하게 튜닝할 수 있습니다."
                ),
            ),
        ),
    ),
    ModuleConfig(
        key="depth",
        title="Depth Controller Review",
        source_filename="depth_controller.py",
        markdown_filename="controller_review_depth.md",
        html_filename="depth_controller_review.html",
        role_summary=(
            "이 코드는 depth sensor와 IMU, manual heave 입력을 받아 최종 `Fz` 명령을 만드는 수심 유지 제어기입니다."
        ),
        design_summary=(
            "설계의 핵심은 `depth hold + pilot depth-rate`입니다. "
            "즉 stick을 위아래로 움직이면 즉시 추진기 출력만 주는 것이 아니라, 목표 수심을 움직이는 방식으로 조종감을 만듭니다."
        ),
        review_focus=(
            "리뷰 포인트는 `target depth를 언제 어떻게 캡처하는지`, "
            "`manual heave freshness를 어떻게 지우는지`, `depth sensor offset을 IMU로 어떻게 보정하는지`, "
            "`최종 heave shaping이 actuator-friendly한지`입니다."
        ),
        runtime_summary=(
            "런타임에서는 `depth_callback()`가 depth sample을 받을 때마다 현재 depth, depth rate, manual heave 상태를 갱신하고 "
            "필요하면 target depth를 다시 잡습니다. 이후 PID, pilot depth-rate, saturation, upward limit, slew-rate 제한을 거쳐 "
            "최종 `Fz` 명령이 만들어집니다. 즉 이 모듈은 단순 PID가 아니라 상태 전이와 출력 shaping이 함께 들어간 depth hold 시스템입니다."
        ),
        parameters=(
            ("`kp_depth`, `ki_depth`, `kd_depth`", "수심 오차를 heave 명령으로 바꾸는 핵심 PID 게인입니다."),
            ("`manual_heave_override_threshold`", "이 값보다 큰 manual heave는 조종자가 depth hold를 직접 흔드는 입력으로 해석됩니다."),
            ("`manual_wrench_timeout_sec`", "manual heave 입력이 이 시간 이상 갱신되지 않으면 stale로 보고 0으로 복구합니다."),
            ("`pilot_depth_rate_enabled`", "manual heave를 직접 추력 명령으로 볼지, 목표 수심의 변화율로 볼지 정합니다."),
            ("`max_pilot_depth_rate`, `pilot_depth_rate_sign`", "pilot depth-rate 모드에서 stick 입력이 target depth를 얼마나 빠르게 움직일지 정합니다."),
            ("`manual_heave_release_target_offset`", "manual heave를 놓았을 때 현재 depth에 더해 새 target으로 삼는 오프셋입니다."),
            ("`max_heave`, `max_upward_heave`", "최종 heave 출력의 절대 한계와 상승 방향 한계를 정합니다."),
            ("`max_heave_delta_per_cycle`", "한 제어 주기에서 heave가 얼마나 급하게 바뀔 수 있는지 제한합니다."),
            ("`depth_rate_alpha`", "depth 미분값에 low-pass filter를 얼마나 강하게 적용할지 정합니다."),
            ("`depth_sensor_offset_x/y/z`, `depth_sensor_offset_compensation_enabled`", "센서 장착 위치 보정과 IMU 기반 compensation 사용 여부를 정합니다."),
        ),
        items=(
            ReviewItem(
                heading="`quat_to_rotation_z_row()`",
                source_names=("quat_to_rotation_z_row",),
                meaning=(
                    "IMU quaternion에서 body z축이 world에서 어디를 보는지 계산하는 보조 함수입니다."
                ),
                impact=(
                    "depth sensor가 기체 중심에서 떨어져 있을 때, pitch/roll에 의해 측정 depth가 흔들리는 문제를 보정할 수 있게 해줍니다."
                ),
                review=(
                    "작은 함수지만 depth sensor offset compensation의 핵심 기반입니다. "
                    "이 함수가 없으면 IMU를 depth controller가 활용할 방법이 사라집니다."
                ),
                details=(
                    "depth sensor가 기체 중심이 아니라 다른 위치에 달려 있으면, 같은 수심에서도 pitch/roll에 따라 센서의 world z 좌표가 달라집니다. "
                    "이 함수는 바로 그 보정을 위해 body z축이 world에서 어떻게 놓여 있는지를 계산합니다.\n\n"
                    "즉 depth controller가 '수심'만 보는 것 같아도 실제로는 자세 정보를 함께 써서 센서 물리 위치를 보정하는 구조이고, 이 함수가 그 수학적 출발점입니다."
                ),
            ),
            ReviewItem(
                heading="`__init__()`",
                source_names=("__init__",),
                meaning=(
                    "depth PID, manual override, pilot depth-rate, sensor offset compensation, output shaping 파라미터를 한 번에 선언합니다. "
                    "즉 depth controller가 단순한 `error -> heave` 계산기인지, 실제 운용형 hold 시스템인지가 여기서 갈립니다."
                ),
                impact=(
                    "이 함수가 depth hold의 운용 정책 전체를 결정합니다. "
                    "즉 단순 PID가 아니라 실제 조종기, arming, offset compensation, target clamp까지 모두 여기서 준비됩니다."
                ),
                review=(
                    "구성이 잘 되어 있고 실제 운용형 controller에 가깝습니다. "
                    "특히 `manual_wrench_timeout_sec`가 있는 점은 attitude/position보다 안전 측면에서 낫습니다."
                ),
                details=(
                    "이 함수는 depth controller를 읽을 때 가장 먼저 봐야 하는 구간입니다. "
                    "왜냐하면 depth hold의 감각은 `kp/ki/kd`보다도 `pilot_depth_rate_enabled`, "
                    "`manual_heave_release_target_offset`, `max_upward_heave`, `depth_rate_alpha` 같은 정책 파라미터들에 크게 좌우되기 때문입니다.\n\n"
                    "또한 이 함수는 sensor offset compensation 관련 파라미터도 함께 준비합니다. "
                    "즉 이 모듈은 단순 PID가 아니라 센서 물리 배치, arm/disarm 동작, manual release 감각, actuator 친화성까지 같이 품고 있는 시스템입니다."
                ),
                parameter_notes=(
                    ("`depth_topic`, `imu_topic`, `cmd_depth_topic`, `manual_wrench_topic`", "depth hold에 들어오는 주요 센서/명령 입력입니다."),
                    ("`kp_depth`, `ki_depth`, `kd_depth`", "수심 오차를 heave 출력으로 바꾸는 핵심 PID 게인입니다."),
                    ("`pilot_depth_rate_enabled`, `max_pilot_depth_rate`, `pilot_depth_rate_sign`", "manual heave를 direct thrust가 아니라 target depth rate로 해석할지 결정합니다."),
                    ("`manual_heave_override_threshold`, `manual_wrench_timeout_sec`", "manual heave 활성 조건과 stale input 해제 정책을 정합니다."),
                    ("`manual_heave_release_target_offset`", "pilot이 stick을 놓은 뒤 hold target을 현재 depth 기준 어디에 둘지 정합니다."),
                    ("`max_heave`, `max_upward_heave`, `max_heave_delta_per_cycle`", "출력 saturation과 actuator-friendly shaping을 담당합니다."),
                    ("`depth_sensor_offset_*`, `depth_sensor_offset_compensation_enabled`", "센서 장착 위치 보상에 필요한 파라미터입니다."),
                ),
            ),
            ReviewItem(
                heading="Arming / Active 상태 함수",
                source_names=("armed_callback", "publish_depth_active", "depth_control_is_active", "clamp_target_depth"),
                meaning=(
                    "이 함수들은 controller가 언제 활성인지, target depth를 허용 범위 안에 둘지, arm/disarm에서 어떤 초기화를 할지 정의합니다."
                ),
                impact=(
                    "ROV가 arm 될 때 current depth를 target으로 잡고, disarm 시 integrator와 출력 상태를 초기화합니다. "
                    "즉 이 계층은 안전성과 target 일관성을 담당합니다."
                ),
                review=(
                    "상태 전이가 비교적 명확합니다. "
                    "실무에서는 `depth_control_is_active()` 같은 함수가 있어야 상위 시스템에서 상태를 해석하기 쉬워집니다."
                ),
                details=(
                    "이 계층은 depth controller가 '언제 계산할 수 있는가'보다 '언제 계산해야 하는가'를 정의합니다. "
                    "`armed_callback()`은 arm/disarm 전이에서 integrator와 last output 같은 내부 상태를 정리하고, "
                    "`publish_depth_active()`는 상위 merger나 UI가 depth hold 상태를 명시적으로 알 수 있게 합니다. "
                    "`clamp_target_depth()`는 target이 물리적으로 지나치게 벗어나지 않도록 제한하는 방어 계층입니다.\n\n"
                    "즉 PID 수식은 depth를 맞추는 역할이고, 이 계층은 그 PID가 잘못된 상태에서 과하게 일하지 않도록 경계를 세우는 역할입니다."
                ),
                parameter_notes=(
                    ("`control_enabled`", "depth hold 계산 자체를 허용할지 정합니다."),
                    ("`max_depth`, `min_depth`", "있다면 target clamp에 직접 연결되는 안전 한계입니다."),
                ),
            ),
            ReviewItem(
                heading="`capture_manual_release_target()`",
                source_names=("capture_manual_release_target",),
                meaning=(
                    "manual heave를 놓았을 때 현재 depth에 release offset을 더해 새 target depth를 잡는 함수입니다."
                ),
                impact=(
                    "이 함수가 조종자가 stick을 놓은 뒤 depth hold가 어디에서 다시 잠길지를 결정합니다."
                ),
                review=(
                    "운용 철학은 분명하지만 `current_depth + offset`은 직관과 다를 수 있습니다. "
                    "오프셋이 항상 들어가면 '놓은 자리 유지'보다 '조금 이동한 자리 유지'가 되어 사용자 혼란을 줄 수 있습니다."
                ),
                details=(
                    "이 함수는 manual heave 조작이 끝났을 때 depth hold가 다시 어느 수심을 목표로 삼을지 결정합니다. "
                    "현재 구현은 단순히 마지막 stick 명령을 끄는 것이 아니라, 현재 depth를 기준으로 새 target을 다시 잡는 철학을 갖고 있습니다.\n\n"
                    "따라서 이 함수의 존재는 depth controller가 '추력 hold'가 아니라 '목표 수심 hold'라는 점을 분명히 보여줍니다. "
                    "사용자가 stick을 놓은 순간 즉시 안정적으로 잠기는 감각을 좌우하는 핵심 함수입니다."
                ),
                parameter_notes=(
                    ("`manual_heave_release_target_offset`", "release 시 현재 depth에서 얼마나 이동한 위치를 새 target으로 삼을지 정합니다."),
                ),
            ),
            ReviewItem(
                heading="`compensate_depth_sensor_offset()`",
                source_names=("imu_callback", "compensate_depth_sensor_offset"),
                meaning=(
                    "IMU로 센서의 world z 위치 변화를 계산해, pitch/roll 때문에 생기는 depth sensor 오차를 보정합니다."
                ),
                impact=(
                    "기체가 기울어져도 가짜 depth 변화에 과민 반응하지 않게 해 줍니다. "
                    "즉 depth hold가 실제 수심 변화를 더 정확히 보게 됩니다."
                ),
                review=(
                    "이 함수는 실전적인 품질을 크게 올리는 부분입니다. "
                    "센서 장착 위치가 중심에서 벗어난 수중체에서는 특히 의미가 큽니다."
                ),
                details=(
                    "이 함수는 센서가 측정한 raw depth를 그대로 믿지 않고, 현재 자세와 센서 오프셋을 이용해 기체 기준점 depth로 다시 환산합니다. "
                    "즉 pitch를 크게 주는 순간 센서가 위아래로 움직여 생기는 가짜 depth 변화에 controller가 속지 않도록 합니다.\n\n"
                    "실제 운용에서는 이 차이가 큽니다. 보정이 없으면 전진/상승 동작과 자세 변화가 섞일 때 depth hold가 필요 이상으로 heave를 흔들 수 있는데, 이 함수가 그 coupling을 줄여 줍니다."
                ),
                parameter_notes=(
                    ("`depth_sensor_offset_x/y/z`", "센서가 기체 기준점에서 얼마나 떨어져 있는지 나타냅니다."),
                    ("`depth_sensor_offset_compensation_enabled`", "자세 기반 위치 보정을 사용할지 정합니다."),
                ),
            ),
            ReviewItem(
                heading="Manual 입력 freshness 계열",
                source_names=("manual_wrench_callback", "manual_wrench_is_fresh", "clear_stale_manual_heave"),
                meaning=(
                    "manual heave 입력이 최근 입력인지 판정하고, stale이면 안전하게 0으로 되돌리며 필요하면 새 target depth를 다시 잡습니다."
                ),
                impact=(
                    "조종기 신호가 끊겼을 때 controller가 마지막 nonzero heave를 계속 믿는 문제를 막습니다."
                ),
                review=(
                    "이 계층은 현재 코드베이스에서 가장 좋은 패턴 중 하나입니다. "
                    "attitude/position도 이 freshness 전략을 가져오면 전체 일관성이 더 좋아집니다."
                ),
                details=(
                    "이 묶음은 manual input을 단순한 값이 아니라 시간축이 있는 상태로 다룬다는 점에서 중요합니다. "
                    "`manual_wrench_callback()`이 마지막 입력과 시각을 저장하고, `manual_wrench_is_fresh()`가 그것이 아직 유효한지 판단하며, "
                    "`clear_stale_manual_heave()`가 오래된 입력을 안전하게 0으로 복구합니다.\n\n"
                    "즉 이 계층은 통신 끊김이나 joystick 정지 상황에서 마지막 nonzero 입력이 controller 안에 유령처럼 남아 있는 문제를 막습니다. "
                    "실전 제어 소프트웨어에서는 이런 freshness 처리가 PID 게인만큼 중요합니다."
                ),
                parameter_notes=(
                    ("`manual_heave_override_threshold`", "manual heave가 실제 override로 간주되는 최소 크기입니다."),
                    ("`manual_wrench_timeout_sec`", "이 시간이 지나면 마지막 manual 입력을 stale로 보고 지웁니다."),
                ),
            ),
            ReviewItem(
                heading="`depth_callback()`",
                source_names=("depth_callback",),
                meaning=(
                    "depth sample이 들어올 때마다 target capture, depth rate 추정, PID 계산, pilot depth-rate, saturation, slew-rate, status publish를 수행합니다."
                ),
                impact=(
                    "이 함수가 곧 depth controller의 본체입니다. "
                    "PID와 운용 상태 머신이 모두 여기에서 만납니다."
                ),
                review=(
                    "구조가 좋고 actuator-friendly합니다. "
                    "특히 `depth_rate_alpha`, `max_heave_delta_per_cycle`, `max_upward_heave`가 실제 시스템을 거칠지 않게 만들어 줍니다."
                ),
                details=(
                    "이 함수 안에는 사실상 depth controller의 모든 핵심이 들어 있습니다. "
                    "먼저 현재 depth를 보정하고, manual heave stale 여부를 해제하고, 필요하면 초기 target을 캡처합니다. "
                    "그 다음 depth rate를 샘플 차분으로 계산한 뒤 low-pass filtering을 적용하고, 그 결과를 PID의 derivative 입력으로 사용합니다.\n\n"
                    "그 이후에는 상황에 따라 분기합니다. disarm이면 무조건 0, control disabled면 0, "
                    "manual heave + pilot depth-rate 모드면 target depth를 움직이고, 그렇지 않으면 정적인 target depth를 향해 PID를 수행합니다. "
                    "마지막으로 saturation, upward limit, slew-rate를 차례로 적용해 heave 명령을 publish합니다. "
                    "즉 이 함수는 제어 law와 output shaping이 하나의 파이프라인으로 묶인 구조입니다."
                ),
            ),
            ReviewItem(
                heading="`on_parameter_update()`",
                source_names=("on_parameter_update",),
                meaning=(
                    "depth controller의 gain, limit, manual policy, compensation 설정을 런타임에 바꿉니다."
                ),
                impact=(
                    "실험 중 depth 감각과 안전 정책을 바로 수정할 수 있게 해줍니다."
                ),
                review=(
                    "runtime tuning 친화적이지만, 파라미터가 많을수록 문서가 중요합니다. "
                    "이번 리뷰 사이트에서 이 함수가 중요한 이유도 바로 그 때문입니다."
                ),
                details=(
                    "depth hold는 겉보기보다 정책 파라미터가 많습니다. direct heave처럼 느껴질지, target depth rate처럼 느껴질지, release 후 즉시 잠길지, 상승 방향을 얼마나 제한할지 모두 런타임에 바뀔 수 있습니다.\n\n"
                    "따라서 이 함수는 단순 관리 루틴이 아니라 현장 튜닝 인터페이스입니다. 리뷰 문서가 파라미터-동작 연결을 자세히 설명해야 하는 이유가 여기 있습니다."
                ),
            ),
        ),
    ),
    ModuleConfig(
        key="position",
        title="Position Controller Review",
        source_filename="position_controller.py",
        markdown_filename="controller_review_position.md",
        html_filename="position_controller_review.html",
        role_summary=(
            "이 코드는 DVL와 IMU, manual input을 이용해 planar hold용 `Fx`, `Fy`, 그리고 경우에 따라 `Fz` 보정까지 만드는 위치 제어기입니다."
        ),
        design_summary=(
            "핵심 구조는 `world frame에서 위치 오차 계산 → body frame 힘으로 회전`입니다. "
            "즉 기체가 yaw되어 있어도 지구 기준 위치를 유지하려는 설계입니다."
        ),
        review_focus=(
            "리뷰 포인트는 `DVL position과 velocity fallback이 어떻게 연결되는지`, "
            "`manual XY override가 hold target과 어떻게 상호작용하는지`, "
            "`yaw motion-aware damping이 어떤 안정화 효과를 만드는지`, "
            "`body-frame 변환 후 force.z를 왜 출력하는지`입니다."
        ),
        runtime_summary=(
            "이 모듈은 `dvl_position_callback()` 또는 `dvl_callback()`에서 위치/속도 참조를 최신 상태로 유지하고, "
            "`publish_control_output()`에서 target과 현재 위치의 오차를 world frame에서 계산합니다. "
            "그 다음 yaw 상태를 반영한 damping을 더하고 결과 힘을 body frame으로 회전해 최종 force command로 publish합니다. "
            "즉 frame 해석과 DVL validity 관리가 control law만큼 중요한 모듈입니다."
        ),
        parameters=(
            ("`kp_x`, `kd_x`, `kp_y`, `kd_y`", "XY 위치 오차를 force로 바꾸는 기본 PD 게인입니다."),
            ("`max_force_x`, `max_force_y`, `max_force_z`", "각 축별 force saturation 한계를 정합니다."),
            ("`manual_xy_override_threshold`", "이 값보다 큰 manual XY 입력이 들어오면 position hold보다 pilot input을 우선합니다."),
            ("`capture_target_on_manual_release`", "pilot이 XY stick을 놓았을 때 현재 위치를 새 hold target으로 다시 잡을지 결정합니다."),
            ("`valid_timeout_sec`", "DVL position/velocity reference를 얼마 동안 유효하다고 볼지 정합니다."),
            ("`yaw_rate_damping_gain`", "yaw 회전이 클수록 XY damping을 얼마나 더 강하게 만들지 정합니다."),
            ("`manual_yaw_damping_boost`", "manual yaw 조작 중 XY hold를 더 안정적으로 만들기 위해 추가되는 damping입니다."),
            ("`use_dvl_position`", "DVL absolute position을 직접 쓸지, velocity integration 중심으로 갈지 결정합니다."),
            ("`integrate_dvl_velocity_when_position_unavailable`", "absolute position이 없을 때 velocity를 적분해 hold reference를 유지할지 정합니다."),
            ("`dvl_mount_roll_deg`, `dvl_mount_pitch_deg`, `dvl_mount_yaw_deg`", "DVL 장착 각도 보정 파라미터입니다."),
        ),
        items=(
            ReviewItem(
                heading="Quaternion 유틸리티",
                source_names=("clamp", "quat_normalize", "quat_multiply", "quat_conjugate", "quat_from_rpy", "quat_rotate_vector"),
                meaning=(
                    "이 함수들은 DVL 속도와 위치제어 출력을 body/world frame 사이에서 변환하기 위한 기본 수학 도구입니다."
                ),
                impact=(
                    "이 계층이 있어야 yaw가 바뀌어도 같은 world 위치를 유지할 수 있습니다. "
                    "즉 이 controller의 존재 이유 자체가 여기서 시작됩니다."
                ),
                review=(
                    "수학 유틸리티는 단순하지만 매우 중요합니다. "
                    "좌표계가 꼬이면 controller tuning 문제가 아니라 frame mismatch 문제가 됩니다."
                ),
                details=(
                    "position hold에서 가장 중요한 것은 오차 자체보다 오차가 어느 좌표계에서 정의되는가입니다. "
                    "이 유틸리티 묶음은 DVL 속도, hold target, 최종 force를 world/body frame 사이에서 일관되게 회전시키는 기반을 제공합니다.\n\n"
                    "즉 이 코드가 단순 planar PD가 아니라 yaw가 변해도 같은 world 위치를 붙잡을 수 있는 이유가 바로 이 계층에 있습니다. "
                    "문제가 생기면 게인을 의심하기 전에 frame rotation이 올바른지 먼저 확인해야 합니다."
                ),
            ),
            ReviewItem(
                heading="`__init__()`",
                source_names=("__init__",),
                meaning=(
                    "DVL 입력 방식, gain, yaw damping, manual override, frame 정책, velocity integration fallback까지 초기화합니다. "
                    "즉 이 함수는 position hold가 어떤 센서 구성과 어떤 좌표계 해석을 전제로 돌아갈지를 정의합니다."
                ),
                impact=(
                    "position hold가 absolute position 기반인지 velocity integration 기반인지, "
                    "manual release 시 target을 다시 잡을지, 어떤 frame으로 publish할지가 모두 여기서 결정됩니다."
                ),
                review=(
                    "실전 운용형 controller답게 파라미터 구성이 좋습니다. "
                    "다만 manual freshness timeout이 없어서 수동 입력이 stale일 때 hold 복귀가 늦어질 수 있습니다."
                ),
                details=(
                    "이 함수는 position controller를 단순 PD controller 이상으로 만들어 주는 핵심 구간입니다. "
                    "먼저 `dvl_topic`, `dvl_position_topic`, `imu_topic` 등으로 어떤 센서 조합을 사용할지 선언합니다. "
                    "그 다음 `kp_x/kd_x`, `kp_y/kd_y`로 기본 복원력과 감쇠를 정하고, "
                    "`yaw_rate_damping_gain`, `manual_yaw_damping_boost`로 yaw 운동 중 position hold를 얼마나 보수적으로 할지 결정합니다.\n\n"
                    "또 `use_dvl_position`, `integrate_dvl_velocity_when_position_unavailable`는 absolute position 기반인지 dead-reckoning fallback 기반인지 선택하게 해 줍니다. "
                    "즉 이 함수는 '위치제어기'를 초기화하는 것이 아니라, 실제 field condition에서 어떤 센서 가정과 어떤 hold 감각으로 운영할지 선언하는 함수라고 보는 편이 정확합니다."
                ),
                parameter_notes=(
                    ("`dvl_topic`, `dvl_position_topic`, `imu_topic`, `manual_wrench_topic`", "position hold가 의존하는 센서와 pilot 입력 채널입니다."),
                    ("`kp_x`, `kd_x`, `kp_y`, `kd_y`", "XY 위치 오차를 planar force로 바꾸는 핵심 PD 게인입니다."),
                    ("`yaw_rate_damping_gain`, `manual_yaw_damping_boost`, `manual_yaw_override_threshold`", "yaw 운동/조작이 있을 때 XY hold를 얼마나 보수적으로 만들지 결정합니다."),
                    ("`manual_xy_override_threshold`, `capture_target_on_manual_release`", "manual XY 조작 중 auto hold를 끊고, release 후 hold target을 다시 잡는 정책입니다."),
                    ("`valid_timeout_sec`", "DVL 기반 reference가 stale인지 판단하는 유효 시간입니다."),
                    ("`use_dvl_position`, `integrate_dvl_velocity_when_position_unavailable`", "absolute position과 velocity fallback 중 어떤 전략을 쓸지 정합니다."),
                    ("`dvl_mount_roll_deg`, `dvl_mount_pitch_deg`, `dvl_mount_yaw_deg`", "DVL 장착 방향이 body frame과 어긋날 때 이를 보정합니다."),
                ),
            ),
            ReviewItem(
                heading="상태/보조 함수",
                source_names=("_publish_zero_force", "_has_valid_position_reference", "_set_control_enabled", "_capture_current_position_as_target", "_apply_deadband", "_dvl_position_is_finite"),
                meaning=(
                    "이 함수들은 제어 출력을 언제 0으로 만들지, target을 언제 갱신할지, DVL 데이터가 유효한지 판정하는 기반 계층입니다."
                ),
                impact=(
                    "센서가 유효하지 않으면 output을 0으로 내리고, enable edge에서 hold target을 다시 캡처할 수 있습니다."
                ),
                review=(
                    "이 보조 계층 덕분에 publish 조건이 깔끔합니다. "
                    "실제 control law보다 앞단의 상태 판정이 잘 분리된 점이 장점입니다."
                ),
                details=(
                    "이 함수들은 각각 작아 보여도 제어기 안정성의 기본 골격을 담당합니다. "
                    "`_has_valid_position_reference()`는 DVL 기반 상태가 충분히 최신인지 검사하고, `_set_control_enabled()`는 enable edge에서 target 재캡처와 zero-force 출력을 관리합니다. "
                    "`_capture_current_position_as_target()`은 pilot release나 enable 시점에서 hold 기준점을 다시 정합니다.\n\n"
                    "즉 이 계층은 'position error를 계산하기 전에 무엇이 유효한 상태인가'를 먼저 정의합니다. "
                    "센서가 stale이거나 armed 상태가 아닌데도 force를 계속 내지 않도록 막는 방어 로직이 여기 모여 있습니다."
                ),
                parameter_notes=(
                    ("`valid_timeout_sec`", "position reference가 stale인지 판단하는 기준 시간입니다."),
                    ("`control_enabled`", "hold force 계산 자체를 허용할지 정합니다."),
                    ("`force_deadband`", "작은 force를 제거하는 deadband 크기입니다."),
                    ("`capture_initial_position_target`", "초기 유효 position을 target으로 자동 캡처할지 정합니다."),
                ),
            ),
            ReviewItem(
                heading="`manual_wrench_callback()`",
                source_names=("manual_wrench_callback",),
                meaning=(
                    "manual XY 입력이 active인지 판정하고, release edge에서 현재 위치를 새 hold target으로 캡처합니다."
                ),
                impact=(
                    "조종자가 이동을 끝내고 stick을 놓았을 때 그 자리에서 poshold가 다시 잠기게 만드는 함수입니다."
                ),
                review=(
                    "운용감은 좋지만 freshness timeout이 없어서 마지막 manual XY가 남으면 poshold 복귀가 안 될 수 있습니다. "
                    "현재 코드의 대표적인 개선 후보입니다."
                ),
                details=(
                    "이 함수는 manual XY를 단순히 저장하는 것보다, 현재 자동 hold를 잠시 끊어야 하는지 판단하는 역할이 더 큽니다. "
                    "threshold를 넘는 입력이 들어오면 `manual_xy_active`를 켜고, release edge에서 현재 위치를 새 target으로 캡처해 자연스럽게 poshold로 복귀하게 만듭니다.\n\n"
                    "즉 사용자는 '직접 움직이다가 손을 놓으면 그 자리에서 다시 hold'되는 감각을 얻게 됩니다. "
                    "다만 freshness 관리가 없기 때문에, 입력 주기가 끊겼을 때 manual 상태가 오래 남을 수 있다는 점은 문서에서 분명히 짚어야 합니다."
                ),
                parameter_notes=(
                    ("`manual_xy_override_threshold`", "manual XY가 auto hold를 끊는 기준입니다."),
                    ("`capture_target_on_manual_release`", "pilot release 시 현재 위치를 새 hold target으로 잡을지 결정합니다."),
                ),
            ),
            ReviewItem(
                heading="DVL 입력 처리",
                source_names=("dvl_position_callback", "dvl_callback"),
                meaning=(
                    "DVL position을 직접 쓰거나, 없으면 velocity를 body→world로 회전해 적분 위치를 만들고 속도 피드백을 갱신합니다."
                ),
                impact=(
                    "센서 구성에 따라 absolute hold와 dead-reckoning hold를 모두 지원하게 해줍니다."
                ),
                review=(
                    "유연성이 높고 실제 운용에 유리합니다. "
                    "특히 `use_dvl_position=False` fallback 설계는 테스트 단계에서 큰 장점이 됩니다."
                ),
                details=(
                    "이 계층은 position controller가 어떤 형태의 DVL 정보를 받아도 내부 world-frame position/velocity 상태로 변환해 주는 센서 적응 계층입니다. "
                    "`dvl_position_callback()`은 absolute position이 있을 때 이를 직접 hold 기준으로 쓰고, `dvl_callback()`은 body-frame velocity를 IMU yaw를 이용해 world frame으로 돌린 뒤 적분 fallback까지 수행합니다.\n\n"
                    "즉 센서가 완벽하지 않아도 position hold를 최대한 유지하려는 실무형 구조입니다. "
                    "absolute 위치와 dead-reckoning 위치가 어떻게 전환되는지 이해하면, poshold drift의 원인이 센서인지 controller인지 구분하기 쉬워집니다."
                ),
                parameter_notes=(
                    ("`use_dvl_position`", "absolute DVL position을 직접 사용할지 정합니다."),
                    ("`integrate_dvl_velocity_when_position_unavailable`", "position이 없을 때 velocity 적분 fallback을 사용할지 정합니다."),
                    ("`dvl_mount_roll_deg`, `dvl_mount_pitch_deg`, `dvl_mount_yaw_deg`", "DVL 축이 body frame과 어긋난 경우 보정합니다."),
                    ("`valid_timeout_sec`", "최근 DVL 기준을 얼마나 오래 유효로 볼지 정합니다."),
                ),
            ),
            ReviewItem(
                heading="`publish_position_estimate()`",
                source_names=("publish_position_estimate",),
                meaning=(
                    "현재 controller가 보고 있는 position estimate를 외부에 publish합니다."
                ),
                impact=(
                    "상위 시각화, 디버깅, 로깅에서 controller 내부 상태를 그대로 볼 수 있게 해줍니다."
                ),
                review=(
                    "작은 함수지만 디버깅 가치가 큽니다. "
                    "이런 상태 출력이 없으면 position hold 문제를 센서 문제와 구분하기 어렵습니다."
                ),
                details=(
                    "이 함수는 controller 내부가 현재 어떤 위치를 믿고 있는지 외부로 드러내는 관찰 창입니다. "
                    "사용자는 이 publish를 통해 hold target과 현재 estimate 사이의 차이를 시각화하거나 로그로 비교할 수 있습니다.\n\n"
                    "실제 제어 문제는 '힘이 이상하다'보다 '컨트롤러가 잘못된 위치를 믿고 있다'에서 시작되는 경우가 많습니다. "
                    "그래서 이런 상태 publish 함수는 작아도 리뷰에서 반드시 의미를 설명해 줘야 합니다."
                ),
            ),
            ReviewItem(
                heading="`publish_control_output()`",
                source_names=("publish_control_output",),
                meaning=(
                    "validity check, target error 계산, yaw-rate aware damping, world→body 회전, manual override 적용, force clamp를 수행하는 본체입니다."
                ),
                impact=(
                    "이 함수가 실제 position hold의 감각을 결정합니다. "
                    "특히 yaw 회전 중 damping boost와 body-frame force 변환이 핵심입니다."
                ),
                review=(
                    "구조는 명확하고 PD hold로서 좋습니다. "
                    "다만 `force_body_z`를 최종 출력에 포함시키는 부분은 설계 의도를 명시하지 않으면 depth와의 coupling을 읽는 사람이 헷갈릴 수 있습니다."
                ),
                details=(
                    "이 함수는 실제 position hold의 중심입니다. 하지만 단순히 오차를 force로 바꾸기 전에 먼저 "
                    "`target_initialized`, `valid reference`, `armed`, `control_enabled`를 확인합니다. "
                    "즉 이 코드는 좋은 출력을 만드는 것보다, 잘못된 상태에서 출력을 내지 않는 것을 먼저 중요하게 봅니다.\n\n"
                    "그 다음 error를 world frame에서 계산하고 yaw 상태에 따라 damping을 강화합니다. "
                    "이후 world force를 body frame으로 회전해 기체가 실제로 낼 수 있는 축으로 바꾸고, manual XY가 active면 auto force를 0으로 눌러 pilot 우선 정책을 구현합니다. "
                    "마지막에는 saturation과 deadband를 적용해 actuator-friendly한 force로 publish합니다."
                ),
                parameter_notes=(
                    ("`kp_x`, `kd_x`, `kp_y`, `kd_y`", "오차와 속도를 force로 바꾸는 식에 직접 들어갑니다."),
                    ("`yaw_rate_damping_gain`, `manual_yaw_damping_boost`", "yaw motion이 클수록 derivative 성분을 얼마나 더 키울지 정합니다."),
                    ("`max_force_x`, `max_force_y`, `max_force_z`", "최종 force clamp 상한입니다."),
                    ("`force_deadband`", "아주 작은 force를 0으로 눌러 actuator chatter를 줄입니다."),
                ),
            ),
            ReviewItem(
                heading="`on_parameter_update()`",
                source_names=("on_parameter_update",),
                meaning=(
                    "gain, threshold, frame, DVL fallback 정책을 실시간으로 수정합니다."
                ),
                impact=(
                    "테스트 중 poshold의 stiffness와 damping을 바로 바꿀 수 있습니다."
                ),
                review=(
                    "runtime tuning은 강점이지만, 어떤 파라미터가 어떤 함수를 통해 결과에 연결되는지 문서가 꼭 필요합니다. "
                    "이번 리뷰 문서가 그 연결을 설명하는 역할을 합니다."
                ),
                details=(
                    "position hold는 gain뿐 아니라 sensor strategy, manual override policy, yaw damping 정책을 함께 튜닝해야 합니다. "
                    "즉 이 함수는 단순 PD gain 조정기가 아니라, 제어 law와 센서 해석 정책을 함께 바꾸는 관리 포인트입니다.\n\n"
                    "그래서 문서에서도 각 파라미터를 단독 설명하는 수준을 넘어, 어떤 함수에서 사용되고 어떤 동작 분기를 바꾸는지까지 함께 설명하는 것이 중요합니다."
                ),
            ),
        ),
    ),
    ModuleConfig(
        key="merger",
        title="Wrench Merger Review",
        source_filename="wrench_merger.py",
        markdown_filename="controller_review_wrench_merger.md",
        html_filename="wrench_merger_review.html",
        role_summary=(
            "이 코드는 manual input, depth, position, attitude 출력을 축별 규칙으로 합쳐 최종 `Wrench`를 만드는 supervisory mixer입니다."
        ),
        design_summary=(
            "핵심 설계는 `manual override + auto hold arbitration`입니다. "
            "각 축별로 manual과 auto의 우선순위를 다르게 해석해 최종 명령을 만듭니다."
        ),
        review_focus=(
            "리뷰 포인트는 `manual freshness가 어떻게 적용되는지`, "
            "`depth active일 때 heave를 어떻게 해석하는지`, "
            "`roll/pitch와 yaw의 arbitration 철학이 어떻게 다른지`입니다."
        ),
        runtime_summary=(
            "이 모듈은 각 callback에서 manual/depth/position/attitude 입력의 최신값만 저장하고, "
            "실제 merge는 timer 기반 `publish_merged_wrench()`에서 수행합니다. "
            "그래서 입력 주기가 서로 달라도 최종 wrench 주기를 일정하게 유지할 수 있고, armed/disarmed 상태를 마지막 게이트로 적용할 수 있습니다."
        ),
        parameters=(
            ("`publish_rate`", "최종 wrench를 얼마 주기로 publish할지 정합니다."),
            ("`manual_wrench_timeout_sec`", "manual input이 stale이면 zero wrench로 간주하는 기준 시간입니다."),
            ("`manual_xy_override_threshold`", "XY 축에서 manual이 auto position보다 우선하는 경계값입니다."),
            ("`manual_heave_override_threshold`", "heave 축에서 manual이 auto depth보다 우선하는 경계값입니다."),
            ("`manual_yaw_override_threshold`", "yaw 축에서 manual yaw가 attitude yaw hold보다 우선하는 경계값입니다."),
        ),
        items=(
            ReviewItem(
                heading="`__init__()`",
                source_names=("__init__",),
                meaning=(
                    "입력 토픽, armed gating, manual override threshold, publish timer와 마지막 입력 상태들을 준비합니다. "
                    "즉 이 함수는 merger가 어떤 축에서 누구 말을 우선 들을지 정하는 arbitration 환경을 초기화합니다."
                ),
                impact=(
                    "이 함수가 곧 이 노드의 arbitration 정책 초기 상태를 만듭니다. "
                    "특히 timer 기반 publish는 입력 주기가 달라도 최종 wrench 주기를 일정하게 유지합니다."
                ),
                review=(
                    "구조가 단순하고 목적이 분명합니다. "
                    "이런 중간 supervisory layer가 있으면 상위 제어기와 allocator를 느슨하게 연결할 수 있습니다."
                ),
                details=(
                    "이 함수가 중요한 이유는 이 노드가 전체 시스템의 마지막 supervisory decision layer이기 때문입니다. "
                    "상위 depth / position / attitude controller들이 각각 좋은 출력을 만들어도, 최종적으로 어떤 출력을 실제 thruster 쪽에 보낼지는 이 노드가 결정합니다.\n\n"
                    "여기서 `manual_*_override_threshold`와 `manual_wrench_timeout_sec`가 함께 선언된다는 점이 중요합니다. "
                    "즉 단순히 manual이 auto보다 우선하는 것이 아니라, manual이 '얼마나 커야 우선인지', '얼마 동안 유효한지'까지 여기서 정의됩니다. "
                    "또 `publish_rate`가 timer 기반으로 사용되므로, 이 함수는 최종 output synchronization 정책도 함께 선언합니다."
                ),
                parameter_notes=(
                    ("`manual_wrench_topic`, `depth_heave_topic`, `depth_active_topic`, `position_force_topic`, `attitude_torque_topic`", "merge 대상이 되는 모든 입력 채널입니다."),
                    ("`output_wrench_topic`", "최종 arbitration 결과가 publish되는 출력 채널입니다."),
                    ("`publish_rate`", "각 입력 callback 즉시 출력이 아니라 timer 기반으로 몇 Hz로 재발행할지 정합니다."),
                    ("`manual_xy_override_threshold`, `manual_heave_override_threshold`, `manual_yaw_override_threshold`", "축별로 manual이 auto를 이기기 시작하는 경계값입니다."),
                    ("`manual_wrench_timeout_sec`", "manual 입력을 stale로 보고 무시하기 시작하는 시간입니다."),
                ),
            ),
            ReviewItem(
                heading="입력 저장 함수",
                source_names=("manual_wrench_callback", "depth_heave_callback", "depth_active_callback", "position_force_callback", "attitude_torque_callback", "armed_callback"),
                meaning=(
                    "각 제어기 출력을 최신값으로 저장하고 armed 상태를 갱신하는 함수들입니다."
                ),
                impact=(
                    "이 함수들이 직접 최종 출력은 만들지 않지만, timer 루프가 읽을 최신 상태를 보관합니다."
                ),
                review=(
                    "timer 기반 merge 구조와 잘 맞습니다. "
                    "다만 manual 이외 auto 입력에 freshness timestamp가 없어 마지막 자동 출력이 오래 남을 수 있습니다."
                ),
                details=(
                    "이 함수들의 공통점은 직접 출력 계산을 하지 않고, timer 루프가 읽을 마지막 상태만 저장한다는 점입니다. "
                    "이 구조 덕분에 depth/position/attitude controller들의 publish 주기가 서로 달라도 merger는 일정한 주기로 최종 wrench를 재구성할 수 있습니다.\n\n"
                    "즉 입력 callback과 최종 arbitration을 분리해 temporal decoupling을 만든 설계입니다. "
                    "다만 manual 외 자동 입력 freshness가 없기 때문에, 상위 노드가 멈췄을 때 마지막 값이 남을 수 있다는 점은 꼭 문서에 드러나야 합니다."
                ),
            ),
            ReviewItem(
                heading="`manual_wrench_is_fresh()`",
                source_names=("publish_zero_wrench", "manual_wrench_is_fresh"),
                meaning=(
                    "manual input이 최근에 들어왔는지 판정하고 stale이면 사실상 zero wrench로 취급하게 만드는 안전 함수입니다."
                ),
                impact=(
                    "조종기 신호 유실 시 마지막 manual 명령이 latch되는 문제를 막아줍니다."
                ),
                review=(
                    "현재 코드베이스에서 freshness 처리가 가장 깔끔하게 들어간 부분입니다. "
                    "같은 아이디어가 auto inputs에도 확장되면 훨씬 견고해집니다."
                ),
                details=(
                    "이 함수는 merger가 manual input을 '값'이 아니라 '최근성 있는 사건'으로 해석하게 만들어 줍니다. "
                    "`publish_zero_wrench()`와 함께 사용되면서 조종기 데이터가 사라졌을 때 마지막 manual 명령을 계속 재사용하지 않도록 방지합니다.\n\n"
                    "supervisory layer에서는 이런 freshness가 특히 중요합니다. 이 노드는 최종 명령을 내보내기 때문에, 오래된 manual 값 하나가 전체 시스템 출력을 계속 왜곡할 수 있기 때문입니다."
                ),
                parameter_notes=(
                    ("`manual_wrench_timeout_sec`", "manual 입력을 더 이상 믿지 않는 시간 기준입니다."),
                ),
            ),
            ReviewItem(
                heading="`publish_merged_wrench()`",
                source_names=("publish_merged_wrench",),
                meaning=(
                    "armed gating, manual freshness 확인, 축별 override 규칙, depth active 정책, yaw arbitration을 적용해 최종 wrench를 publish합니다."
                ),
                impact=(
                    "이 함수가 전체 제어 시스템의 최종 의사결정자입니다. "
                    "같은 upper controller 출력이라도 여기서 어떤 축은 manual이 이기고 어떤 축은 auto가 이깁니다."
                ),
                review=(
                    "정책은 명료하고 읽기 쉽습니다. "
                    "다만 `last_position_force`, `last_depth_heave`, `last_attitude_torque`에도 freshness를 넣지 않으면 상위 노드 장애 시 마지막 값이 계속 살아남습니다."
                ),
                details=(
                    "이 함수는 사실상 전체 제어기의 최종 의사결정표입니다. "
                    "먼저 armed state를 확인해 출력 자체를 허용할지 결정하고, 그 다음 manual input freshness를 검사해 stale manual을 자동으로 0으로 만듭니다. "
                    "이후 각 축별로 다른 규칙을 적용합니다: surge/sway는 manual XY threshold를 넘으면 manual, 아니면 position force를 씁니다. "
                    "heave는 depth_active 여부가 중요해서, depth hold가 켜져 있으면 manual heave보다 depth controller 출력을 우선 해석합니다. "
                    "roll/pitch는 attitude auto torque가 항상 이기고, yaw는 manual yaw가 threshold를 넘을 때만 auto yaw hold를 덮어씁니다.\n\n"
                    "즉 이 함수는 단순 merge가 아니라, '축마다 다른 운용 철학을 구현하는 arbitration state machine'입니다."
                ),
                parameter_notes=(
                    ("`manual_xy_override_threshold`", "surge/sway에서 position hold를 manual이 끊는 기준입니다."),
                    ("`manual_heave_override_threshold`", "depth hold가 비활성일 때 heave를 manual이 강제로 가져오는 기준입니다."),
                    ("`manual_yaw_override_threshold`", "yaw hold보다 manual yaw를 우선하는 기준입니다."),
                    ("`manual_wrench_timeout_sec`", "이 시간이 지나면 마지막 manual 입력을 더 이상 믿지 않습니다."),
                ),
            ),
            ReviewItem(
                heading="`on_parameter_update()`",
                source_names=("on_parameter_update",),
                meaning=(
                    "publish rate와 override threshold를 런타임에 갱신하고 timer도 재생성합니다."
                ),
                impact=(
                    "운용자가 merge 주기와 manual override 감도를 실시간 조정할 수 있습니다."
                ),
                review=(
                    "supervisory node답게 runtime tuning이 단순하고 이해하기 쉽습니다."
                ),
                details=(
                    "이 함수는 merge 주기와 manual override 감도를 운영 중에도 바꿀 수 있게 해 줍니다. "
                    "supervisory node에서는 작은 threshold 변화만으로도 manual/auto 우선순위가 달라지므로, 이런 갱신 함수는 생각보다 시스템 거동에 큰 영향을 미칩니다.\n\n"
                    "따라서 리뷰 문서에서도 단순히 '업데이트 가능'이라고 끝내지 않고, 어떤 threshold가 어떤 축의 arbitration을 바꾸는지 연결해서 설명하는 것이 중요합니다."
                ),
            ),
        ),
    ),
    ModuleConfig(
        key="allocator",
        title="Allocator Review",
        source_filename="allocator_node.py",
        markdown_filename="controller_review_allocator.md",
        html_filename="allocator_node_review.html",
        role_summary=(
            "이 코드는 최종 `Wrench(Fx, Fy, Fz, Tx, Ty, Tz)`를 8개 thruster 명령으로 바꾸는 control allocation 노드입니다."
        ),
        design_summary=(
            "설계의 핵심은 `수평 4개(Fx, Fy, Tz)`와 `수직 4개(Fz, Tx, Ty)`를 분리해서 pseudo-inverse와 priority logic으로 다루는 것입니다."
        ),
        review_focus=(
            "리뷰 포인트는 `행렬이 실제 기체 배치를 어떻게 표현하는지`, "
            "`수평/수직 allocator가 어떤 철학으로 분리되는지`, "
            "`compensation 항이 왜 필요한지`, "
            "`최종 shaping과 slew-rate가 actuator에 어떤 영향을 주는지`입니다."
        ),
        runtime_summary=(
            "런타임에서는 `callback()`가 들어온 wrench를 먼저 내부 축 부호와 gain으로 해석하고, "
            "수평과 수직 그룹으로 나눠 allocation을 수행합니다. "
            "그 뒤 compensation, priority allocation, normalization, deadband, slew-rate를 차례로 적용해 "
            "실제 thruster array로 publish합니다. 즉 이 모듈은 '마지막 수학 + 마지막 actuator shaping' 계층입니다."
        ),
        parameters=(
            ("`horizontal_output_gain`, `vertical_output_gain`, `yaw_output_gain`", "수평/수직 force와 yaw torque가 thruster 출력으로 얼마나 강하게 반영될지 정합니다."),
            ("`heave_gain`", "heave 요구를 vertical thruster 출력으로 키우는 기본 gain입니다."),
            ("`pitch_torque_gain`, `rear_vertical_bias`", "pitch recovery 성향과 후방 수직 thruster 바이어스를 조절합니다."),
            ("`torque_first_allocation`", "수직 그룹에서 자세 토크를 먼저 만족시키고 남는 헤드룸에 heave를 넣을지 결정합니다."),
            ("`level_horizontal_compensation_*`", "기체가 기울어진 상태에서 수평 이동이 heave 성분을 만들 때 이를 보정하는 파라미터 묶음입니다."),
            ("`attitude_priority_horizontal_slowdown_*`", "자세 토크 요구가 커질수록 horizontal output을 얼마나 줄일지 정합니다."),
            ("`surge_pitch_moment_*`", "surge thrust가 만드는 pitch moment를 보상할지와 그 강도를 정합니다."),
            ("`imu_pitch_hold_*`", "현재 pitch와 target pitch 차이를 allocator 차원에서 추가 보상할지 정합니다."),
            ("`slew_rate`", "thruster 출력이 한 번에 너무 급하게 바뀌지 않도록 제한합니다."),
            ("`max_output`, `output_scale`, `output_deadband`", "최종 thruster 명령 범위, 전체 출력 크기, deadband 제거 수준을 정합니다."),
        ),
        items=(
            ReviewItem(
                heading="기본 유틸리티",
                source_names=("normalize", "normalize_group_unit", "quat_to_rotation_z_row", "quat_to_rpy"),
                meaning=(
                    "행렬 생성과 IMU 보상 계산에 필요한 벡터/자세 유틸리티입니다."
                ),
                impact=(
                    "allocation 행렬과 IMU 기반 보상 항의 수학적 기반을 제공합니다."
                ),
                review=(
                    "작은 함수지만 allocator가 단순 mixer가 아니라 IMU-aware allocation이라는 점을 보여줍니다."
                ),
                details=(
                    "allocator에서 이 유틸리티들은 단순 계산 편의 함수가 아니라, 뒤쪽 보상 로직과 allocation geometry가 모두 기대고 있는 수학 기반입니다. "
                    "`normalize_group_unit()`은 한 그룹 출력이 saturation을 넘지 않도록 정규화하고, `quat_to_rotation_z_row()`는 현재 기체의 기울기 정보를 보상 항 계산에 사용할 수 있게 해 줍니다.\n\n"
                    "즉 allocator가 고정된 행렬 곱셈기를 넘어, 현재 자세를 아는 분배기처럼 동작하게 만드는 첫 계층입니다."
                ),
            ),
            ReviewItem(
                heading="`__init__()`",
                source_names=("__init__",),
                meaning=(
                    "thruster sign, gain, compensation, priority, output shaping, runtime tuning 관련 모든 파라미터를 선언합니다. "
                    "즉 allocator가 단순 행렬 연산 노드인지, 실제 기체 거동까지 반영하는 smart output layer인지가 여기서 결정됩니다."
                ),
                impact=(
                    "allocator가 단순 `TAM * u = τ` 계산기가 아니라 실제 운용형 shaping node라는 점이 여기서 드러납니다."
                ),
                review=(
                    "파라미터가 많지만 역할이 분명합니다. "
                    "다만 `fx = -msg.force.x`처럼 본체 쪽 부호 해석이 하드코딩되어 있어서 문서화가 꼭 필요합니다."
                ),
                details=(
                    "지금 보여주신 화면에서 가장 빈약하게 보였던 부분이 바로 이 함수입니다. 실제로는 이 함수가 allocator 전체 성격을 거의 다 설명해 줍니다. "
                    "첫 번째 덩어리는 `wrench_cmd_topic`, `thruster_cmd_topic`, `imu_topic` 같은 입출력 연결입니다. "
                    "두 번째 덩어리는 `heave_gain`, `horizontal_output_gain`, `yaw_output_gain`, `vertical_output_gain`처럼 force/torque 축별 gain입니다. "
                    "세 번째 덩어리는 `torque_first_allocation`, `rear_vertical_bias`, `pitch_torque_gain`처럼 allocation 우선순위와 기체 성향을 반영하는 파라미터입니다.\n\n"
                    "그리고 이 코드의 개성을 가장 잘 보여주는 것은 뒤쪽 compensation 파라미터들입니다. "
                    "`level_horizontal_compensation_*`, `attitude_priority_horizontal_slowdown_*`, `surge_pitch_moment_*`, `imu_pitch_hold_*`는 "
                    "단순한 `TAM pseudo-inverse`를 넘어서 실제 기체가 기울고, 전진하고, 자세를 유지할 때 어떤 출력을 내야 하는지를 allocator 수준에서 조정합니다. "
                    "즉 이 함수는 allocator의 철학서에 가깝습니다."
                ),
                parameter_notes=(
                    ("`wrench_cmd_topic`, `thruster_cmd_topic`, `imu_topic`", "allocator가 무엇을 입력으로 받고 무엇을 출력할지 정합니다."),
                    ("`heave_gain`, `horizontal_output_gain`, `vertical_output_gain`, `yaw_output_gain`", "각 축 요구를 실제 thruster 명령으로 얼마나 크게 반영할지 정하는 기본 gain입니다."),
                    ("`torque_first_allocation`", "수직 그룹에서 heave보다 자세 토크를 먼저 만족시킬지 결정합니다."),
                    ("`rear_vertical_bias`, `pitch_torque_gain`", "수직 thruster 분배 성향과 pitch 복원 특성을 조정합니다."),
                    ("`level_horizontal_compensation_*`", "기체가 기울어진 상태에서 수평 이동이 heave로 새는 문제를 allocator 단계에서 보정합니다."),
                    ("`attitude_priority_horizontal_slowdown_*`", "자세 토크 요구가 크면 horizontal 출력을 얼마나 줄일지 정합니다."),
                    ("`surge_pitch_moment_*`", "surge thrust가 만드는 pitch moment를 allocator 차원에서 보상합니다."),
                    ("`imu_pitch_hold_*`", "현재 pitch와 target pitch 차이를 보고 추가적인 hold 보상을 넣습니다."),
                    ("`slew_rate`, `max_output`, `output_scale`, `output_deadband`", "최종 actuator-friendly output shaping과 saturation 성격을 결정합니다."),
                ),
            ),
            ReviewItem(
                heading="IMU / Attitude 입력 함수",
                source_names=("imu_callback", "cmd_attitude_callback", "cmd_attitude_trim_callback", "output_scale_callback", "joy_speed_scale_callback"),
                meaning=(
                    "allocator가 현재 자세, 목표 trim, 출력 scale 상태를 받아 compensation과 output shaping에 반영하는 입력 계층입니다."
                ),
                impact=(
                    "이 함수들이 있어 allocator는 단순 분배가 아니라 현재 자세와 운용 상태를 고려하는 smart mixer가 됩니다."
                ),
                review=(
                    "상위 controller와 allocator가 느슨하지만 의미 있게 연결되어 있습니다."
                ),
                details=(
                    "이 계층은 allocator가 독립적인 하위 노드이면서도 상위 자세 제어 의도와 조종 상태를 읽을 수 있게 해 줍니다. "
                    "`imu_callback()`은 현재 pitch/roll을 보상 함수들에 제공하고, `cmd_attitude_callback()` 및 `cmd_attitude_trim_callback()`은 목표 자세 기준을 allocator 보상에 연결합니다. "
                    "`output_scale_callback()`과 `joy_speed_scale_callback()`은 전체 추진 크기를 런타임에 조절하는 운영 인터페이스입니다.\n\n"
                    "즉 allocator는 단순 수동 mixer가 아니라, 상위 제어 의도와 현재 기체 자세를 함께 반영하는 output shaping layer입니다."
                ),
                parameter_notes=(
                    ("`cmd_attitude_topic`, `cmd_attitude_trim_topic`", "목표 pitch/trim 기준을 allocator 보상에 전달합니다."),
                    ("`output_scale_topic`, `joy_speed_scale_topic`, `use_joy_speed_scale_for_output`", "최종 thruster 출력의 전체 크기를 외부에서 조정합니다."),
                ),
            ),
            ReviewItem(
                heading="Compensation 함수들",
                source_names=("level_horizontal_heave_compensation", "attitude_priority_horizontal_scale", "surge_pitch_moment_compensation", "imu_pitch_hold_compensation"),
                meaning=(
                    "수평 이동으로 인한 heave 누수, attitude demand가 클 때 horizontal 약화, surge에 따른 pitch moment, IMU 기반 pitch hold 보상 등 실제 기체 거동을 보정합니다."
                ),
                impact=(
                    "이 함수들이 켜지면 allocator는 단순 분배기에서 벗어나, 기체 자세와 운동 모드에 따라 더 실용적인 출력을 냅니다."
                ),
                review=(
                    "이 부분이 가장 실험적이면서도 고급스럽습니다. "
                    "동시에 gain 의존성이 큰 영역이므로 기능별 로그와 문서가 꼭 있어야 합니다."
                ),
                details=(
                    "이 묶음은 allocator를 단순한 TAM 분배기에서 실제 운용형 shaping node로 바꿔 주는 핵심입니다. "
                    "`level_horizontal_heave_compensation()`은 기체가 기울어진 상태에서 수평 추력이 수직 성분을 만드는 문제를 완화하고, "
                    "`attitude_priority_horizontal_scale()`은 자세 토크 요구가 클 때 수평 이동을 일부 희생하게 만듭니다. "
                    "`surge_pitch_moment_compensation()`과 `imu_pitch_hold_compensation()`은 전진 thrust와 현재 pitch 상태를 반영해 추가적인 수직 보상을 생성합니다.\n\n"
                    "즉 이 계층은 '요구 wrench'와 '실제 기체가 체감하는 거동' 사이의 간극을 메우는 경험적 보상층입니다. "
                    "튜닝 난이도는 있지만, 제대로 맞으면 상위 제어기가 덜 힘들어집니다."
                ),
                parameter_notes=(
                    ("`level_horizontal_compensation_*`", "수평 추진에 따라 생기는 heave leak 보상 정책입니다."),
                    ("`attitude_priority_horizontal_slowdown_*`", "자세 토크가 커질수록 horizontal 출력을 얼마나 줄일지 정합니다."),
                    ("`surge_pitch_moment_*`", "전진 추력이 만드는 pitch moment 보상 강도와 활성 조건입니다."),
                    ("`imu_pitch_hold_*`", "현재/목표 pitch 차이에 따라 추가 보상을 넣는 allocator 내부 hold 파라미터입니다."),
                ),
            ),
            ReviewItem(
                heading="`init_matrices()`",
                source_names=("init_matrices",),
                meaning=(
                    "thruster 위치/방향으로 TAM, 수평 H 행렬, 수직 V 행렬, pseudo-inverse를 구성합니다."
                ),
                impact=(
                    "기체 배치가 이 함수에 담깁니다. "
                    "행렬이 틀리면 heave만 줘도 pitch가 생기고, yaw만 줘도 sway가 생깁니다."
                ),
                review=(
                    "allocator 해석의 핵심입니다. "
                    "실제 모델과 이 함수의 thruster geometry가 맞는지 항상 함께 검증해야 합니다."
                ),
                details=(
                    "이 함수는 기체의 thruster 위치와 방향을 수학 모델로 고정하는 구간입니다. "
                    "수평 4기와 수직 4기의 각 레버암과 thrust 방향이 여기서 TAM과 pseudo-inverse 행렬로 변환됩니다.\n\n"
                    "따라서 allocator 문제의 상당수는 사실 이 함수에서 시작됩니다. yaw만 줬는데 sway가 섞이거나 heave만 줬는데 pitch가 생긴다면, 게인보다 먼저 이 geometry와 부호 정의가 실제 하드웨어와 일치하는지 검증해야 합니다."
                ),
                parameter_notes=(
                    ("`rear_vertical_bias`, `pitch_torque_gain`", "수직 thruster 분배 행렬이 실제로 어떤 성향을 띨지에 간접적으로 영향을 줍니다."),
                ),
            ),
            ReviewItem(
                heading="Allocation / Shaping 보조 함수",
                source_names=("apply_deadband", "add_component_with_headroom", "allocate_priority_components", "apply_slew_rate"),
                meaning=(
                    "출력 deadband 제거, 남은 헤드룸 안에서 성분 추가, priority allocation, slew-rate 제한을 담당합니다."
                ),
                impact=(
                    "수치적으로는 같은 wrench라도 actuator 친화성은 크게 달라집니다. "
                    "이 함수들이 saturation 시 거동과 응답 속도를 정합니다."
                ),
                review=(
                    "현재 allocator가 heuristic allocator라는 점이 가장 잘 드러나는 구간입니다. "
                    "이 계층 덕분에 practical하지만, 최적화 기반 allocator와는 성격이 다릅니다."
                ),
                details=(
                    "이 함수들은 이상적인 연속 wrench를 실제 actuator가 낼 수 있는 명령열로 다듬는 후처리 계층입니다. "
                    "`add_component_with_headroom()`과 `allocate_priority_components()`는 saturation 가까이에서 어떤 성분을 먼저 살릴지 결정하고, "
                    "`apply_deadband()`는 너무 작은 명령을 없애 chatter를 줄이며, `apply_slew_rate()`는 한 번에 급격히 바뀌는 thrust를 막습니다.\n\n"
                    "즉 allocator 품질은 행렬 해석만으로 끝나지 않습니다. 같은 wrench라도 이런 shaping 계층이 다르면 실제 모터 체감, 발열, 응답성, 자세 안정성이 크게 달라집니다."
                ),
                parameter_notes=(
                    ("`slew_rate`", "출력이 시간적으로 얼마나 빨리 변할 수 있는지 제한합니다."),
                    ("`output_deadband`", "아주 작은 thrust 명령을 제거합니다."),
                    ("`max_output`, `output_scale`", "최종 thrust의 절대 크기와 전체 배율을 정합니다."),
                ),
            ),
            ReviewItem(
                heading="`callback()`",
                source_names=("callback",),
                meaning=(
                    "최종 wrench를 받아 부호 해석, scaling, horizontal/vertical allocation, compensation, shaping, saturation, publish까지 한 번에 수행합니다."
                ),
                impact=(
                    "이 함수가 곧 thruster command의 최종 형태를 결정합니다. "
                    "어떤 축 요구가 우선되는지, saturation에서 무엇을 희생하는지가 여기서 결정됩니다."
                ),
                review=(
                    "코드의 의도는 분명합니다. "
                    "다만 `Fx` 하드코딩 반전과 heuristic saturation은 읽는 사람에게 반드시 설명이 필요합니다."
                ),
                details=(
                    "이 함수는 allocator의 본체입니다. 먼저 입력 wrench를 읽고 내부 좌표계 기준으로 `fx`, `fy`, `fz`, `tx`, `ty`, `tz`를 해석합니다. "
                    "여기서 `fx = -msg.force.x`처럼 축 부호를 내부 convention에 맞게 뒤집는 부분이 있기 때문에, 이 함수는 좌표계 해석의 실제 출발점이기도 합니다.\n\n"
                    "그 다음 수평 성분(`Fx`, `Fy`, `Tz`)과 수직 성분(`Fz`, `Tx`, `Ty`)을 분리해서 allocation을 수행하고, "
                    "보상 항들을 추가한 뒤 priority allocation, normalization, deadband, slew-rate를 순서대로 적용합니다. "
                    "즉 이 함수는 단순 수학 공식이 아니라 '요구 wrench를 실제 thruster array가 낼 수 있는 형태로 번역하는 전체 파이프라인'입니다."
                ),
                parameter_notes=(
                    ("`horizontal_output_gain`, `vertical_output_gain`, `yaw_output_gain`, `heave_gain`", "입력 wrench를 thruster 명령 크기로 스케일링하는 데 직접 들어갑니다."),
                    ("`torque_first_allocation`", "수직 allocation에서 heave와 자세 토크 중 무엇을 먼저 보장할지 결정합니다."),
                    ("`level_horizontal_compensation_*`, `surge_pitch_moment_*`, `imu_pitch_hold_*`", "allocation 중간에 추가되는 보상 항의 크기와 활성 조건을 정합니다."),
                    ("`slew_rate`, `max_output`, `output_scale`, `output_deadband`", "최종 thruster 출력을 다듬는 마지막 shaping 파라미터입니다."),
                ),
            ),
            ReviewItem(
                heading="`on_parameter_update()`",
                source_names=("on_parameter_update",),
                meaning=(
                    "allocation gain, compensation, slew, output scale을 런타임에 갱신합니다."
                ),
                impact=(
                    "allocator tuning을 빠르게 반복할 수 있게 해줍니다."
                ),
                review=(
                    "실험 속도를 크게 올리는 좋은 구조입니다. "
                    "이번 리뷰 문서에서 함수-파라미터 연결을 설명하는 이유도 이 함수 때문입니다."
                ),
                details=(
                    "allocator는 실제로 현장 튜닝 비중이 매우 높은 모듈입니다. thrust geometry는 고정되어 있어도, gain과 compensation만으로도 체감 조종감이 크게 달라집니다. "
                    "이 함수는 그 조정을 런타임에 가능하게 만들어 테스트 반복 속도를 높여 줍니다.\n\n"
                    "대신 문서가 부족하면 위험합니다. 어떤 파라미터가 allocation 행렬 이전에 적용되는지, 어떤 것은 compensation이고 어떤 것은 최종 shaping인지 구분되지 않으면 잘못된 축을 튜닝할 수 있기 때문입니다."
                ),
            ),
        ),
    ),
)


SUMMARY_TEXT = """
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
""".strip()

SUMMARY_HTML = """
<h3>전체 제어 흐름</h3>
<p>이 코드 구조는 여러 제어기가 각자 필요한 힘 또는 토크를 계산하고, <code>WrenchMerger</code>가 이를 하나의 <code>Wrench</code>로 병합한 뒤, <code>Allocator</code>가 8개 스러스터 명령으로 변환하는 방식입니다.</p>
<h3>단계 설명</h3>
<ul class="summary-list">
  <li>수동 입력: 조종기 또는 상위 제어 입력이 <code>/rov/wrench_manual</code> 형태로 들어옵니다.</li>
  <li>자세 제어: <code>attitude_controller.py</code>가 IMU 기반 <code>roll</code>, <code>pitch</code>, <code>yaw</code> 토크를 계산합니다.</li>
  <li>수심 제어: <code>depth_controller.py</code>가 목표 수심과 현재 수심 차이로 heave 명령을 계산합니다.</li>
  <li>위치 제어: <code>position_controller.py</code>가 DVL 기반 위치/속도 정보를 이용해 XY force를 계산합니다.</li>
  <li>병합: <code>wrench_merger.py</code>가 수동 입력과 자동 제어 출력을 우선순위 규칙으로 합칩니다.</li>
  <li>할당: <code>allocator_node.py</code>가 최종 wrench를 8개 스러스터 출력으로 변환합니다.</li>
</ul>
""".strip()

MODULE_CHAPTERS = {
    "allocator": 2,
    "attitude": 3,
}

ALLOCATOR_FUNCTION_DOCS = {
    "normalize": {
        "role": "입력 벡터를 단위 벡터로 정규화합니다. 스러스터 방향 벡터가 정확한 힘 방향만 표현하도록 만들기 위해 사용됩니다.",
        "why": "복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.",
        "impact": "이 함수의 결과는 같은 노드의 다음 계산 단계 또는 다른 제어 노드의 입력으로 사용됩니다. 따라서 값의 단위, 부호, 좌표계가 전체 ROV 움직임에 직접 영향을 줍니다.",
        "flow": ("입력 벡터를 `numpy array`로 변환합니다.", "노름이 너무 작으면 0 나눗셈을 피하기 위해 원래 벡터를 그대로 사용합니다.", "그 외에는 노름으로 나누어 단위 벡터를 반환합니다."),
    },
    "normalize_group_unit": {
        "role": "스러스터 그룹 출력의 최대 절댓값이 1을 넘으면 같은 비율로 전체를 축소합니다. 출력 포화 범위를 유지하면서 방향성은 보존합니다.",
        "why": "복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.",
        "impact": "스러스터 출력 분배에 직접 영향을 줍니다. 계산 결과는 최종 thruster command의 크기와 방향을 결정합니다.",
        "flow": ("입력 그룹 출력을 배열로 변환합니다.", "가장 큰 절댓값을 찾습니다.", "1보다 크면 전체를 같은 비율로 축소하고, 아니면 그대로 반환합니다."),
    },
    "quat_to_rotation_z_row": {
        "role": "IMU quaternion에서 body z축이 world 좌표계에서 향하는 방향 성분을 계산합니다. 기체가 기울어진 상태의 heave 보상 계산에 사용됩니다.",
        "why": "ROV 제어에서는 자세 표현과 좌표계 변환이 계속 필요하므로, 반복되는 수학 연산을 함수로 분리한 것입니다.",
        "impact": "이 함수의 결과는 같은 노드의 다음 계산 단계 또는 다른 제어 노드의 입력으로 사용됩니다. 따라서 값의 단위, 부호, 좌표계가 전체 ROV 움직임에 직접 영향을 줍니다.",
        "flow": ("입력 quaternion을 정규화합니다.", "정규화가 불가능할 정도로 작으면 기본 z축 `(0, 0, 1)`을 반환합니다.", "정규화된 quaternion으로 world 기준 body z축 방향 성분을 계산합니다."),
    },
    "quat_to_rpy": {
        "role": "quaternion 자세 표현을 roll, pitch, yaw 각도로 변환합니다. 사람이 이해하기 쉬운 자세 오차 및 보상 계산에 사용됩니다.",
        "why": "ROV 제어에서는 자세 표현과 좌표계 변환이 계속 필요하므로, 반복되는 수학 연산을 함수로 분리한 것입니다.",
        "impact": "이 함수의 결과는 같은 노드의 다음 계산 단계 또는 다른 제어 노드의 입력으로 사용됩니다. 따라서 값의 단위, 부호, 좌표계가 전체 ROV 움직임에 직접 영향을 줍니다.",
        "flow": ("입력 quaternion을 정규화합니다.", "roll, pitch, yaw를 순서대로 계산합니다.", "pitch는 asin 범위를 넘지 않도록 안전하게 처리합니다."),
    },
    "__init__": {
        "role": "ROS2 노드의 파라미터, 상태 변수, subscriber, publisher, timer를 초기화합니다. 해당 제어 노드가 시스템에 연결되는 시작점입니다.",
        "why": "노드가 실행되기 전에 필요한 파라미터, 통신 인터페이스, 상태 변수를 모두 준비해야 하기 때문에 사용됩니다.",
        "impact": "이 함수의 결과는 같은 노드의 다음 계산 단계 또는 다른 제어 노드의 입력으로 사용됩니다. 따라서 값의 단위, 부호, 좌표계가 전체 ROV 움직임에 직접 영향을 줍니다.",
        "flow": ("노드 이름을 설정합니다.", "ROS parameter를 선언하고 현재 값을 읽습니다.", "제어에 필요한 내부 상태 변수를 초기화합니다.", "subscriber와 publisher를 생성합니다.", "parameter callback을 등록하고 초기 설정값을 로그에 남깁니다."),
    },
    "imu_callback": {
        "role": "IMU 메시지를 수신하여 현재 자세, 각속도, 또는 z축 방향 정보를 내부 상태에 저장합니다.",
        "why": "ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.",
        "impact": "이 함수의 결과는 같은 노드의 다음 계산 단계 또는 다른 제어 노드의 입력으로 사용됩니다. 따라서 값의 단위, 부호, 좌표계가 전체 ROV 움직임에 직접 영향을 줍니다.",
        "flow": ("ROS2 IMU 메시지를 수신합니다.", "orientation에서 현재 world z축 방향과 roll/pitch/yaw를 계산합니다.", "내부 자세 상태와 `have_imu` 플래그를 갱신합니다."),
    },
    "cmd_attitude_callback": {
        "role": "외부에서 들어오는 목표 자세 명령을 내부 목표 roll/pitch 값으로 반영합니다.",
        "why": "ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.",
        "impact": "roll, pitch 자세 유지 토크와 allocator 내부 보상 기준에 영향을 줍니다.",
        "flow": ("ROS2 메시지를 수신합니다.", "NaN이 아닌 x/y 값을 읽습니다.", "목표 roll/pitch 내부 상태를 갱신합니다."),
    },
    "cmd_attitude_trim_callback": {
        "role": "trim 형태의 목표 자세를 수신하여 roll/pitch 보정 기준으로 사용합니다.",
        "why": "ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.",
        "impact": "roll, pitch 자세 유지 토크와 allocator 내부 보상 기준에 영향을 줍니다.",
        "flow": ("ROS2 메시지를 수신합니다.", "NaN이 아닌 x/y 값을 읽습니다.", "trim 기준으로 사용할 목표 roll/pitch를 갱신합니다."),
    },
    "output_scale_callback": {
        "role": "전체 스러스터 출력 스케일을 실시간으로 갱신합니다.",
        "why": "ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.",
        "impact": "최종 thruster command 전체 크기에 직접 영향을 줍니다.",
        "flow": ("ROS2 메시지를 수신합니다.", "입력 값을 0~1 범위로 clamp합니다.", "내부 `output_scale` 상태를 갱신합니다."),
    },
    "joy_speed_scale_callback": {
        "role": "조이스틱 속도 스케일을 전체 출력 스케일로 연결할지 결정합니다.",
        "why": "조종기 속도 모드와 allocator 출력 크기를 연동하기 위해 사용됩니다.",
        "impact": "조이스틱 기반 운용 시 최종 출력 감도와 최대 추력 수준에 직접 영향을 줍니다.",
        "flow": ("ROS2 메시지를 수신합니다.", "`use_joy_speed_scale_for_output`가 켜져 있는지 확인합니다.", "활성 상태면 `output_scale_callback()`을 재사용해 출력 스케일을 갱신합니다."),
    },
    "level_horizontal_heave_compensation": {
        "role": "기체가 roll/pitch로 기울어진 상태에서 수평 힘이 수직 방향으로 새는 효과를 보상하기 위한 heave 값을 계산합니다.",
        "why": "복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.",
        "impact": "수평 이동 중 기체가 의도치 않게 뜨거나 가라앉는 현상에 직접 영향을 줍니다.",
        "flow": ("보상 기능이 켜져 있고 IMU가 유효한지 확인합니다.", "현재 body z축의 world 성분을 읽습니다.", "기울기 때문에 생기는 heave leak를 계산하고 제한값 안으로 clamp합니다."),
    },
    "attitude_priority_horizontal_scale": {
        "role": "roll/pitch 토크 요구가 큰 상황에서 수평 이동 명령을 줄여 자세 제어 우선권을 확보합니다.",
        "why": "수평 이동과 자세 복원이 동시에 포화될 때, 어떤 축을 우선할지 분명히 하기 위해 사용됩니다.",
        "impact": "roll, pitch 복원이 급한 상황에서 surge/sway 응답이 얼마나 희생될지 결정합니다.",
        "flow": ("slowdown 기능 활성 여부를 확인합니다.", "현재 자세 토크 요구 크기를 계산합니다.", "start/full 구간에 따라 1.0에서 `min_scale`까지 scale을 계산합니다."),
    },
    "surge_pitch_moment_compensation": {
        "role": "전진/후진 힘이 pitch 모멘트를 만드는 상황을 feed-forward 방식으로 보상합니다.",
        "why": "기체 구조상 surge 추력이 nose-up 또는 nose-down 성향을 만들 수 있어, 이를 allocator 단계에서 미리 상쇄하기 위해 사용됩니다.",
        "impact": "전진 시 pitch 흔들림과 수직 스러스터 부담 분배에 직접 영향을 줍니다.",
        "flow": ("보상 기능 활성 여부와 최소 surge 조건을 확인합니다.", "필요하면 목표 pitch 크기에 따른 gating을 적용합니다.", "설정 gain과 한계값을 사용해 추가 pitch 보상량을 계산합니다."),
    },
    "imu_pitch_hold_compensation": {
        "role": "현재 pitch와 목표 pitch의 차이를 이용해 surge 중 pitch 유지 보상량을 계산합니다.",
        "why": "전진 중 실제 pitch가 목표에서 벗어날 때 상위 controller 이전에 allocator 차원에서 추가 보정을 넣기 위해 사용됩니다.",
        "impact": "surge 상황에서 pitch hold가 얼마나 단단하게 유지될지에 영향을 줍니다.",
        "flow": ("보상 기능과 IMU 유효성을 확인합니다.", "최소 surge 조건과 pitch deadband를 검사합니다.", "pitch error에 gain을 곱하고 한계값 안으로 clamp합니다."),
    },
    "init_matrices": {
        "role": "스러스터 위치와 방향으로부터 TAM, 수평 allocation 행렬, 수직 allocation 행렬, pseudo-inverse를 생성합니다.",
        "why": "복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.",
        "impact": "스러스터 출력 분배에 직접 영향을 줍니다. 계산 결과는 최종 thruster command의 크기와 방향을 결정합니다.",
        "flow": ("각 스러스터의 위치와 방향을 정의합니다.", "전체 TAM과 수평/수직 그룹 행렬을 만듭니다.", "pseudo-inverse를 계산해 이후 allocation 단계에서 재사용할 수 있게 저장합니다."),
    },
    "apply_deadband": {
        "role": "작은 출력값을 0으로 만들어 스러스터 미세 떨림이나 불필요한 명령을 줄입니다.",
        "why": "복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.",
        "impact": "아주 작은 thruster command를 제거하여 actuator chatter와 불필요한 소비전력을 줄입니다.",
        "flow": ("입력을 배열로 변환합니다.", "`output_deadband`보다 작은 값을 0으로 만듭니다.", "deadband가 적용된 결과를 반환합니다."),
    },
    "add_component_with_headroom": {
        "role": "기존 출력에 추가 제어 성분을 더할 때 -1~1 범위를 넘지 않도록 남은 headroom만큼만 추가합니다.",
        "why": "여러 제어 성분을 단순 합산하면 saturation으로 우선순위가 무너질 수 있어, 남은 출력 공간을 계산해 안전하게 합치기 위해 사용됩니다.",
        "impact": "어떤 제어 성분이 saturation 상황에서 살아남는지에 직접 영향을 줍니다.",
        "flow": ("기존 출력과 추가 성분을 배열로 변환합니다.", "추가 성분이 매우 작으면 기존 출력을 그대로 사용합니다.", "합산 결과가 한계를 넘으면 scale을 줄여 headroom 안에서만 더합니다."),
    },
    "allocate_priority_components": {
        "role": "여러 제어 성분을 우선순위 순서대로 합성합니다. 먼저 들어온 성분이 출력 공간을 우선 사용합니다.",
        "why": "복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.",
        "impact": "heave, roll, pitch 같은 성분 중 어떤 항이 saturation 시 우선권을 갖는지 결정합니다.",
        "flow": ("초기 출력을 0으로 시작합니다.", "components를 순서대로 순회합니다.", "`add_component_with_headroom()`으로 우선순위를 유지하며 합성합니다."),
    },
    "apply_slew_rate": {
        "role": "이전 출력과 목표 출력 사이의 변화량을 시간 기준으로 제한합니다. 스러스터 명령의 급격한 변화를 줄입니다.",
        "why": "스러스터와 전원 계통에 갑작스러운 명령 변화가 가해지는 것을 줄이기 위해 사용됩니다.",
        "impact": "최종 thruster command의 응답 속도와 부드러움에 직접 영향을 줍니다.",
        "flow": ("현재 시각과 이전 시각으로 `dt`를 계산합니다.", "목표 출력과 이전 출력 차이를 구합니다.", "축별 최대 변화량을 제한한 뒤 새 출력을 저장하고 반환합니다."),
    },
    "callback": {
        "role": "최종 Wrench 명령을 받아 수평/수직 allocation, 보상, normalization, scaling, slew-rate를 거쳐 thruster command를 발행합니다.",
        "why": "ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.",
        "impact": "스러스터 출력 분배에 직접 영향을 줍니다. 계산 결과는 최종 thruster command의 크기와 방향을 결정합니다.",
        "flow": ("`/rov/wrench_cmd`에서 force와 torque를 읽습니다.", "surge, sway, heave, roll, pitch, yaw 성분을 분리합니다.", "기울어진 자세에서 수평 이동 시 필요한 heave 보상량을 계산합니다.", "수평 스러스터 그룹과 수직 스러스터 그룹으로 나누어 pseudo-inverse allocation을 수행합니다.", "토크 우선 또는 heave 우선 정책에 따라 vertical 출력을 합성합니다.", "출력 normalization, sign, output_scale, max_output, slew-rate, deadband를 적용합니다.", "`Float64MultiArray`로 8개 thruster command를 발행합니다."),
    },
    "on_parameter_update": {
        "role": "ROS2 runtime parameter 변경을 노드 내부 변수에 반영합니다.",
        "why": "실제 로봇 테스트 중 gain과 제한값을 노드를 재시작하지 않고 바꾸기 위해 사용됩니다.",
        "impact": "이 함수의 결과는 같은 노드의 다음 계산 단계 또는 다른 제어 노드의 입력으로 사용됩니다. 따라서 값의 단위, 부호, 좌표계가 전체 ROV 움직임에 직접 영향을 줍니다.",
        "flow": ("변경 요청된 parameter 목록을 순회합니다.", "parameter 이름에 맞는 내부 변수를 갱신합니다.", "각도 단위 parameter는 필요한 경우 radian으로 변환합니다.", "갱신 결과를 log로 남기고 `SetParametersResult`를 반환합니다."),
    },
    "main": {
        "role": "rclpy를 초기화하고 노드를 생성한 뒤 spin을 수행합니다.",
        "why": "ROS2 노드 생명주기를 시작하고 종료 처리를 안정적으로 수행하기 위해 사용됩니다.",
        "impact": "이 함수는 allocator 노드가 실제 ROS graph 안에서 동작하기 시작하는 진입점입니다.",
        "flow": ("`rclpy.init()`으로 ROS2를 초기화합니다.", "노드 객체를 생성합니다.", "`rclpy.spin()`으로 callback 처리를 시작합니다.", "종료 시 노드를 정리하고 `rclpy.shutdown()`을 호출합니다."),
    },
}

ATTITUDE_FUNCTION_DOCS = {
    "clamp": {"role": "값을 지정된 최소/최대 범위 안으로 제한합니다.", "why": "복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.", "impact": "roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.", "flow": ("입력값과 최소/최대 한계를 받습니다.", "최솟값보다 작으면 최솟값으로 제한합니다.", "최댓값보다 크면 최댓값으로 제한하고, 범위 안이면 그대로 반환합니다.")},
    "vec_norm": {"role": "3차원 벡터의 크기를 계산합니다. 각속도 크기 판단 등에 사용됩니다.", "why": "복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.", "impact": "roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.", "flow": ("x, y, z 성분을 읽습니다.", "각 성분 제곱합을 계산합니다.", "제곱근을 취해 벡터 크기를 반환합니다.")},
    "quat_normalize": {"role": "quaternion을 단위 quaternion으로 정규화합니다.", "why": "ROV 제어에서는 자세 표현과 좌표계 변환이 계속 필요하므로, 반복되는 수학 연산을 함수로 분리한 것입니다.", "impact": "roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.", "flow": ("입력 quaternion 성분을 분리합니다.", "노름을 계산합니다.", "노름이 너무 작으면 기본 단위 quaternion을 반환하고, 아니면 정규화 결과를 반환합니다.")},
    "quat_conj": {"role": "quaternion의 켤레를 계산합니다. 회전 역변환에 사용됩니다.", "why": "ROV 제어에서는 자세 표현과 좌표계 변환이 계속 필요하므로, 반복되는 수학 연산을 함수로 분리한 것입니다.", "impact": "roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.", "flow": ("입력 quaternion 성분을 분리합니다.", "벡터부 부호를 반전합니다.", "스칼라부는 유지한 채 켤레 quaternion을 반환합니다.")},
    "quat_mul": {"role": "두 quaternion의 곱을 계산합니다. 자세 오차 또는 벡터 회전에 사용됩니다.", "why": "ROV 제어에서는 자세 표현과 좌표계 변환이 계속 필요하므로, 반복되는 수학 연산을 함수로 분리한 것입니다.", "impact": "roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.", "flow": ("두 quaternion의 성분을 각각 분리합니다.", "Hamilton product 공식을 적용합니다.", "곱셈 결과 quaternion을 반환합니다.")},
    "quat_to_rpy": {"role": "quaternion 자세 표현을 roll, pitch, yaw 각도로 변환합니다. 사람이 이해하기 쉬운 자세 오차 및 보상 계산에 사용됩니다.", "why": "ROV 제어에서는 자세 표현과 좌표계 변환이 계속 필요하므로, 반복되는 수학 연산을 함수로 분리한 것입니다.", "impact": "roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.", "flow": ("quaternion 성분으로 roll을 계산합니다.", "pitch를 계산하며 asin 범위를 넘어설 경우 안전하게 처리합니다.", "yaw를 계산해 세 각도를 반환합니다.")},
    "rpy_to_quat": {"role": "roll, pitch, yaw 목표를 quaternion으로 변환합니다.", "why": "ROV 제어에서는 자세 표현과 좌표계 변환이 계속 필요하므로, 반복되는 수학 연산을 함수로 분리한 것입니다.", "impact": "roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.", "flow": ("roll, pitch, yaw의 half-angle 삼각함수를 계산합니다.", "quaternion 성분을 조합합니다.", "정규화된 목표 quaternion을 반환합니다.")},
    "wrap_to_pi": {"role": "각도를 -pi~pi 범위로 정규화합니다. yaw wrap 문제를 방지합니다.", "why": "ROV 제어에서는 자세 표현과 좌표계 변환이 계속 필요하므로, 반복되는 수학 연산을 함수로 분리한 것입니다.", "impact": "roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.", "flow": ("입력 각도의 sin과 cos를 계산합니다.", "`atan2`를 이용해 같은 방향의 대표 각도로 변환합니다.", "-pi~pi 범위의 각도를 반환합니다.")},
    "__init__": {"role": "ROS2 노드의 파라미터, 상태 변수, subscriber, publisher, timer를 초기화합니다. 해당 제어 노드가 시스템에 연결되는 시작점입니다.", "why": "노드가 실행되기 전에 필요한 파라미터, 통신 인터페이스, 상태 변수를 모두 준비해야 하기 때문에 사용됩니다.", "impact": "roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.", "flow": ("노드 이름을 설정합니다.", "ROS parameter를 선언하고 현재 값을 읽습니다.", "제어에 필요한 내부 상태 변수를 초기화합니다.", "subscriber와 publisher를 생성합니다.", "timer와 parameter callback을 등록합니다.", "초기 설정값을 log로 출력합니다.")},
    "_update_target_quaternion": {"role": "현재 target roll/pitch/yaw로부터 목표 quaternion을 갱신합니다.", "why": "복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.", "impact": "roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.", "flow": ("현재 target roll/pitch/yaw를 읽습니다.", "`rpy_to_quat()`로 목표 quaternion을 계산합니다.", "target initialized 상태를 참으로 갱신합니다.")},
    "_force_level_roll_pitch_target": {"role": "level target과 trim 값을 이용해 roll/pitch 목표를 강제로 갱신합니다.", "why": "복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.", "impact": "roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.", "flow": ("level target 사용 여부를 확인합니다.", "target roll/pitch와 trim 값을 radian으로 변환해 목표값을 갱신합니다.", "갱신된 목표로 quaternion을 다시 계산합니다.")},
    "_capture_current_attitude_as_target": {"role": "현재 자세를 제어 목표로 캡처합니다. 초기화 또는 제어 재활성화 시 사용됩니다.", "why": "복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.", "impact": "roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.", "flow": ("현재 yaw를 wrap 처리해 목표 yaw로 저장합니다.", "level mode이면 설정된 level/trim 목표를 사용하고, 아니면 현재 control roll/pitch를 사용합니다.", "새 목표 quaternion을 계산합니다.")},
    "_capture_current_yaw_as_target": {"role": "현재 yaw를 heading hold 목표로 캡처합니다.", "why": "복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.", "impact": "roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.", "flow": ("현재 yaw를 wrap 처리합니다.", "목표 yaw에 저장합니다.", "새 목표 quaternion을 계산합니다.")},
    "_set_control_enabled": {"role": "제어 enable 상태 변경 시 목표값, 적분항, 출력 상태를 초기화하거나 0 출력합니다.", "why": "복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.", "impact": "roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.", "flow": ("이전 enable 상태와 새 상태를 비교합니다.", "enable 전이 시 필터와 목표 자세, 적분항을 초기화합니다.", "disable 전이 시 0 torque publish와 상태 정리를 수행합니다.")},
    "imu_callback": {"role": "IMU 메시지를 수신하여 현재 자세, 각속도, 또는 z축 방향 정보를 내부 상태에 저장합니다.", "why": "ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.", "impact": "roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.", "flow": ("ROS2 메시지를 수신합니다.", "orientation을 정규화하고 roll/pitch/yaw를 계산합니다.", "각속도와 현재 자세 상태를 갱신하고 필요하면 초기 target을 캡처합니다.")},
    "manual_wrench_callback": {"role": "조종기 또는 상위 입력에서 들어오는 수동 Wrench 명령을 저장합니다.", "why": "ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.", "impact": "roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.", "flow": ("ROS2 메시지를 수신합니다.", "수동 wrench를 내부 상태에 저장합니다.", "이후 control loop가 사용할 최신 manual 입력으로 유지합니다.")},
    "cmd_attitude_callback": {"role": "외부에서 들어오는 목표 자세 명령을 내부 목표 roll/pitch/yaw 값으로 반영합니다.", "why": "ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.", "impact": "roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.", "flow": ("ROS2 메시지를 수신합니다.", "NaN이 아닌 목표 roll/pitch/yaw를 읽습니다.", "업데이트가 있으면 목표 quaternion을 다시 계산합니다.")},
    "cmd_attitude_trim_callback": {"role": "trim 형태의 목표 자세를 수신하여 roll/pitch 보정 기준으로 사용합니다.", "why": "ROS2 topic 기반 시스템에서 비동기 메시지를 받아 제어 상태를 최신 값으로 유지하기 위해 사용됩니다.", "impact": "roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.", "flow": ("ROS2 메시지를 수신합니다.", "NaN이 아닌 trim 값을 읽어 degree 기준 trim 상태를 갱신합니다.", "업데이트가 있으면 필요 시 level target을 다시 구성합니다.")},
    "_apply_deadband": {"role": "작은 torque 값을 0으로 만들어 미세 떨림과 불필요한 토크 출력을 줄입니다.", "why": "복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.", "impact": "roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.", "flow": ("입력 torque 크기를 확인합니다.", "`torque_deadband`보다 작으면 0으로 만듭니다.", "그 외에는 원래 값을 반환합니다.")},
    "_reset_control_attitude_filter": {"role": "roll/pitch 제어용 필터 상태를 현재 IMU 자세로 초기화합니다.", "why": "복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.", "impact": "roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.", "flow": ("현재 roll/pitch를 읽습니다.", "control roll/pitch 상태를 현재 자세로 맞춥니다.", "필터 초기화 완료 플래그를 켭니다.")},
    "_update_control_attitude_filter": {"role": "각속도 적분 예측과 IMU 측정을 섞어 roll/pitch 제어용 자세 값을 갱신합니다.", "why": "복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.", "impact": "roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.", "flow": ("필터 활성 여부와 dt 유효성을 확인합니다.", "body rate 적분으로 예측값을 계산합니다.", "measurement와의 차이를 제한된 correction으로 반영해 control roll/pitch를 갱신합니다.")},
    "_apply_rp_torque_slew": {"role": "roll/pitch 제어 토크 변화량을 제한합니다.", "why": "복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.", "impact": "roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.", "flow": ("dt로 허용 가능한 최대 토크 변화량을 계산합니다.", "현재 목표 torque를 이전 torque 주변 허용 범위로 clamp합니다.", "갱신된 torque를 저장하고 반환합니다.")},
    "_translation_tilt_feedforward": {"role": "수평 이동 중 목표 roll/pitch trim 유지에 필요한 feed-forward 토크를 계산합니다.", "why": "복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.", "impact": "roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.", "flow": ("기능 활성 여부를 확인합니다.", "surge/sway 입력에서 deadband를 뺀 유효 drive를 계산합니다.", "설정 gain과 최대치로 roll/pitch feed-forward torque를 만듭니다.")},
    "control_loop": {"role": "자세 오차와 각속도 damping으로 roll/pitch/yaw 토크를 계산하고 발행하는 주 제어 루프입니다.", "why": "복잡한 제어 계산을 작은 단위로 분리하여 역할을 명확히 하고, 다른 계산 단계에서 재사용하기 위해 사용됩니다.", "impact": "roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.", "flow": ("IMU와 target 초기화 여부를 확인합니다.", "dt를 계산하고 control_enabled 상태를 확인합니다.", "roll/pitch 필터를 갱신하고 목표 자세를 준비합니다.", "수동 wrench 입력에서 heave, surge, sway, yaw 명령을 읽습니다.", "roll/pitch/yaw 오차와 각속도 damping으로 제어 토크를 계산합니다.", "수동 yaw 조작 중에는 yaw hold를 양보하고, release 시 현재 yaw를 목표로 캡처합니다.", "계산된 토크를 limit와 slew-rate 처리 후 attitude torque topic으로 발행합니다.")},
    "on_parameter_update": {"role": "ROS2 runtime parameter 변경을 노드 내부 변수에 반영합니다.", "why": "실제 로봇 테스트 중 gain과 제한값을 노드를 재시작하지 않고 바꾸기 위해 사용됩니다.", "impact": "roll, pitch, yaw 자세 유지 토크에 영향을 준다. 특히 trim 자세, heading hold, rate damping 동작과 연결됩니다.", "flow": ("변경 요청된 parameter 목록을 순회합니다.", "parameter 이름에 맞는 내부 변수를 갱신합니다.", "각도 단위 parameter는 필요한 경우 radian으로 변환합니다.", "갱신 결과를 log로 남깁니다.", "성공 또는 실패 결과를 `SetParametersResult`로 반환합니다.")},
    "main": {"role": "rclpy를 초기화하고 노드를 생성한 뒤 spin을 수행합니다.", "why": "ROS2 노드 생명주기를 시작하고 종료 처리를 안정적으로 수행하기 위해 사용됩니다.", "impact": "이 함수는 attitude controller 노드가 실제 ROS graph 안에서 동작하기 시작하는 진입점입니다.", "flow": ("`rclpy.init()`으로 ROS2를 초기화합니다.", "노드 객체를 생성합니다.", "`rclpy.spin()`으로 callback 처리를 시작합니다.", "종료 시 노드를 정리하고 `rclpy.shutdown()`을 호출합니다.")},
}

CHAPTER_FUNCTION_DOCS = {
    "allocator": ALLOCATOR_FUNCTION_DOCS,
    "attitude": ATTITUDE_FUNCTION_DOCS,
}


DEF_RE = re.compile(r"^(\s*)def\s+([A-Za-z_][A-Za-z0-9_]*)\s*\(")
CLASS_RE = re.compile(r"^(\s*)class\s+([A-Za-z_][A-Za-z0-9_]*)\s*[:(]")


def slugify(text: str) -> str:
    slug = re.sub(r"[^a-zA-Z0-9]+", "-", text.lower()).strip("-")
    return slug or "section"


def code_path(config: ModuleConfig) -> Path:
    return CODE_DIR / config.source_filename


def extract_def_ranges(lines: list[str]) -> list[dict[str, int | str]]:
    defs: list[dict[str, int | str]] = []
    boundaries: list[tuple[int, int]] = []
    for index, line in enumerate(lines):
        match = DEF_RE.match(line)
        if match:
            defs.append(
                {
                    "name": match.group(2),
                    "indent": len(match.group(1)),
                    "start": index,
                }
            )
            boundaries.append((index, len(match.group(1))))
            continue

        class_match = CLASS_RE.match(line)
        if class_match:
            boundaries.append((index, len(class_match.group(1))))

    for index, item in enumerate(defs):
        indent = int(item["indent"])
        end = len(lines)
        current_start = int(item["start"])
        for next_start, next_indent in boundaries:
            if next_start <= current_start:
                continue
            if next_indent <= indent:
                end = next_start
                break
        item["end"] = end
    return defs


def extract_snippet(source: str, names: Iterable[str]) -> str:
    lines = source.splitlines()
    defs = extract_def_ranges(lines)
    snippets: list[str] = []
    for name in names:
        matched = next((item for item in defs if item["name"] == name), None)
        if matched is None:
            continue
        start = int(matched["start"])
        end = int(matched["end"])
        snippet = textwrap.dedent("\n".join(lines[start:end]).rstrip())
        if snippet:
            snippets.append(snippet)
    return "\n\n".join(snippets).strip()


def get_def_map(source: str) -> dict[str, dict[str, int | str]]:
    return {str(item["name"]): item for item in extract_def_ranges(source.splitlines())}


def get_signature(snippet: str) -> str:
    first_line = snippet.splitlines()[0].strip() if snippet.strip() else ""
    return first_line.removesuffix(":")


def get_parameter_names_from_signature(signature: str) -> list[str]:
    match = re.search(r"\((.*)\)", signature)
    if not match:
        return []
    raw = match.group(1).strip()
    if not raw:
        return []
    parts: list[str] = []
    current: list[str] = []
    depth = 0
    for char in raw:
        if char in "([":
            depth += 1
        elif char in ")]":
            depth = max(0, depth - 1)
        if char == "," and depth == 0:
            parts.append("".join(current).strip())
            current = []
            continue
        current.append(char)
    if current:
        parts.append("".join(current).strip())
    names: list[str] = []
    for part in parts:
        item = part.strip()
        if not item:
            continue
        item = item.split("=")[0].strip()
        item = item.split(":")[0].strip()
        names.append(item)
    return names


def format_input_description(name: str, signature: str) -> str:
    params = get_parameter_names_from_signature(signature)
    if not params:
        return "없음"
    return ", ".join(params)


def format_output_description(name: str, snippet: str) -> str:
    if name == "__init__":
        return "직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다."
    if name == "main":
        return "직접적인 return 값보다는 노드 실행과 종료 처리가 핵심 출력입니다."
    if name.endswith("_callback"):
        return "직접적인 return 값보다는 내부 상태 갱신 또는 ROS topic 발행이 핵심 출력입니다."
    if name == "on_parameter_update":
        return "파라미터 갱신 결과를 `SetParametersResult`로 반환하면서 내부 상태를 함께 갱신합니다."
    if "return " in snippet:
        return "계산 결과를 return하며, 호출한 제어 로직에서 다음 계산의 입력으로 사용됩니다."
    return "내부 상태 갱신이 중심이며, 필요 시 계산 결과를 return합니다."


def format_allocator_function_title(name: str) -> str:
    if name == "main":
        return "전역 함수.main()"
    if name in {"normalize", "normalize_group_unit", "quat_to_rotation_z_row", "quat_to_rpy"}:
        return f"전역 함수.{name}()"
    return f"AllocatorNode.{name}()"


def format_chapter_function_title(module_key: str, name: str) -> str:
    if module_key == "allocator":
        return format_allocator_function_title(name)
    if module_key == "attitude":
        if name == "main":
            return "전역 함수.main()"
        if name in {"clamp", "vec_norm", "quat_normalize", "quat_conj", "quat_mul", "quat_to_rpy", "rpy_to_quat", "wrap_to_pi"}:
            return f"전역 함수.{name}()"
        return f"AttitudeController.{name}()"
    return name


def render_flow_steps_markdown(steps: tuple[str, ...]) -> list[str]:
    return [f"  - {step}" for step in steps]


def render_flow_steps_html(steps: tuple[str, ...]) -> str:
    return "<ul class=\"flow-list\">" + "".join(f"<li>{inline_code(step)}</li>" for step in steps) + "</ul>"


def inline_code(text: str) -> str:
    parts = re.split(r"(`[^`]+`)", text)
    rendered: list[str] = []
    for part in parts:
        if part.startswith("`") and part.endswith("`"):
            rendered.append(f"<code>{html.escape(part[1:-1])}</code>")
        else:
            rendered.append(html.escape(part))
    return "".join(rendered)


def render_paragraphs(text: str) -> str:
    paragraphs = [segment.strip() for segment in text.split("\n\n") if segment.strip()]
    return "\n".join(f"<p>{inline_code(paragraph)}</p>" for paragraph in paragraphs)


def render_parameter_list_html(parameters: tuple[tuple[str, str], ...]) -> str:
    items = []
    for name, description in parameters:
        items.append(
            f"<li><strong>{inline_code(name)}</strong><span>{inline_code(description)}</span></li>"
        )
    return '<ul class="parameter-list">' + "".join(items) + "</ul>"


def render_parameter_notes_markdown(parameters: tuple[tuple[str, str], ...]) -> list[str]:
    return [f"- {name}: {description}" for name, description in parameters]


def build_function_map(source: str) -> list[str]:
    names: list[str] = []
    for line in source.splitlines():
        match = DEF_RE.match(line)
        if match:
            names.append(match.group(2))
    return names


def render_markdown_chapter_module(config: ModuleConfig, source: str) -> str:
    function_map = build_function_map(source)
    def_map = get_def_map(source)
    chapter_no = MODULE_CHAPTERS.get(config.key, 2)
    function_docs = CHAPTER_FUNCTION_DOCS[config.key]
    chapter_title = {
        "allocator": "최종 Wrench 명령을 8개 스러스터 명령으로 변환하는 Control Allocation 노드",
        "attitude": "IMU 기반 Roll/Pitch/Yaw 자세 유지 토크를 생성하는 자세 제어 노드",
    }.get(config.key, config.role_summary)
    chapter_description = {
        "allocator": "이 파일은 제어기가 계산한 6축 wrench를 실제 8개 스러스터 명령으로 바꾸는 마지막 단계입니다. 제어 성능뿐 아니라 실제 로봇 안전에도 직접 연결됩니다.",
        "attitude": "이 파일은 IMU로 현재 자세를 읽고 목표 자세와 비교하여 roll/pitch/yaw 토크를 만듭니다. 수심, 위치 유지 중에도 기체 자세가 무너지지 않도록 하는 기반 제어기입니다.",
    }.get(config.key, config.role_summary)
    lines: list[str] = [
        "# ROV Control Code Review - 함수별 설명 문서",
        "",
        f"{chapter_no}장. `{config.source_filename}`",
        "",
        chapter_title,
        "",
        chapter_description,
        "",
        f"- 파일: `{config.source_filename}`",
        f"- 함수 개수: {len(function_map)}",
        f"- 주요 역할: {chapter_title}",
        "",
    ]

    for index, name in enumerate(function_map, start=1):
        meta = function_docs.get(name)
        if meta is None:
            continue
        snippet = extract_snippet(source, (name,))
        signature = get_signature(snippet)
        params = format_input_description(name, signature)
        output = format_output_description(name, snippet)
        def_info = def_map.get(name)
        start = int(def_info["start"]) + 1 if def_info else 1
        end = int(def_info["end"]) if def_info else start
        lines.extend(
            [
                f"{chapter_no}장.{index} {format_chapter_function_title(config.key, name)}",
                "",
                f"- 위치: `{config.source_filename}:{start}-{end}`",
                f"- 입력: {params}",
                f"- 출력: {output}",
                f"- 역할: {meta['role']}",
                f"- 왜 사용했는가: {meta['why']}",
                f"- 제어 영향: {meta['impact']}",
                "- 내부 동작 흐름:",
                *render_flow_steps_markdown(tuple(meta["flow"])),
                "- 코드 일부:",
                "",
                "```python",
                snippet,
                "```",
                "",
            ]
        )

    lines.extend(
        [
            "## 전체 코드",
            "",
            "```python",
            source.rstrip(),
            "```",
            "",
        ]
    )
    return "\n".join(lines)


def render_html_chapter_module(config: ModuleConfig, source: str) -> str:
    function_map = build_function_map(source)
    def_map = get_def_map(source)
    chapter_no = MODULE_CHAPTERS.get(config.key, 2)
    function_docs = CHAPTER_FUNCTION_DOCS[config.key]
    chapter_title = {
        "allocator": "최종 Wrench 명령을 8개 스러스터 명령으로 변환하는 Control Allocation 노드",
        "attitude": "IMU 기반 Roll/Pitch/Yaw 자세 유지 토크를 생성하는 자세 제어 노드",
    }.get(config.key, config.role_summary)
    chapter_description = {
        "allocator": "이 파일은 제어기가 계산한 6축 wrench를 실제 8개 스러스터 명령으로 바꾸는 마지막 단계입니다. 제어 성능뿐 아니라 실제 로봇 안전에도 직접 연결됩니다.",
        "attitude": "이 파일은 IMU로 현재 자세를 읽고 목표 자세와 비교하여 roll/pitch/yaw 토크를 만듭니다. 수심, 위치 유지 중에도 기체 자세가 무너지지 않도록 하는 기반 제어기입니다.",
    }.get(config.key, config.role_summary)
    nav_links = "\n".join(
        f'<a href="#fn-{html.escape(name)}">{html.escape(name)}()</a>' for name in function_map if name in function_docs
    )
    sections: list[str] = []
    for index, name in enumerate(function_map, start=1):
        meta = function_docs.get(name)
        if meta is None:
            continue
        snippet = extract_snippet(source, (name,))
        signature = get_signature(snippet)
        params = format_input_description(name, signature)
        output = format_output_description(name, snippet)
        def_info = def_map.get(name)
        start = int(def_info["start"]) + 1 if def_info else 1
        end = int(def_info["end"]) if def_info else start
        sections.append(
            f"""
      <section id="fn-{html.escape(name)}" class="review-section">
        <div class="panel-heading">
          <div>
            <p class="section-kicker">Chapter {chapter_no}.{index}</p>
            <h2>{html.escape(format_chapter_function_title(config.key, name))}</h2>
          </div>
        </div>
        <div class="doc-grid">
          <article class="review-card code-card">
            <h3>코드 일부</h3>
            <pre><code>{html.escape(snippet)}</code></pre>
          </article>
          <article class="review-card prose-card">
            <dl class="doc-meta">
              <div><dt>위치</dt><dd><code>{html.escape(config.source_filename)}:{start}-{end}</code></dd></div>
              <div><dt>입력</dt><dd>{inline_code(params)}</dd></div>
              <div><dt>출력</dt><dd>{inline_code(output)}</dd></div>
              <div><dt>역할</dt><dd>{inline_code(meta["role"])}</dd></div>
              <div><dt>왜 사용했는가</dt><dd>{inline_code(meta["why"])}</dd></div>
              <div><dt>제어 영향</dt><dd>{inline_code(meta["impact"])}</dd></div>
              <div><dt>내부 동작 흐름</dt><dd>{render_flow_steps_html(tuple(meta["flow"]))}</dd></div>
            </dl>
          </article>
        </div>
      </section>
            """.strip()
        )

    return f"""<!DOCTYPE html>
<html lang="ko">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>{html.escape(config.title)}</title>
  <link rel="stylesheet" href="styles.css" />
</head>
<body class="detail-body">
  <div class="detail-shell">
    <aside class="detail-sidebar">
      <a class="back-link" href="index.html">← 리뷰 홈으로</a>
      <p class="eyebrow">ROV Control Code Review</p>
      <h1>{chapter_no}장. {html.escape(config.source_filename)}</h1>
      <p class="meta">{inline_code(chapter_title)}</p>
      <nav class="toc">
        <a href="#chapter">장 개요</a>
        {nav_links}
        <a href="#full-source">전체 코드</a>
      </nav>
    </aside>
    <main class="detail-content">
      <section id="chapter" class="hero">
        <div>
          <p class="eyebrow">Module Chapter</p>
          <h2>{html.escape(chapter_title)}</h2>
          <p>{inline_code(chapter_title)}</p>
          <p>{inline_code(chapter_description)}</p>
          <ul class="summary-list">
            <li>파일: <code>{html.escape(config.source_filename)}</code></li>
            <li>함수 개수: {len(function_map)}</li>
            <li>주요 역할: {inline_code(chapter_title)}</li>
          </ul>
        </div>
      </section>
      {' '.join(sections)}
      <section id="full-source" class="panel">
        <div class="panel-heading">
          <div>
            <p class="section-kicker">Full Source</p>
            <h2>전체 코드</h2>
          </div>
        </div>
        <details class="source-details" open>
          <summary>전체 파일 펼치기 / 접기</summary>
          <pre><code>{html.escape(source.rstrip())}</code></pre>
        </details>
      </section>
    </main>
  </div>
</body>
</html>
"""


def render_markdown_module(config: ModuleConfig, source: str) -> str:
    if config.key in CHAPTER_FUNCTION_DOCS:
        return render_markdown_chapter_module(config, source)

    function_map = build_function_map(source)
    lines: list[str] = [
        f"# {config.title}",
        "",
        f"대상 파일: `code_review/code/{config.source_filename}`",
        "",
        "## 역할",
        config.role_summary,
        "",
        "## 설계 해석",
        config.design_summary,
        "",
        "## 리뷰 초점",
        config.review_focus,
        "",
        "## 런타임 동작 해설",
        config.runtime_summary,
        "",
        "## 핵심 파라미터",
    ]
    lines.extend([f"- {name}: {description}" for name, description in config.parameters])
    lines.extend([
        "",
        "## 함수 맵",
    ])
    lines.extend(f"- `{name}()`" for name in function_map)

    lines.append("")
    lines.append("## 함수 리뷰")
    for item in config.items:
        snippet = extract_snippet(source, item.source_names)
        lines.extend(
            [
                "",
                f"### {item.heading}",
                "",
                "**의미**",
                "",
                item.meaning,
                "",
                "**영향**",
                "",
                item.impact,
                "",
                "**리뷰 메모**",
                "",
                item.review,
            ]
        )
        if item.details:
            lines.extend(
                [
                    "",
                    "**상세 해설**",
                    "",
                    item.details,
                ]
            )
        if item.parameter_notes:
            lines.extend(
                [
                    "",
                    "**이 함수와 관련된 파라미터**",
                    "",
                    *render_parameter_notes_markdown(item.parameter_notes),
                ]
            )
        lines.extend(
            [
                "",
                "```python",
                snippet,
                "```",
            ]
        )

    lines.extend(
        [
            "",
            "## 전체 코드",
            "",
            "```python",
            source.rstrip(),
            "```",
            "",
        ]
    )
    return "\n".join(lines)


def render_html_module(config: ModuleConfig, source: str) -> str:
    if config.key in CHAPTER_FUNCTION_DOCS:
        return render_html_chapter_module(config, source)

    function_map = build_function_map(source)
    nav_links = "\n".join(
        f'<a href="#{slugify(item.heading)}">{html.escape(item.heading)}</a>' for item in config.items
    )
    function_list = "\n".join(f"<li><code>{html.escape(name)}()</code></li>" for name in function_map)
    review_sections: list[str] = []
    for item in config.items:
        snippet = extract_snippet(source, item.source_names)
        review_sections.append(
            f"""
      <section id="{slugify(item.heading)}" class="review-section">
        <div class="section-heading">
          <div>
            <p class="section-kicker">Function Review</p>
            <h2>{html.escape(item.heading)}</h2>
          </div>
        </div>
        <div class="review-grid">
          <article class="review-card code-card">
            <h3>실제 코드</h3>
            <pre><code>{html.escape(snippet)}</code></pre>
          </article>
          <article class="review-card prose-card">
            <h3>의미</h3>
            {render_paragraphs(item.meaning)}
            <h3>영향</h3>
            {render_paragraphs(item.impact)}
            <h3>리뷰 메모</h3>
            {render_paragraphs(item.review)}
            {'<h3>상세 해설</h3>' + render_paragraphs(item.details) if item.details else ''}
            {'<h3>이 함수와 관련된 파라미터</h3>' + render_parameter_list_html(item.parameter_notes) if item.parameter_notes else ''}
          </article>
        </div>
      </section>
            """.strip()
        )

    return f"""<!DOCTYPE html>
<html lang="ko">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>{html.escape(config.title)}</title>
  <link rel="stylesheet" href="styles.css" />
</head>
<body class="detail-body">
  <div class="detail-shell">
    <aside class="detail-sidebar">
      <a class="back-link" href="index.html">← 리뷰 홈으로</a>
      <p class="eyebrow">Controller Code Review</p>
      <h1>{html.escape(config.title)}</h1>
      <p class="meta"><code>code_review/code/{html.escape(config.source_filename)}</code></p>
      <nav class="toc">
        <a href="#role">역할</a>
        <a href="#parameters">핵심 파라미터</a>
        <a href="#map">함수 맵</a>
        {nav_links}
        <a href="#full-source">전체 코드</a>
      </nav>
    </aside>

    <main class="detail-content">
      <section id="role" class="hero">
        <div>
          <p class="eyebrow">Module Overview</p>
          <h2>{html.escape(config.title)}</h2>
          {render_paragraphs(config.role_summary)}
          {render_paragraphs(config.design_summary)}
          {render_paragraphs(config.review_focus)}
          {render_paragraphs(config.runtime_summary)}
        </div>
      </section>

      <section id="parameters" class="panel">
        <div class="panel-heading">
          <div>
            <p class="section-kicker">Parameters</p>
            <h2>핵심 파라미터 설명</h2>
          </div>
        </div>
        <div class="review-card">
          {render_parameter_list_html(config.parameters)}
        </div>
      </section>

      <section id="map" class="panel">
        <div class="panel-heading">
          <div>
            <p class="section-kicker">Function Map</p>
            <h2>이 파일에 있는 함수들</h2>
          </div>
        </div>
        <div class="review-card">
          <ul class="function-map">
            {function_list}
          </ul>
        </div>
      </section>

      {' '.join(review_sections)}

      <section id="full-source" class="panel">
        <div class="panel-heading">
          <div>
            <p class="section-kicker">Full Source</p>
            <h2>전체 코드</h2>
          </div>
        </div>
        <details class="source-details" open>
          <summary>전체 파일 펼치기 / 접기</summary>
          <pre><code>{html.escape(source.rstrip())}</code></pre>
        </details>
      </section>
    </main>
  </div>
</body>
</html>
"""


def render_summary_markdown() -> str:
    lines = [
        "# Control Code Review",
        "",
        SUMMARY_TEXT,
        "",
        "## 코드 리뷰 페이지",
    ]
    for module in MODULES:
        lines.extend(
            [
                f"- `{module.title}`",
                f"  - Markdown: `code_review/controller_review/{module.markdown_filename}`",
                f"  - HTML: `code_review/controller_review/{module.html_filename}`",
            ]
        )
    lines.append("")
    return "\n".join(lines)


def render_index_html() -> str:
    cards = []
    for module in MODULES:
        cards.append(
            f"""
        <article class="module-card">
          <p class="eyebrow">Code Review</p>
          <h3>{html.escape(module.title)}</h3>
          <p>{html.escape(module.role_summary)}</p>
          <p class="module-note">{html.escape(module.review_focus)}</p>
          <div class="module-actions">
            <a class="primary-link" href="{html.escape(module.html_filename)}">상세 HTML 보기</a>
            <a class="secondary-link" href="{html.escape(module.markdown_filename)}">MD 보기</a>
          </div>
        </article>
            """.strip()
        )

    return f"""<!DOCTYPE html>
<html lang="ko">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>Controller Review</title>
  <link rel="stylesheet" href="styles.css" />
</head>
<body>
  <div class="page-shell">
    <aside class="sidebar">
      <div class="brand">
        <p class="eyebrow">Controller Review</p>
        <h1>Function-Based Code Review</h1>
        <p class="meta">실제 코드 기준으로 함수 의미, 영향, 리뷰 포인트를 정리한 리뷰 허브입니다.</p>
      </div>
      <nav class="toc">
        <a href="#overview">개요</a>
        <a href="#modules">코드별 리뷰</a>
      </nav>
    </aside>

    <main class="content">
      <section id="overview" class="hero">
        <div>
          <p class="eyebrow">Review Hub</p>
          <h2>코드를 누르면 실제 코드와 함께 함수 중심 리뷰 페이지로 이동합니다</h2>
          {SUMMARY_HTML}
        </div>
      </section>

      <section id="modules" class="panel">
        <div class="panel-heading">
          <div>
            <p class="section-kicker">Modules</p>
            <h2>코드별 상세 리뷰 페이지</h2>
          </div>
        </div>
        <div class="module-grid">
          {' '.join(cards)}
        </div>
      </section>
    </main>
  </div>
</body>
</html>
"""


def render_styles() -> str:
    return """\
:root {
  --bg: #08111f;
  --panel: rgba(16, 24, 40, 0.84);
  --panel-strong: #13233e;
  --line: rgba(156, 178, 216, 0.16);
  --text: #eef5ff;
  --muted: #abc0df;
  --blue: #67b8ff;
  --purple: #aa8cff;
  --shadow: 0 24px 60px rgba(0, 0, 0, 0.28);
  --radius: 22px;
}

* {
  box-sizing: border-box;
}

html {
  scroll-behavior: smooth;
}

body {
  margin: 0;
  font-family: "Inter", "Pretendard", "Noto Sans KR", sans-serif;
  background:
    radial-gradient(circle at top left, rgba(103, 184, 255, 0.18), transparent 28%),
    radial-gradient(circle at top right, rgba(170, 140, 255, 0.12), transparent 24%),
    linear-gradient(180deg, #07101d 0%, #091423 100%);
  color: var(--text);
}

a {
  color: inherit;
  text-decoration: none;
}

code {
  padding: 0.12rem 0.35rem;
  border-radius: 8px;
  background: rgba(255, 255, 255, 0.06);
  font-family: "JetBrains Mono", "Fira Code", monospace;
  font-size: 0.92em;
}

pre code {
  padding: 0;
  background: transparent;
  border-radius: 0;
}

.page-shell,
.detail-shell {
  display: grid;
  grid-template-columns: 290px 1fr;
  min-height: 100vh;
}

.sidebar,
.detail-sidebar {
  position: sticky;
  top: 0;
  align-self: start;
  height: 100vh;
  padding: 32px 24px;
  border-right: 1px solid var(--line);
  background: rgba(4, 10, 20, 0.76);
  backdrop-filter: blur(16px);
}

.content,
.detail-content {
  padding: 28px;
}

.eyebrow,
.section-kicker {
  margin: 0 0 10px;
  color: var(--blue);
  font-size: 0.78rem;
  letter-spacing: 0.16em;
  text-transform: uppercase;
}

.meta,
.hero p,
.module-card p,
.review-card p,
.source-details summary {
  color: var(--muted);
  line-height: 1.7;
}

.hero,
.panel,
.module-card,
.review-card,
.review-section,
.source-details {
  background: var(--panel);
  border: 1px solid var(--line);
  border-radius: var(--radius);
  box-shadow: var(--shadow);
}

.hero,
.panel,
.review-section {
  padding: 28px;
  margin-top: 24px;
}

.hero {
  margin-top: 0;
}

.toc {
  display: grid;
  gap: 10px;
  margin-top: 28px;
}

.toc a,
.back-link,
.primary-link,
.secondary-link {
  padding: 10px 12px;
  border-radius: 12px;
  transition: 0.2s ease;
}

.toc a:hover,
.back-link:hover,
.primary-link:hover,
.secondary-link:hover {
  background: rgba(103, 184, 255, 0.12);
}

.back-link {
  display: inline-block;
  margin-bottom: 20px;
  color: var(--muted);
}

.panel-heading,
.section-heading {
  display: flex;
  justify-content: space-between;
  gap: 16px;
  align-items: start;
  margin-bottom: 20px;
}

.module-grid {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 18px;
}

.summary-list {
  margin: 0;
  padding-left: 18px;
  color: var(--muted);
  line-height: 1.8;
}

.module-card,
.review-card {
  padding: 22px;
}

.module-note {
  min-height: 72px;
}

.module-actions {
  display: flex;
  gap: 10px;
  flex-wrap: wrap;
  margin-top: 16px;
}

.primary-link,
.secondary-link {
  border: 1px solid var(--line);
}

.primary-link {
  background: linear-gradient(135deg, rgba(103, 184, 255, 0.24), rgba(170, 140, 255, 0.18));
}

.review-grid {
  display: grid;
  grid-template-columns: minmax(420px, 1.05fr) minmax(320px, 0.95fr);
  gap: 18px;
}

.doc-grid {
  display: grid;
  grid-template-columns: minmax(420px, 1.05fr) minmax(340px, 0.95fr);
  gap: 18px;
}

.prose-card h3,
.code-card h3 {
  margin-top: 0;
}

.code-card pre,
.source-details pre {
  margin: 0;
  padding: 18px;
  overflow-x: auto;
  border-radius: 16px;
  border: 1px solid rgba(103, 184, 255, 0.14);
  background: rgba(4, 10, 20, 0.88);
  color: #dbe8ff;
  line-height: 1.6;
}

.function-map {
  margin: 0;
  padding-left: 18px;
  columns: 2;
  color: var(--muted);
  line-height: 1.9;
}

.parameter-list {
  margin: 0;
  padding: 0;
  list-style: none;
  display: grid;
  gap: 14px;
}

.doc-meta {
  margin: 0;
  display: grid;
  gap: 14px;
}

.doc-meta div {
  padding: 14px 16px;
  border-radius: 16px;
  border: 1px solid rgba(255, 255, 255, 0.06);
  background: rgba(255, 255, 255, 0.02);
}

.doc-meta dt {
  margin: 0 0 6px;
  color: var(--text);
  font-weight: 700;
}

.doc-meta dd {
  margin: 0;
  color: var(--muted);
  line-height: 1.8;
}

.flow-list {
  margin: 0;
  padding-left: 18px;
  color: var(--muted);
  line-height: 1.8;
}

.parameter-list li {
  display: grid;
  gap: 6px;
  padding: 14px 16px;
  border-radius: 16px;
  border: 1px solid rgba(255, 255, 255, 0.06);
  background: rgba(255, 255, 255, 0.02);
}

.parameter-list li strong {
  color: var(--text);
  font-weight: 600;
}

.parameter-list li span {
  color: var(--muted);
  line-height: 1.7;
}

.source-details {
  padding: 18px;
}

.source-details summary {
  cursor: pointer;
  margin-bottom: 14px;
}

@media (max-width: 1200px) {
  .page-shell,
  .detail-shell {
    grid-template-columns: 1fr;
  }

  .sidebar,
  .detail-sidebar {
    position: relative;
    height: auto;
    border-right: 0;
    border-bottom: 1px solid var(--line);
  }
}

@media (max-width: 900px) {
  .content,
  .detail-content,
  .hero,
  .panel,
  .review-section {
    padding: 20px;
  }

  .module-grid,
  .review-grid,
  .doc-grid {
    grid-template-columns: 1fr;
  }

  .function-map {
    columns: 1;
  }
}
"""


def write_text(path: Path, content: str) -> None:
    path.write_text(content.strip() + "\n", encoding="utf-8")


def main() -> None:
    for module in MODULES:
        source = code_path(module).read_text(encoding="utf-8")
        write_text(BASE_DIR / module.markdown_filename, render_markdown_module(module, source))
        write_text(BASE_DIR / module.html_filename, render_html_module(module, source))

    write_text(BASE_DIR / "control_code_review.md", render_summary_markdown())
    write_text(BASE_DIR / "index.html", render_index_html())
    write_text(BASE_DIR / "styles.css", render_styles())


if __name__ == "__main__":
    main()
