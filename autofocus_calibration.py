from picamera2 import Picamera2
from libcamera import controls

import cv2
import serial
import time
import csv
import json
import numpy as np
import os
import threading
import tempfile
import lgpio


# ============================================================
# MULTI-FOCUS PROJECTOR V4.2 - UART HEARTBEAT + ST SETTLE SIGNAL
#
# 핵심 원칙
# ------------------------------------------------------------
# 1) Q1/Q2/Q3/Q4는 끝까지 완전히 독립적으로 측정/판단한다.
# 2) Q끼리 위치 관계(좌/우, 상/하, 대각선)를 코드에 하드코딩하지 않는다.
# 3) Projector BEST ADC는 global max가 아니라 local peak의 "모양"으로 고른다.
# 4) Projector 선택의 중심 metric은 Raw Laplacian이다.
# 5) 밝기와 같이 증가하는 넓고 평평한 peak는 quality penalty를 준다.
# 6) Camera lens calibration은 Raw/CLAHE/Tenengrad를 모두 분석하고
#    Q마다 가장 신뢰도 높은 metric의 local peak를 독립 선택한다.
# 7) Camera Module 3 open-loop VCM 특성 때문에 lens switch 후
#    park -> target -> settle -> discard -> 영상 안정화 확인을 수행한다.
# 8) 마지막 Plane clustering은 "참고 결과"일 뿐 초점 선택에는 절대 사용하지 않는다.
#
# V4.2 변경점
# ------------------------------------------------------------
# ATmega PD4 -> 레벨시프터 -> Pi GPIO22 로 들어오는 ST(정착) 신호를
# 폴링해서 projector VCM 고정 3초 대기를 조기 종료한다.
#
# 엣지 인터럽트가 아니라 레벨 폴링을 쓰는 이유:
#   ATmega가 Pi의 대기 준비보다 먼저 정착하면 엣지를 놓쳐
#   오히려 매번 타임아웃까지 기다리게 된다.
#   레벨은 늦게 읽어도 HIGH가 유지되므로 이 문제가 없다.
#
# ST_MIN_SETTLE_TIME은 절대 생략하지 않는다.
#   ST는 "위치 오차가 작다"만 보장하고
#   기구 진동이 끝났다는 보장은 하지 않는다.
#
# Q layout
#          Q2 | Q1
#         ---------
#          Q3 | Q4
# ============================================================


# ============================================================
# USER CONFIG
# ============================================================

# ----------------------------
# UART
# ----------------------------
UART_PORT = "/dev/ttyAMA2"
BAUD_RATE = 115200

# 현재 projector target을 UART로 계속 재전송한다.
# 예: target=279이면 다음 target으로 바뀔 때까지 T279\n 을 반복 송신.
#
# 주의: ATmega는 같은 target이 재전송되면 제어 상태를 리셋하지 않는다.
#       (리셋하면 stable_count가 매번 0으로 돌아가 ST가 올라갈 수 없다)
UART_TARGET_HEARTBEAT_ENABLED = True
UART_TARGET_HEARTBEAT_INTERVAL = 0.20   # seconds (5 Hz)


# ----------------------------
# Projector VCM sweep
#
# ADC 범위는 하드코딩하지 않는다.
# 시작 시 ATmega 에 'M' 을 보내 PWM 0/350 실측 범위를 받고
# 그 안쪽에서만 스윕 타겟을 만든다.
# ----------------------------
RANGE_REQUEST_ENABLED = True
RANGE_REQUEST_TIMEOUT = 20.0    # 'M' 은 PWM 0/350 각 2.5초 + 여유

# 실측 범위 양 끝에서 이만큼 잘라낸다.
# 끝단은 PWM 이 레일에 붙어 정착이 안 되므로 제외한다.
SWEEP_MARGIN = 5

# 여유를 뺀 폭이 이보다 좁으면 마진을 줄인다.
SWEEP_MIN_SPAN = 24

ADC_STEP = 3
SWEEP_MAX_STEPS = 35            # 이보다 많아지면 step 을 자동으로 키운다

# 높은 ADC 쪽은 실제 초점 후보가 나오지 않는 영역이라 시간 절약을 위해
# 최종 sweep step이 결정된 뒤 마지막 N개 측정점을 제거한다.
# 낮은 ADC 쪽 시작점/앞마진은 절대 건드리지 않는다.
SWEEP_HIGH_ADC_TAIL_SKIP_STEPS = 10
SWEEP_MIN_REMAINING_STEPS = 8

# 범위 수신 실패 시 폴백 (실측값 기준)
FALLBACK_ADC_MIN = 323
FALLBACK_ADC_MAX = 393

# 런타임에 결정된다
ADC_START = None
ADC_END = None

PROJECTOR_SETTLE_TIME = 3.0
PROJECTOR_SAMPLES = 9
PROJECTOR_FRAME_INTERVAL = 0.05

PROJECTOR_SELECTION_METRIC = "RawLaplacian"

PROJECTOR_SETTLE_TIME = 3.0
PROJECTOR_SAMPLES = 9
PROJECTOR_FRAME_INTERVAL = 0.05

# 최종 projector BEST ADC 선택 metric
PROJECTOR_SELECTION_METRIC = "RawLaplacian"


# ----------------------------
# ST 정착 신호 (ATmega PD4 -> 레벨시프터 -> Pi GPIO22)
# ----------------------------
# ST를 폴링해 PROJECTOR_SETTLE_TIME 고정 대기를 조기 종료한다.
#
# ST_MIN_SETTLE_TIME은 절대 생략하면 안 된다.
# ST는 "오차가 작다"만 보장하고 기구 진동이 끝났다는 보장은 하지 않는다.
# 이 하한 없이 ST만 믿으면 흔들리는 렌즈를 촬영한다.
ST_ENABLED = True
ST_GPIO_PIN = 22
ST_MIN_SETTLE_TIME = 0.30           # VCM 정착 후 기계 진동 감쇠 하한
ST_COMMAND_GUARD_TIME = 0.05        # ATmega가 ST를 내릴 시간
ST_POLL_INTERVAL = 0.01             # 100Hz 폴링
# ST 대기 최대 시간. PROJECTOR_SETTLE_TIME 과 분리한다.
# ST 가 안 오면 그 데이터는 신뢰할 수 없으므로 넉넉히 기다린다.
ST_TIMEOUT = 2.0                    # GPIO ST가 안 뜨는 비정상 대기 상한

# 타임아웃 시 타겟을 재전송하고 한 번 더 기다린다.
ST_RETRY_COUNT = 1

# GPIO ST가 노이즈/속도판정 때문에 LOW인 채 남아도,
# 실제 Hall ADC가 target 근처에 일정 시간 머물면 소프트 정착으로 인정한다.
SOFT_SETTLE_ENABLED = True
SOFT_SETTLE_ERROR_COUNTS = 2
SOFT_SETTLE_HOLD_TIME = 0.15       # target±2에서 150 ms 연속
SOFT_SETTLE_QUERY_INTERVAL = 0.05  # Q query 20 Hz
SOFT_SETTLE_QUERY_TIMEOUT = 0.06


# ============================================================
# Camera lens calibration
# ============================================================
# Projector가 한 위치에서 심하게 defocus면 camera lens curve도 flat할 수 있으므로
# 여러 reference ADC를 generic하게 확인한다.
# 위치 관계와 무관하며, Q마다 가장 quality가 좋은 run을 따로 선택한다.
# 런타임에 측정 범위로부터 생성한다
CAMERA_CAL_REFERENCE_ADC_SEQUENCE = []

CAMERA_LENS_SWEEP_STEP = 0.20
CAMERA_LENS_CAL_SETTLE_TIME = 0.18
CAMERA_LENS_DISCARD_FRAMES = 3
CAMERA_LENS_CAL_SAMPLES = 5
CAMERA_LENS_CAL_FRAME_INTERVAL = 0.03

# Camera calibration에서는 3 metric 중 Q별로 가장 peak quality가 높은 metric 사용
CAMERA_CAL_METRICS = [
    "RawLaplacian",
    "ClaheLaplacian",
    "Tenengrad",
]

CAMERA_CAL_QUALITY_MIN = 0.45

# ------------------------------------------------------------
# Camera lens left/right pair consistency check
#
# 사용자 기준:
#   Q2 | Q1
#  ---------
#   Q3 | Q4
#
# 같은 렌즈의 왼쪽(Q2,Q3), 오른쪽(Q1,Q4)은
# 최종 Camera LensPosition이 서로 거의 같아야 한다고 보고,
# 차이가 이 값 이상이면 Camera Lens calibration만 다시 수행한다.
# Projector ADC sweep은 아직 시작 전이므로 다시 하지 않는다.
# ------------------------------------------------------------
CAMERA_PAIR_MAX_DELTA = 0.20       # diopter, pass condition: delta < 0.20
CAMERA_PAIR_MAX_ATTEMPTS = 4       # 최초 1회 + 재시도 최대 3회


# ============================================================
# Camera lens switching during projector sweep
# ============================================================
CAMERA_SWITCH_USE_UNIDIRECTIONAL_APPROACH = True
CAMERA_LENS_PARK_POSITION = 0.0

CAMERA_SWITCH_PARK_SETTLE_TIME = 0.20
CAMERA_SWITCH_TARGET_SETTLE_TIME = 0.55
CAMERA_SWITCH_DISCARD_FRAMES = 5

# 영상 기반 안정화 확인
CAMERA_STABILITY_METRIC = "Tenengrad"
CAMERA_STABILITY_WINDOW = 3
CAMERA_STABILITY_REL_SPREAD = 0.03
CAMERA_STABILITY_TIMEOUT = 1.50
CAMERA_STABILITY_FRAME_INTERVAL = 0.05
CAMERA_STABILITY_ABORT_ON_TIMEOUT = False


# ============================================================
# Exposure / White balance
# ============================================================
AE_CONVERGE_TIME = 3.0
EXPOSURE_LOCK_SETTLE_TIME = 0.5


# ============================================================
# Image processing
# ============================================================
CLAHE_CLIP_LIMIT = 2.0
CLAHE_TILE_GRID = (8, 8)
GAUSSIAN_KERNEL = (3, 3)
GAUSSIAN_SIGMA = 0


# ============================================================
# Local peak analysis
# ============================================================
# candidate 주변 몇 sample까지 local baseline / brightness correlation에 사용할지
PEAK_ANALYSIS_RADIUS = 4

# quality score 정규화 기준
PEAK_REL_PROMINENCE_GOOD = 0.25
PEAK_CURVATURE_GOOD = 0.20
PEAK_SNR_GOOD = 8.0

# Half-prominence width가 2 point보다 넓어질수록 완만한 penalty
PEAK_WIDTH_PENALTY_PER_EXTRA_POINT = 0.30

# Positive brightness correlation penalty
# corr <= 0.35 : penalty 없음
# corr -> 1.0 : 최대 55% 감점
PEAK_BRIGHTNESS_CORR_START = 0.35
PEAK_BRIGHTNESS_CORR_MAX_PENALTY = 0.55

# sweep edge 주변 penalty
PEAK_EDGE_PENALTY_EDGE = 0.55       # edge sample 자체
PEAK_EDGE_PENALTY_DISTANCE1 = 0.78
PEAK_EDGE_PENALTY_DISTANCE2 = 0.92

# 최종 confidence용 기준
PEAK_LOW_QUALITY_THRESHOLD = 0.45
PEAK_AMBIGUOUS_MARGIN = 0.15

# 사후 plane candidate clustering tolerance
# 절대 초점판단에는 사용하지 않는다.
PLANE_CLUSTER_TOLERANCE_ADC = 4.5


# ============================================================
# Brightness diagnostics
# ============================================================
BRIGHTNESS_DARK = 40.0
BRIGHTNESS_GOOD_LOW = 60.0
BRIGHTNESS_GOOD_HIGH = 200.0
BRIGHTNESS_SATURATED = 230.0


# ============================================================
# Camera / display
# ============================================================
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720

WINDOW_NAME = "Multi-Focus Projector V4"
MOVE_TO_SECOND_MONITOR = True
SECOND_MONITOR_X = 700
SECOND_MONITOR_Y = 50


# ============================================================
# Quadrants - center 50% ROI
# ============================================================
ROI_X1 = FRAME_WIDTH // 4
ROI_X2 = (FRAME_WIDTH * 3) // 4
ROI_Y1 = FRAME_HEIGHT // 4
ROI_Y2 = (FRAME_HEIGHT * 3) // 4

ROI_CENTER_X = (ROI_X1 + ROI_X2) // 2
ROI_CENTER_Y = (ROI_Y1 + ROI_Y2) // 2

QUADRANTS = {
    # 사용자 기준:
    # Q1 = 오른쪽 위, Q2 = 왼쪽 위, Q3 = 왼쪽 아래, Q4 = 오른쪽 아래
    "Q1": (
        ROI_CENTER_X,
        ROI_Y1,
        ROI_X2 - ROI_CENTER_X,
        ROI_CENTER_Y - ROI_Y1,
    ),
    "Q2": (
        ROI_X1,
        ROI_Y1,
        ROI_CENTER_X - ROI_X1,
        ROI_CENTER_Y - ROI_Y1,
    ),
    "Q3": (
        ROI_X1,
        ROI_CENTER_Y,
        ROI_CENTER_X - ROI_X1,
        ROI_Y2 - ROI_CENTER_Y,
    ),
    "Q4": (
        ROI_CENTER_X,
        ROI_CENTER_Y,
        ROI_X2 - ROI_CENTER_X,
        ROI_Y2 - ROI_CENTER_Y,
    ),
}

QUADRANT_NAMES = {
    "Q1": "Top Right",
    "Q2": "Top Left",
    "Q3": "Bottom Left",
    "Q4": "Bottom Right",
}

QUADRANT_ORDER = ["Q1", "Q2", "Q3", "Q4"]


# ============================================================
# Hardware init
# ============================================================
print("UART 초기화 중...")
ser = serial.Serial(
    port=UART_PORT,
    baudrate=BAUD_RATE,
    timeout=1,
)
time.sleep(0.5)
print("UART 준비 완료")

print("Camera Module 3 초기화 중...")
picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={
        "size": (FRAME_WIDTH, FRAME_HEIGHT),
        "format": "RGB888",
    }
)
picam2.configure(config)
picam2.start()
time.sleep(2.0)
print("Camera Module 3 준비 완료")

required_controls = [
    "AfMode",
    "LensPosition",
]

for control_name in required_controls:
    if control_name not in picam2.camera_controls:
        raise RuntimeError(
            f"Camera control 지원 안 됨: {control_name}"
        )

lens_control = picam2.camera_controls["LensPosition"]
CAMERA_LENS_MIN = float(lens_control[0])
CAMERA_LENS_MAX = float(lens_control[1])
CAMERA_LENS_DEFAULT = float(lens_control[2])

print()
print("============================================")
print(" CAMERA LENS INFORMATION")
print("============================================")
print(f"Lens Min     = {CAMERA_LENS_MIN:.4f}")
print(f"Lens Max     = {CAMERA_LENS_MAX:.4f}")
print(f"Lens Default = {CAMERA_LENS_DEFAULT:.4f}")
print()

CLAHE = cv2.createCLAHE(
    clipLimit=CLAHE_CLIP_LIMIT,
    tileGridSize=CLAHE_TILE_GRID,
)


# ============================================================
# ST 정착 신호 GPIO 초기화
#
# ATmega PD4 -> 레벨시프터 HV2/LV2 -> Pi GPIO22 (물리 15번)
#
# 풀다운을 걸어 ATmega가 꺼져 있을 때 HIGH로 오인하지 않게 한다.
# 초기화 실패 시에도 프로그램은 계속 진행하고
# 기존 고정 대기 방식으로 자동 폴백한다.
# ============================================================
_st_handle = None

if ST_ENABLED:
    try:
        _st_handle = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_input(
            _st_handle,
            ST_GPIO_PIN,
            lgpio.SET_PULL_DOWN,
        )
        print(
            f"ST signal = ON (GPIO{ST_GPIO_PIN}, "
            f"min floor {ST_MIN_SETTLE_TIME:.2f}s, "
            f"timeout {PROJECTOR_SETTLE_TIME:.2f}s)"
        )
    except Exception as e:
        print(f"ST GPIO 초기화 실패, 고정 대기로 진행: {e}")
        _st_handle = None
else:
    print("ST signal = OFF (fixed settle time)")


# ============================================================
# ST 통계 (마지막에 절약 시간 요약)
# ============================================================
_st_stats = {
    "Waits": 0,
    "EarlyExits": 0,
    "Timeouts": 0,
    "TotalElapsed": 0.0,
    "TotalSaved": 0.0,
}


# ============================================================
# Projector UART target heartbeat
#
# send_target()은 target을 즉시 한 번 송신하고,
# background thread가 다음 target 변경 전까지 같은 Txxx를
# UART_TARGET_HEARTBEAT_INTERVAL마다 계속 재전송한다.
#
# ATmega는 같은 target 재전송을 하트비트로 간주해
# 제어 상태를 리셋하지 않는다. (HB 카운터로 확인 가능)
# ============================================================
_uart_write_lock = threading.Lock()
_target_lock = threading.Lock()
_uart_heartbeat_stop = threading.Event()
_uart_heartbeat_thread = None
_current_projector_target = None


def _write_target_once(adc):
    command = f"T{int(round(adc))}\n"
    with _uart_write_lock:
        ser.write(command.encode())
        ser.flush()


def send_target(adc):
    """
    새 projector target을 설정한다.

    1) shared current target 갱신
    2) target 변경 직후 즉시 한 번 송신
    3) heartbeat thread가 이후 같은 target을 계속 재전송
    """
    global _current_projector_target

    target = int(round(adc))

    with _target_lock:
        _current_projector_target = target

    # target 변경 순간에는 heartbeat 주기를 기다리지 않고 즉시 전달
    _write_target_once(target)

    print(
        f"Projector Target -> {target} "
        f"(UART heartbeat {UART_TARGET_HEARTBEAT_INTERVAL:.2f}s)"
    )


def _uart_target_heartbeat_worker():
    while not _uart_heartbeat_stop.is_set():

        with _target_lock:
            target = _current_projector_target

        if (
            UART_TARGET_HEARTBEAT_ENABLED
            and target is not None
        ):
            try:
                _write_target_once(target)
            except Exception as e:
                # 메인 측정은 유지하되 UART 오류는 터미널에 남긴다.
                print(f"UART heartbeat warning: {e}")

        _uart_heartbeat_stop.wait(
            UART_TARGET_HEARTBEAT_INTERVAL
        )


def start_uart_target_heartbeat():
    global _uart_heartbeat_thread

    if not UART_TARGET_HEARTBEAT_ENABLED:
        print("UART target heartbeat = OFF")
        return

    if (
        _uart_heartbeat_thread is not None
        and _uart_heartbeat_thread.is_alive()
    ):
        return

    _uart_heartbeat_stop.clear()

    _uart_heartbeat_thread = threading.Thread(
        target=_uart_target_heartbeat_worker,
        name="projector-uart-heartbeat",
        daemon=True,
    )
    _uart_heartbeat_thread.start()

    print(
        f"UART target heartbeat = ON "
        f"({1.0 / UART_TARGET_HEARTBEAT_INTERVAL:.1f} Hz)"
    )


def stop_uart_target_heartbeat():
    global _uart_heartbeat_thread

    _uart_heartbeat_stop.set()

    if (
        _uart_heartbeat_thread is not None
        and _uart_heartbeat_thread.is_alive()
    ):
        _uart_heartbeat_thread.join(timeout=1.0)

    _uart_heartbeat_thread = None

# ============================================================
# ATmega ADC 범위 측정 요청
#
# 'M' 전송 -> ATmega 가 PWM 0 / 350 을 찍고
#             "RANGE:min:max" 를 UART1 로 회신한다.
#
# [규칙 2] Negative Gain
#   PWM 0   -> ADC 최대
#   PWM 350 -> ADC 최소
# ============================================================
def request_adc_range():
    if not RANGE_REQUEST_ENABLED:
        print("범위 측정 생략 -> 폴백값 사용")
        return FALLBACK_ADC_MIN, FALLBACK_ADC_MAX, False

    print()
    print("============================================")
    print(" ADC RANGE MEASURE (ATmega 'M')")
    print("============================================")
    print("PWM 0 / 350 을 찍습니다. 렌즈가 크게 움직입니다.")

    ser.reset_input_buffer()

    with _uart_write_lock:
        ser.write(b"M\n")
        ser.flush()

    deadline = time.time() + RANGE_REQUEST_TIMEOUT

    while time.time() < deadline:
        try:
            line = ser.readline().decode(
                errors="ignore"
            ).strip()
        except Exception as e:
            print(f"UART read warning: {e}")
            line = ""

        if line:
            print("  rx:", line)

            if line.startswith("RANGE:"):
                parts = line.split(":")

                if len(parts) >= 3:
                    try:
                        adc_min = int(parts[1])
                        adc_max = int(parts[2])
                    except ValueError:
                        continue

                    if adc_max - adc_min >= 10:
                        print()
                        print(f"  ADC min = {adc_min}  (PWM 350)")
                        print(f"  ADC max = {adc_max}  (PWM 0)")
                        print(f"  travel  = {adc_max - adc_min} counts")
                        return adc_min, adc_max, True

                    print(
                        f"  범위가 너무 좁습니다 "
                        f"({adc_max - adc_min} counts) -> 폴백"
                    )
                    break

        # 대기 중에도 화면은 유지
        try:
            frame = picam2.capture_array()
            show_dashboard(
                frame=frame,
                phase="ADC RANGE MEASURE",
                progress_text=(
                    f"waiting RANGE ... "
                    f"{deadline - time.time():.1f}s"
                ),
            )
        except KeyboardInterrupt:
            raise
        except Exception:
            pass

    print()
    print("  RANGE 수신 실패 -> 폴백값 사용")
    print(f"  {FALLBACK_ADC_MIN} ~ {FALLBACK_ADC_MAX}")

    return FALLBACK_ADC_MIN, FALLBACK_ADC_MAX, False


def build_sweep_range(adc_min, adc_max):
    """
    실측 범위 양 끝을 잘라 스윕 구간을 만든다.

    끝단은 PWM 이 레일에 붙어 정착(ST)이 안 되므로 제외한다.
    폭이 좁으면 마진을 자동으로 줄인다.
    """
    margin = SWEEP_MARGIN

    while margin > 0:
        start = adc_min + margin
        end = adc_max - margin

        if end - start >= SWEEP_MIN_SPAN:
            return start, end, margin

        margin -= 5

    # 마진 0 이어도 좁으면 그대로 사용
    return adc_min, adc_max, 0


def trim_high_adc_tail(start, end, step, skip_steps):
    """
    현재 sweep step 기준으로 높은 ADC 쪽 마지막 skip_steps개 측정점을 제거한다.

    예:
      기존 24 points, skip=10 -> 앞쪽 14 points만 유지.

    낮은 ADC 시작점(start)은 그대로 유지한다.
    안전상 최소 SWEEP_MIN_REMAINING_STEPS개는 남긴다.
    """
    values = []
    adc = int(start)

    while adc <= int(end):
        values.append(int(adc))
        adc += int(step)

    if not values or values[-1] != int(end):
        values.append(int(end))

    original_count = len(values)

    max_skip = max(0, original_count - SWEEP_MIN_REMAINING_STEPS)
    actual_skip = min(max(0, int(skip_steps)), max_skip)

    if actual_skip > 0:
        kept = values[:-actual_skip]
    else:
        kept = values

    return int(kept[-1]), original_count, len(kept), actual_skip


def build_camera_cal_references(start, end):
    """
    카메라 렌즈 캘리브레이션 reference ADC 를 스윕 구간에서 생성한다.
    중앙 -> 1/4 -> 3/4 -> 양 끝 순서로 시도한다.
    """
    span = end - start
    center = start + span // 2

    candidates = [
        center,
        start + span // 4,
        start + (span * 3) // 4,
        start,
        end,
    ]

    result = []

    for value in candidates:
        value = int(round(value))

        if value < start:
            value = start

        if value > end:
            value = end

        if value not in result:
            result.append(value)

    return result

def extract_quadrant(gray, quadrant_name):
    x, y, w, h = QUADRANTS[quadrant_name]
    return gray[y:y + h, x:x + w]


def preprocess_focus_image(image):
    enhanced = CLAHE.apply(image)
    filtered = cv2.GaussianBlur(
        enhanced,
        GAUSSIAN_KERNEL,
        GAUSSIAN_SIGMA,
    )
    return filtered


def calculate_focus_metrics(image):
    brightness = float(image.mean())

    raw_laplacian = float(
        cv2.Laplacian(
            image,
            cv2.CV_64F,
        ).var()
    )

    enhanced = preprocess_focus_image(image)

    clahe_laplacian = float(
        cv2.Laplacian(
            enhanced,
            cv2.CV_64F,
        ).var()
    )

    gx = cv2.Sobel(
        enhanced,
        cv2.CV_64F,
        1,
        0,
        ksize=3,
    )

    gy = cv2.Sobel(
        enhanced,
        cv2.CV_64F,
        0,
        1,
        ksize=3,
    )

    tenengrad = float(
        np.mean(gx * gx + gy * gy)
    )

    return {
        "Brightness": brightness,
        "RawLaplacian": raw_laplacian,
        "ClaheLaplacian": clahe_laplacian,
        "Tenengrad": tenengrad,
    }


def median_dict(dict_list):
    if not dict_list:
        raise ValueError("median_dict: empty input")

    result = {}
    for key in dict_list[0].keys():
        result[key] = float(
            np.median([
                float(row[key])
                for row in dict_list
            ])
        )
    return result


def brightness_verdict(brightness):
    if brightness > BRIGHTNESS_SATURATED:
        return "SATURATED"
    if brightness < BRIGHTNESS_DARK:
        return "TOO DARK"
    if brightness < BRIGHTNESS_GOOD_LOW:
        return "DARK / MARGINAL"
    if brightness > BRIGHTNESS_GOOD_HIGH:
        return "BRIGHT / MARGINAL"
    return "OK"


def safe_corrcoef(a, b):
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)

    if len(a) < 3:
        return 0.0

    if np.std(a) < 1e-12 or np.std(b) < 1e-12:
        return 0.0

    value = float(np.corrcoef(a, b)[0, 1])
    if not np.isfinite(value):
        return 0.0
    return value


def robust_sigma(values):
    values = np.asarray(values, dtype=float)

    if len(values) < 2:
        return 0.0

    median_value = float(np.median(values))
    mad = float(
        np.median(
            np.abs(values - median_value)
        )
    )

    sigma_from_mad = 1.4826 * mad
    sigma_from_std = float(np.std(values, ddof=1))

    # MAD가 0으로 무너지는 경우도 있어서 std의 절반을 하한으로 사용
    return max(
        sigma_from_mad,
        0.5 * sigma_from_std,
        1e-9,
    )


# ============================================================
# Local peak analysis
# ============================================================
def quadratic_fit_at_index(x_values, y_values, peak_index):
    x = np.asarray(x_values, dtype=float)
    y = np.asarray(y_values, dtype=float)
    i = int(peak_index)

    sample_x = float(x[i])
    sample_y = float(y[i])

    if i <= 0 or i >= len(y) - 1:
        return {
            "SampleX": sample_x,
            "SampleY": sample_y,
            "RefinedX": sample_x,
            "RefinedY": sample_y,
            "FitOK": False,
        }

    x3 = x[i - 1:i + 2]
    y3 = y[i - 1:i + 2]

    try:
        a, b, c = np.polyfit(x3, y3, 2)
    except Exception:
        return {
            "SampleX": sample_x,
            "SampleY": sample_y,
            "RefinedX": sample_x,
            "RefinedY": sample_y,
            "FitOK": False,
        }

    # concave-down parabola만 peak fitting으로 인정
    if (
        not np.isfinite(a)
        or not np.isfinite(b)
        or abs(a) < 1e-12
        or a >= 0
    ):
        return {
            "SampleX": sample_x,
            "SampleY": sample_y,
            "RefinedX": sample_x,
            "RefinedY": sample_y,
            "FitOK": False,
        }

    refined_x = float(-b / (2.0 * a))

    if refined_x < float(x3[0]) or refined_x > float(x3[-1]):
        return {
            "SampleX": sample_x,
            "SampleY": sample_y,
            "RefinedX": sample_x,
            "RefinedY": sample_y,
            "FitOK": False,
        }

    refined_y = float(
        a * refined_x * refined_x
        + b * refined_x
        + c
    )

    return {
        "SampleX": sample_x,
        "SampleY": sample_y,
        "RefinedX": refined_x,
        "RefinedY": refined_y,
        "FitOK": True,
    }


def detect_local_peak_indices(y_values):
    y = np.asarray(y_values, dtype=float)
    n = len(y)

    if n == 0:
        return []

    if n == 1:
        return [0]

    candidates = []

    for i in range(1, n - 1):
        is_local_max = (
            y[i] >= y[i - 1]
            and y[i] >= y[i + 1]
            and (
                y[i] > y[i - 1]
                or y[i] > y[i + 1]
            )
        )

        if is_local_max:
            candidates.append(i)

    # Edge가 옆 point보다 큰 경우 fallback candidate로 기록한다.
    # 단 quality에서 큰 penalty를 받는다.
    if y[0] > y[1]:
        candidates.append(0)

    if y[-1] > y[-2]:
        candidates.append(n - 1)

    # 완전히 monotonic/flat이면 global max라도 fallback으로 남긴다.
    if len(candidates) == 0:
        candidates.append(int(np.argmax(y)))

    return sorted(set(candidates))


def analyze_local_peak_candidates(
    x_values,
    y_values,
    brightness_values,
    raw_sample_map=None,
    apply_brightness_penalty=True,
):
    """
    global max가 아니라 모든 local maximum을 후보로 만든다.

    후보 quality 요소:
    - RelativeProminence : 주변 local baseline 대비 얼마나 솟았는가
    - Curvature          : 바로 좌/우 대비 얼마나 peak다운가
    - HalfPromWidth      : 넓고 평평한 peak인가
    - PeakSNR            : 9-frame scatter 대비 peak prominence가 충분한가
    - BrightnessCorr     : focus score가 밝기와 같이 상승하는 가짜 peak인가
    - EdgeDistance       : sweep edge에 너무 붙어 있는가

    IMPORTANT:
    다른 quadrant 결과는 여기서 절대 사용하지 않는다.
    """
    x = np.asarray(x_values, dtype=float)
    y = np.asarray(y_values, dtype=float)
    brightness = np.asarray(
        brightness_values,
        dtype=float,
    )

    n = len(x)

    if not (
        len(y) == n
        and len(brightness) == n
    ):
        raise ValueError(
            "peak analysis input length mismatch"
        )

    if n == 0:
        return []

    candidate_indices = detect_local_peak_indices(y)

    # raw sample map이 없을 때 사용할 curve-level noise estimate
    if n >= 3:
        diff_noise = robust_sigma(np.diff(y))
    else:
        diff_noise = max(float(np.std(y)), 1e-9)

    candidates = []

    for i in candidate_indices:
        left = max(0, i - PEAK_ANALYSIS_RADIUS)
        right = min(
            n,
            i + PEAK_ANALYSIS_RADIUS + 1,
        )

        left_values = y[left:i]
        right_values = y[i + 1:right]

        if len(left_values) > 0 and len(right_values) > 0:
            # 양쪽 valley 중 더 높은 쪽을 baseline으로 사용.
            # 한쪽만 내려갔다가 다른 쪽이 높은 broad trend면 prominence가 작아진다.
            baseline = max(
                float(np.min(left_values)),
                float(np.min(right_values)),
            )
        elif len(left_values) > 0:
            baseline = float(np.min(left_values))
        elif len(right_values) > 0:
            baseline = float(np.min(right_values))
        else:
            baseline = float(y[i])

        peak_value = float(y[i])
        absolute_prominence = max(
            0.0,
            peak_value - baseline,
        )

        relative_prominence = (
            absolute_prominence
            / max(abs(baseline), 1e-9)
        )

        # 바로 좌/우에서 얼마나 꺾이는지
        if 0 < i < n - 1:
            curvature = max(
                0.0,
                (
                    2.0 * peak_value
                    - float(y[i - 1])
                    - float(y[i + 1])
                )
                / max(abs(peak_value), 1e-9),
            )
        else:
            curvature = 0.0

        # half-prominence width
        half_level = (
            baseline
            + 0.5 * absolute_prominence
        )

        width_left = i
        while (
            width_left - 1 >= 0
            and y[width_left - 1] >= half_level
        ):
            width_left -= 1

        width_right = i
        while (
            width_right + 1 < n
            and y[width_right + 1] >= half_level
        ):
            width_right += 1

        half_prom_width_points = (
            width_right - width_left + 1
        )

        # candidate 주변 focus-vs-brightness correlation
        local_corr = safe_corrcoef(
            y[left:right],
            brightness[left:right],
        )

        # 9-frame raw scatter를 알고 있으면 실제 measurement noise 사용
        noise_sigma = diff_noise

        if raw_sample_map is not None:
            sample_key = float(x[i])

            if sample_key in raw_sample_map:
                noise_sigma = robust_sigma(
                    raw_sample_map[sample_key]
                )

        peak_snr = (
            absolute_prominence
            / max(noise_sigma, 1e-9)
        )

        # ----------------------------------------------------
        # 0~1 shape score
        # ----------------------------------------------------
        prominence_score = min(
            1.0,
            relative_prominence
            / PEAK_REL_PROMINENCE_GOOD,
        )

        curvature_score = min(
            1.0,
            curvature
            / PEAK_CURVATURE_GOOD,
        )

        width_score = 1.0 / (
            1.0
            + PEAK_WIDTH_PENALTY_PER_EXTRA_POINT
            * max(
                half_prom_width_points - 2,
                0,
            )
        )

        snr_score = min(
            1.0,
            peak_snr / PEAK_SNR_GOOD,
        )

        shape_score = (
            0.50 * prominence_score
            + 0.30 * curvature_score
            + 0.10 * width_score
            + 0.10 * snr_score
        )

        # ----------------------------------------------------
        # Brightness correlation penalty
        # positive correlation만 penalty.
        # 실제 sharp focus peak가 brightness valley와 겹치는 negative corr는
        # 억지로 감점하지 않는다.
        # ----------------------------------------------------
        brightness_penalty_factor = 1.0

        if apply_brightness_penalty:
            positive_corr = max(
                0.0,
                local_corr,
            )

            if positive_corr > PEAK_BRIGHTNESS_CORR_START:
                normalized_corr = (
                    positive_corr
                    - PEAK_BRIGHTNESS_CORR_START
                ) / (
                    1.0
                    - PEAK_BRIGHTNESS_CORR_START
                )

                normalized_corr = float(
                    np.clip(
                        normalized_corr,
                        0.0,
                        1.0,
                    )
                )

                brightness_penalty_factor = (
                    1.0
                    - PEAK_BRIGHTNESS_CORR_MAX_PENALTY
                    * normalized_corr
                )

        # ----------------------------------------------------
        # Edge penalty
        # ----------------------------------------------------
        edge_distance = min(
            i,
            n - 1 - i,
        )

        if edge_distance == 0:
            edge_penalty_factor = (
                PEAK_EDGE_PENALTY_EDGE
            )
        elif edge_distance == 1:
            edge_penalty_factor = (
                PEAK_EDGE_PENALTY_DISTANCE1
            )
        elif edge_distance == 2:
            edge_penalty_factor = (
                PEAK_EDGE_PENALTY_DISTANCE2
            )
        else:
            edge_penalty_factor = 1.0

        quality = (
            shape_score
            * brightness_penalty_factor
            * edge_penalty_factor
        )

        fit = quadratic_fit_at_index(
            x,
            y,
            i,
        )

        candidates.append({
            "Index": int(i),
            "SampleX": fit["SampleX"],
            "SampleScore": fit["SampleY"],
            "RefinedX": fit["RefinedX"],
            "RefinedScore": fit["RefinedY"],
            "FitOK": fit["FitOK"],
            "Baseline": baseline,
            "AbsoluteProminence": absolute_prominence,
            "RelativeProminence": relative_prominence,
            "Curvature": curvature,
            "HalfPromWidthPoints": int(
                half_prom_width_points
            ),
            "LocalBrightnessCorr": local_corr,
            "NoiseSigma": noise_sigma,
            "PeakSNR": peak_snr,
            "ShapeScore": shape_score,
            "BrightnessPenaltyFactor": (
                brightness_penalty_factor
            ),
            "EdgeDistance": int(edge_distance),
            "EdgePenaltyFactor": (
                edge_penalty_factor
            ),
            "Quality": float(quality),
            "PeakBrightness": float(
                brightness[i]
            ),
            "IsEdge": bool(edge_distance == 0),
        })

    candidates.sort(
        key=lambda row: row["Quality"],
        reverse=True,
    )

    return candidates


def summarize_best_peak(candidates):
    if not candidates:
        raise ValueError("No peak candidate")

    best = dict(candidates[0])

    if len(candidates) >= 2:
        second_quality = float(
            candidates[1]["Quality"]
        )
    else:
        second_quality = 0.0

    quality_margin = max(
        0.0,
        float(best["Quality"])
        - second_quality,
    )

    margin_score = float(
        np.clip(
            quality_margin
            / max(PEAK_AMBIGUOUS_MARGIN, 1e-9),
            0.0,
            1.0,
        )
    )

    confidence = (
        0.75 * float(best["Quality"])
        + 0.25 * margin_score
    )

    warnings = []

    if best["IsEdge"]:
        warnings.append("EDGE_PEAK")

    if best["Quality"] < PEAK_LOW_QUALITY_THRESHOLD:
        warnings.append("LOW_PEAK_QUALITY")

    if quality_margin < PEAK_AMBIGUOUS_MARGIN:
        warnings.append("AMBIGUOUS_PEAKS")

    if best["PeakBrightness"] < BRIGHTNESS_DARK:
        warnings.append("DARK_ROI")

    if best["PeakBrightness"] > BRIGHTNESS_SATURATED:
        warnings.append("SATURATED_ROI")

    if best["LocalBrightnessCorr"] > 0.60:
        warnings.append("BRIGHTNESS_COUPLED")

    if best["HalfPromWidthPoints"] > 4:
        warnings.append("WIDE_PEAK")

    if confidence >= 0.75:
        confidence_label = "HIGH"
    elif confidence >= 0.50:
        confidence_label = "MEDIUM"
    else:
        confidence_label = "LOW"

    best["SecondQuality"] = second_quality
    best["QualityMargin"] = quality_margin
    best["Confidence"] = confidence
    best["ConfidenceLabel"] = confidence_label
    best["Warnings"] = warnings

    return best


# ============================================================
# Dashboard
# ============================================================
def make_dashboard(
    frame,
    active_quadrant=None,
    current_adc=None,
    lens_position=None,
    metrics=None,
    phase="",
    progress_text="",
):
    canvas = np.zeros(
        (720, 1280, 3),
        dtype=np.uint8,
    )

    display = cv2.cvtColor(
        frame,
        cv2.COLOR_RGB2BGR,
    )

    cv2.rectangle(
        display,
        (ROI_X1, ROI_Y1),
        (ROI_X2, ROI_Y2),
        (0, 255, 0),
        3,
    )

    cv2.line(
        display,
        (ROI_CENTER_X, ROI_Y1),
        (ROI_CENTER_X, ROI_Y2),
        (0, 255, 255),
        2,
    )

    cv2.line(
        display,
        (ROI_X1, ROI_CENTER_Y),
        (ROI_X2, ROI_CENTER_Y),
        (0, 255, 255),
        2,
    )

    for q, rect in QUADRANTS.items():
        x, y, w, h = rect

        if q == active_quadrant:
            cv2.rectangle(
                display,
                (x, y),
                (x + w, y + h),
                (255, 0, 255),
                5,
            )

        cv2.putText(
            display,
            q,
            (x + 12, y + 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
        )

    display_small = cv2.resize(
        display,
        (854, 480),
    )
    canvas[0:480, 0:854] = display_small

    gray = cv2.cvtColor(
        frame,
        cv2.COLOR_RGB2GRAY,
    )

    if active_quadrant is not None:
        q_image = extract_quadrant(
            gray,
            active_quadrant,
        )
    else:
        q_image = gray[
            ROI_Y1:ROI_Y2,
            ROI_X1:ROI_X2,
        ]

    raw_view = cv2.cvtColor(
        q_image,
        cv2.COLOR_GRAY2BGR,
    )
    raw_view = cv2.resize(
        raw_view,
        (426, 240),
    )
    canvas[0:240, 854:1280] = raw_view

    enhanced = preprocess_focus_image(q_image)
    enhanced_view = cv2.cvtColor(
        enhanced,
        cv2.COLOR_GRAY2BGR,
    )
    enhanced_view = cv2.resize(
        enhanced_view,
        (426, 240),
    )
    canvas[240:480, 854:1280] = enhanced_view

    cv2.putText(
        canvas,
        "RAW ROI",
        (870, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 0),
        2,
    )

    cv2.putText(
        canvas,
        "CLAHE + GAUSSIAN",
        (870, 268),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (0, 255, 255),
        2,
    )

    cv2.rectangle(
        canvas,
        (0, 480),
        (1280, 720),
        (35, 35, 35),
        -1,
    )

    cv2.putText(
        canvas,
        f"PHASE : {phase}",
        (25, 518),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.66,
        (255, 255, 255),
        2,
    )

    if active_quadrant is not None:
        cv2.putText(
            canvas,
            f"ACTIVE : {active_quadrant}",
            (25, 552),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (255, 255, 255),
            2,
        )

    if current_adc is not None:
        cv2.putText(
            canvas,
            f"PROJECTOR ADC : {current_adc}",
            (240, 552),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (255, 255, 255),
            2,
        )

    if lens_position is not None:
        cv2.putText(
            canvas,
            f"CAMERA LENS : {lens_position:.4f}",
            (560, 552),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (255, 255, 255),
            2,
        )

    if metrics is not None:
        cv2.putText(
            canvas,
            f"B={metrics['Brightness']:.1f}  "
            f"RawLap={metrics['RawLaplacian']:.1f}",
            (25, 596),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (0, 255, 0),
            2,
        )

        cv2.putText(
            canvas,
            f"ClaheLap={metrics['ClaheLaplacian']:.1f}  "
            f"Ten={metrics['Tenengrad']:.1f}",
            (520, 596),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (0, 255, 255),
            2,
        )

    if progress_text:
        cv2.putText(
            canvas,
            progress_text,
            (25, 680),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (255, 255, 0),
            2,
        )

    cv2.putText(
        canvas,
        "Q = ABORT",
        (1090, 700),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.50,
        (255, 255, 255),
        1,
    )

    return canvas


def show_dashboard(
    frame,
    active_quadrant=None,
    current_adc=None,
    lens_position=None,
    metrics=None,
    phase="",
    progress_text="",
):
    dashboard = make_dashboard(
        frame=frame,
        active_quadrant=active_quadrant,
        current_adc=current_adc,
        lens_position=lens_position,
        metrics=metrics,
        phase=phase,
        progress_text=progress_text,
    )

    cv2.imshow(
        WINDOW_NAME,
        dashboard,
    )

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        raise KeyboardInterrupt

    return key


def initialize_debug_window():
    cv2.namedWindow(
        WINDOW_NAME,
        cv2.WINDOW_AUTOSIZE,
    )

    frame = picam2.capture_array()
    dashboard = make_dashboard(
        frame,
        phase="INITIALIZING",
    )

    cv2.imshow(
        WINDOW_NAME,
        dashboard,
    )
    cv2.waitKey(100)

    session_type = os.environ.get(
        "XDG_SESSION_TYPE",
        "",
    ).lower()

    print(f"Desktop Session = {session_type}")

    if (
        MOVE_TO_SECOND_MONITOR
        and session_type != "wayland"
    ):
        try:
            cv2.moveWindow(
                WINDOW_NAME,
                SECOND_MONITOR_X,
                SECOND_MONITOR_Y,
            )
            cv2.waitKey(100)
            print("Debug Window 보조모니터 이동 시도")
        except Exception as e:
            print("Window 이동 실패:", e)

    elif session_type == "wayland":
        print("Wayland: 창을 직접 보조모니터로 옮겨주세요.")


def wait_for_user_start():
    print()
    print("============================================")
    print(" DEBUG WINDOW 확인")
    print("============================================")
    print("S = START / Q = ABORT")

    while True:
        frame = picam2.capture_array()
        dashboard = make_dashboard(
            frame,
            phase="READY - PRESS S TO START",
            progress_text="V4.2: ST settle signal enabled",
        )

        cv2.putText(
            dashboard,
            "PRESS S TO START",
            (330, 350),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.3,
            (0, 255, 255),
            3,
        )

        cv2.imshow(
            WINDOW_NAME,
            dashboard,
        )

        key = cv2.waitKey(30) & 0xFF

        if key == ord("s"):
            print("측정을 시작합니다.")
            return

        if key == ord("q"):
            raise KeyboardInterrupt


def live_wait(
    seconds,
    quadrant_name=None,
    current_adc=None,
    lens_position=None,
    phase="WAITING",
    progress_text="",
):
    start = time.time()

    while time.time() - start < seconds:
        frame = picam2.capture_array()

        show_dashboard(
            frame=frame,
            active_quadrant=quadrant_name,
            current_adc=current_adc,
            lens_position=lens_position,
            phase=phase,
            progress_text=progress_text,
        )

        time.sleep(0.03)


# ============================================================
# ST 정착 신호 폴링
#
# 엣지 인터럽트가 아니라 레벨 폴링을 쓰는 이유:
#   ATmega가 Pi의 대기 준비보다 먼저 정착하면 엣지를 놓쳐
#   오히려 매번 타임아웃까지 기다리게 된다.
#   레벨은 늦게 읽어도 HIGH가 유지되므로 이 문제가 없다.
# ============================================================
def read_st_pin():
    if _st_handle is None:
        return None

    try:
        return bool(
            lgpio.gpio_read(_st_handle, ST_GPIO_PIN)
        )
    except Exception:
        return None



def query_projector_position():
    """
    ATmega Q -> POS:<adc>:<st> 를 짧게 조회한다.
    heartbeat thread와 UART TX가 섞이지 않도록 같은 lock을 사용한다.
    """
    if not SOFT_SETTLE_ENABLED:
        return None, None

    old_timeout = ser.timeout

    try:
        with _uart_write_lock:
            ser.timeout = SOFT_SETTLE_QUERY_TIMEOUT
            ser.write(b"Q\n")
            ser.flush()

            deadline = time.time() + SOFT_SETTLE_QUERY_TIMEOUT

            while time.time() < deadline:
                raw = ser.readline()
                if not raw:
                    continue

                line = raw.decode(errors="ignore").strip()

                if not line.startswith("POS:"):
                    # STABLE:, UNREACH: 등 비동기 메시지는 건너뛴다.
                    continue

                parts = line.split(":")
                if len(parts) < 3:
                    continue

                try:
                    position = int(parts[1])
                    st = bool(int(parts[2]))
                except ValueError:
                    continue

                return position, st

    except Exception:
        return None, None

    finally:
        ser.timeout = old_timeout

    return None, None


def settle_projector(
    adc,
    quadrant_name=None,
    phase="PROJECTOR VCM SETTLING",
    progress_text="",
):
    """
    ST 기반 projector VCM 정착 대기.

    고정 3초로 넘어가지 않는다. ST=1 이 될 때까지 기다린다.
    ST 없이 촬영하면 흔들리는 렌즈를 찍게 되므로
    타임아웃 시에는 타겟을 재전송하고 한 번 더 시도한다.

    반환: (settled, elapsed)
    """
    # ST 미사용 / 초기화 실패 -> 기존 고정 대기로 폴백
    if _st_handle is None:
        live_wait(
            PROJECTOR_SETTLE_TIME,
            quadrant_name=quadrant_name,
            current_adc=adc,
            phase=phase + " (FIXED)",
            progress_text=progress_text,
        )

        _st_stats["Waits"] += 1
        _st_stats["TotalElapsed"] += PROJECTOR_SETTLE_TIME

        return False, PROJECTOR_SETTLE_TIME

    start = time.time()
    settled = False

    for attempt in range(ST_RETRY_COUNT + 1):

        if attempt > 0:
            print(
                f"  ST 재시도 {attempt}/{ST_RETRY_COUNT} "
                f"-> T{adc} 재전송"
            )
            # 같은 값이면 ATmega 가 하트비트로 무시하므로
            # R 로 상태를 털고 다시 보낸다.
            with _uart_write_lock:
                ser.write(b"R\n")
                ser.flush()
            time.sleep(0.05)
            send_target(adc)

        live_wait(
            ST_COMMAND_GUARD_TIME,
            quadrant_name=quadrant_name,
            current_adc=adc,
            phase=phase + " - GUARD",
            progress_text=progress_text,
        )

        live_wait(
            ST_MIN_SETTLE_TIME,
            quadrant_name=quadrant_name,
            current_adc=adc,
            phase=phase + " - MIN FLOOR",
            progress_text=progress_text,
        )

        deadline = time.time() + ST_TIMEOUT
        soft_in_band_since = None
        last_soft_query = 0.0
        last_position = None
        settle_reason = ""

        while time.time() < deadline:
            # 1) Hardware ST is still the first choice.
            if read_st_pin():
                settled = True
                settle_reason = "GPIO ST"
                break

            now = time.time()

            # 2) Fallback: actual Hall position remains target±2 for 150 ms.
            if (
                SOFT_SETTLE_ENABLED
                and now - last_soft_query >= SOFT_SETTLE_QUERY_INTERVAL
            ):
                last_soft_query = now
                position, q_st = query_projector_position()

                if position is not None:
                    last_position = position

                    # UART Q itself may already report ST=1 before GPIO read catches it.
                    if q_st:
                        settled = True
                        settle_reason = "UART ST"
                        break

                    if abs(position - int(round(adc))) <= SOFT_SETTLE_ERROR_COUNTS:
                        if soft_in_band_since is None:
                            soft_in_band_since = now
                        elif now - soft_in_band_since >= SOFT_SETTLE_HOLD_TIME:
                            settled = True
                            settle_reason = (
                                f"ADC hold ±{SOFT_SETTLE_ERROR_COUNTS} "
                                f"for {SOFT_SETTLE_HOLD_TIME:.2f}s"
                            )
                            break
                    else:
                        soft_in_band_since = None

            frame = picam2.capture_array()

            if soft_in_band_since is None:
                soft_text = "ADC hold: waiting"
            else:
                soft_text = (
                    f"ADC hold: {now - soft_in_band_since:.2f}/"
                    f"{SOFT_SETTLE_HOLD_TIME:.2f}s"
                )

            position_text = (
                "?" if last_position is None else str(last_position)
            )

            show_dashboard(
                frame=frame,
                active_quadrant=quadrant_name,
                current_adc=adc,
                phase=phase + " - WAIT ST",
                progress_text=(
                    f"{progress_text} | ST=LOW | POS={position_text} | "
                    f"{soft_text} | {deadline - time.time():.1f}s left"
                ),
            )

            time.sleep(ST_POLL_INTERVAL)

        if settled:
            if settle_reason:
                print(f"  SETTLED by {settle_reason}")
            break

    elapsed = time.time() - start
    saved = max(0.0, PROJECTOR_SETTLE_TIME - elapsed)

    _st_stats["Waits"] += 1
    _st_stats["TotalElapsed"] += elapsed
    _st_stats["TotalSaved"] += saved

    if settled:
        _st_stats["EarlyExits"] += 1
        print(
            f"  ST HIGH -> 정착 확인 | {elapsed:.2f}s"
        )
    else:
        _st_stats["Timeouts"] += 1
        print(
            f"  ST TIMEOUT | {elapsed:.2f}s "
            f"-> 이 ADC 데이터는 신뢰할 수 없음"
        )

    return settled, elapsed


def print_st_summary():
    if _st_stats["Waits"] == 0:
        return

    print()
    print("============================================")
    print(" ST SETTLE SIGNAL SUMMARY")
    print("============================================")
    print(f"Total waits    = {_st_stats['Waits']}")
    print(f"Early exits    = {_st_stats['EarlyExits']}")
    print(f"Timeouts       = {_st_stats['Timeouts']}")
    print(
        f"Average wait   = "
        f"{_st_stats['TotalElapsed'] / _st_stats['Waits']:.2f}s "
        f"(fixed would be {PROJECTOR_SETTLE_TIME:.2f}s)"
    )
    print(
        f"Total saved    = "
        f"{_st_stats['TotalSaved']:.1f}s"
    )


# ============================================================
# Exposure lock
# ============================================================
def converge_and_lock_exposure():
    print()
    print("============================================")
    print(" EXPOSURE / AWB LOCK")
    print("============================================")

    # 지원 여부를 확인하고 가능한 control만 사용
    controls_to_enable = {}

    if "AeEnable" in picam2.camera_controls:
        controls_to_enable["AeEnable"] = True

    if "AwbEnable" in picam2.camera_controls:
        controls_to_enable["AwbEnable"] = True

    if controls_to_enable:
        picam2.set_controls(controls_to_enable)

    time.sleep(AE_CONVERGE_TIME)

    metadata = picam2.capture_metadata()

    exposure_time = metadata.get(
        "ExposureTime",
        None,
    )
    analogue_gain = metadata.get(
        "AnalogueGain",
        None,
    )
    colour_gains = metadata.get(
        "ColourGains",
        None,
    )

    lock_controls = {}

    if "AeEnable" in picam2.camera_controls:
        lock_controls["AeEnable"] = False

    if "AwbEnable" in picam2.camera_controls:
        lock_controls["AwbEnable"] = False

    if (
        exposure_time is not None
        and "ExposureTime" in picam2.camera_controls
    ):
        lock_controls["ExposureTime"] = int(
            exposure_time
        )

    if (
        analogue_gain is not None
        and "AnalogueGain" in picam2.camera_controls
    ):
        lock_controls["AnalogueGain"] = float(
            analogue_gain
        )

    if (
        colour_gains is not None
        and "ColourGains" in picam2.camera_controls
    ):
        lock_controls["ColourGains"] = (
            float(colour_gains[0]),
            float(colour_gains[1]),
        )

    if lock_controls:
        picam2.set_controls(lock_controls)

    time.sleep(EXPOSURE_LOCK_SETTLE_TIME)

    print(f"ExposureTime = {exposure_time}")
    print(f"AnalogueGain = {analogue_gain}")
    print(f"ColourGains  = {colour_gains}")
    print("AE/AWB LOCK 완료")

    return {
        "ExposureTime": exposure_time,
        "AnalogueGain": analogue_gain,
        "ColourGains": colour_gains,
    }


def brightness_check(current_adc):
    print()
    print("============================================")
    print(" BRIGHTNESS CHECK")
    print("============================================")

    collected = {
        q: [] for q in QUADRANT_ORDER
    }

    for _ in range(7):
        frame = picam2.capture_array()
        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_RGB2GRAY,
        )

        for q in QUADRANT_ORDER:
            q_image = extract_quadrant(gray, q)
            collected[q].append(
                float(q_image.mean())
            )

        show_dashboard(
            frame=frame,
            current_adc=current_adc,
            phase="BRIGHTNESS CHECK",
        )

        time.sleep(0.04)

    result = {}

    for q in QUADRANT_ORDER:
        value = float(np.median(collected[q]))
        result[q] = value
        print(
            f"{q}: brightness={value:6.1f}  "
            f"{brightness_verdict(value)}"
        )

    print(
        "※ 어두운 Q도 중단하지 않음. "
        "projector peak confidence에 DARK warning만 기록."
    )

    return result


# ============================================================
# Camera lens movement / stability
# ============================================================
def set_camera_lens_direct(
    target_position,
    quadrant_name=None,
    current_adc=None,
    phase="CAMERA LENS MOVE",
    use_park=True,
):
    target_position = float(
        np.clip(
            target_position,
            CAMERA_LENS_MIN,
            CAMERA_LENS_MAX,
        )
    )

    park_position = float(
        np.clip(
            CAMERA_LENS_PARK_POSITION,
            CAMERA_LENS_MIN,
            CAMERA_LENS_MAX,
        )
    )

    if use_park:
        picam2.set_controls({
            "AfMode": controls.AfModeEnum.Manual,
            "LensPosition": park_position,
        })

        live_wait(
            CAMERA_SWITCH_PARK_SETTLE_TIME,
            quadrant_name=quadrant_name,
            current_adc=current_adc,
            lens_position=park_position,
            phase=phase + " - PARK",
            progress_text=(
                f"Park {park_position:.2f} D"
            ),
        )

    picam2.set_controls({
        "AfMode": controls.AfModeEnum.Manual,
        "LensPosition": target_position,
    })

    settle_time = (
        CAMERA_SWITCH_TARGET_SETTLE_TIME
        if use_park
        else CAMERA_LENS_CAL_SETTLE_TIME
    )

    live_wait(
        settle_time,
        quadrant_name=quadrant_name,
        current_adc=current_adc,
        lens_position=target_position,
        phase=phase,
        progress_text=(
            f"Target {target_position:.3f} D"
        ),
    )

    discard_count = (
        CAMERA_SWITCH_DISCARD_FRAMES
        if use_park
        else CAMERA_LENS_DISCARD_FRAMES
    )

    for _ in range(discard_count):
        frame = picam2.capture_array()
        show_dashboard(
            frame=frame,
            active_quadrant=quadrant_name,
            current_adc=current_adc,
            lens_position=target_position,
            phase=phase + " - DISCARD",
            progress_text="Discarding transient frames",
        )

    return target_position


def wait_for_camera_focus_stability(
    quadrant_name,
    current_adc,
    camera_lens_position,
    stability_log=None,
):
    history = []
    start_time = time.time()
    sample_index = 0
    last_rel_spread = None

    while time.time() - start_time < CAMERA_STABILITY_TIMEOUT:
        frame = picam2.capture_array()
        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_RGB2GRAY,
        )
        q_image = extract_quadrant(
            gray,
            quadrant_name,
        )
        metrics = calculate_focus_metrics(q_image)

        value = float(
            metrics[CAMERA_STABILITY_METRIC]
        )

        history.append(value)
        sample_index += 1

        if len(history) > CAMERA_STABILITY_WINDOW:
            history = history[-CAMERA_STABILITY_WINDOW:]

        stable = False
        rel_spread = None

        if len(history) == CAMERA_STABILITY_WINDOW:
            med = float(np.median(history))
            rel_spread = (
                max(history) - min(history)
            ) / max(abs(med), 1e-9)

            last_rel_spread = rel_spread
            stable = (
                rel_spread
                <= CAMERA_STABILITY_REL_SPREAD
            )

        if stability_log is not None:
            stability_log.append({
                "ADC": current_adc,
                "Quadrant": quadrant_name,
                "CameraLensPosition": camera_lens_position,
                "StabilitySampleIndex": sample_index,
                "Brightness": metrics["Brightness"],
                "RawLaplacian": metrics["RawLaplacian"],
                "ClaheLaplacian": metrics["ClaheLaplacian"],
                "Tenengrad": metrics["Tenengrad"],
                "RelativeSpread": (
                    rel_spread
                    if rel_spread is not None
                    else ""
                ),
                "Stable": stable,
            })

        if rel_spread is None:
            spread_text = (
                f"window={len(history)}/"
                f"{CAMERA_STABILITY_WINDOW}"
            )
        else:
            spread_text = (
                f"spread={rel_spread * 100:.2f}%"
            )

        show_dashboard(
            frame=frame,
            active_quadrant=quadrant_name,
            current_adc=current_adc,
            lens_position=camera_lens_position,
            metrics=metrics,
            phase="CAMERA LENS STABILITY CHECK",
            progress_text=(
                f"{CAMERA_STABILITY_METRIC}="
                f"{value:.1f} | {spread_text}"
            ),
        )

        if stable:
            elapsed = time.time() - start_time
            print(
                f"{quadrant_name} lens 안정화 완료 | "
                f"spread={rel_spread * 100:.2f}% | "
                f"{elapsed:.2f}s"
            )

            return {
                "Stable": True,
                "Elapsed": elapsed,
                "RelativeSpread": rel_spread,
                "Samples": sample_index,
            }

        time.sleep(CAMERA_STABILITY_FRAME_INTERVAL)

    elapsed = time.time() - start_time

    message = (
        f"{quadrant_name} lens 안정화 TIMEOUT | "
        f"last spread="
        f"{last_rel_spread if last_rel_spread is not None else 'N/A'}"
    )

    if CAMERA_STABILITY_ABORT_ON_TIMEOUT:
        raise RuntimeError(message)

    print("WARNING:", message)

    return {
        "Stable": False,
        "Elapsed": elapsed,
        "RelativeSpread": last_rel_spread,
        "Samples": sample_index,
    }


# ============================================================
# Sweep lists
# ============================================================
def build_adc_values():
    values = []
    adc = ADC_START

    while adc <= ADC_END:
        values.append(int(adc))
        adc += ADC_STEP

    if not values or values[-1] != ADC_END:
        values.append(int(ADC_END))

    return values


def build_lens_sweep_positions():
    values = []
    p = CAMERA_LENS_MIN

    while p <= CAMERA_LENS_MAX + 1e-9:
        values.append(round(float(p), 4))
        p += CAMERA_LENS_SWEEP_STEP

    if (
        not values
        or abs(values[-1] - CAMERA_LENS_MAX) > 1e-6
    ):
        values.append(
            round(float(CAMERA_LENS_MAX), 4)
        )

    return values


# ============================================================
# Camera lens calibration
# ============================================================
def measure_all_quadrants_at_current_lens(
    lens_position,
    reference_adc,
    progress_text,
):
    samples = {
        q: [] for q in QUADRANT_ORDER
    }

    for _ in range(CAMERA_LENS_CAL_SAMPLES):
        frame = picam2.capture_array()
        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_RGB2GRAY,
        )

        for q in QUADRANT_ORDER:
            q_image = extract_quadrant(gray, q)
            samples[q].append(
                calculate_focus_metrics(q_image)
            )

        show_dashboard(
            frame=frame,
            current_adc=reference_adc,
            lens_position=lens_position,
            phase="CAMERA LENS CALIBRATION",
            progress_text=progress_text,
        )

        time.sleep(CAMERA_LENS_CAL_FRAME_INTERVAL)

    return {
        q: median_dict(samples[q])
        for q in QUADRANT_ORDER
    }


def run_camera_lens_sweep(
    reference_adc,
    lens_positions,
    projector_already_at_reference=False,
):
    print()
    print("############################################")
    print(
        f" CAMERA LENS SWEEP | Projector ADC {reference_adc}"
    )
    print("############################################")

    if not projector_already_at_reference:
        send_target(reference_adc)

        settle_projector(
            adc=reference_adc,
            phase="PROJECTOR SETTLING FOR LENS CAL",
            progress_text=f"Reference ADC {reference_adc}",
        )
    else:
        print(
            "Projector가 이미 첫 reference ADC에 있으므로 "
            "중복 VCM 이동/settle을 생략합니다."
        )

    picam2.set_controls({
        "AfMode": controls.AfModeEnum.Manual,
        "LensPosition": float(CAMERA_LENS_MIN),
    })
    time.sleep(CAMERA_LENS_CAL_SETTLE_TIME)

    run_data = {
        q: [] for q in QUADRANT_ORDER
    }

    for index, lens_position in enumerate(
        lens_positions,
        start=1,
    ):
        set_camera_lens_direct(
            target_position=lens_position,
            current_adc=reference_adc,
            phase="CAMERA LENS CALIBRATION",
            use_park=False,
        )

        medians = measure_all_quadrants_at_current_lens(
            lens_position=lens_position,
            reference_adc=reference_adc,
            progress_text=(
                f"Lens {index}/{len(lens_positions)} | "
                f"{lens_position:.2f} D"
            ),
        )

        for q in QUADRANT_ORDER:
            run_data[q].append({
                "ReferenceADC": reference_adc,
                "LensPosition": lens_position,
                **medians[q],
            })

        print(
            f"Lens {lens_position:5.2f} | "
            + " | ".join(
                f"{q} Raw={medians[q]['RawLaplacian']:7.1f} "
                f"CL={medians[q]['ClaheLaplacian']:6.1f} "
                f"T={medians[q]['Tenengrad']:8.1f}"
                for q in QUADRANT_ORDER
            )
        )

    return run_data


def analyze_camera_lens_run(run_data):
    analysis = {}

    for q in QUADRANT_ORDER:
        x = [
            row["LensPosition"]
            for row in run_data[q]
        ]
        brightness = [
            row["Brightness"]
            for row in run_data[q]
        ]

        metric_results = {}

        for metric in CAMERA_CAL_METRICS:
            y = [
                row[metric]
                for row in run_data[q]
            ]

            candidates = analyze_local_peak_candidates(
                x_values=x,
                y_values=y,
                brightness_values=brightness,
                raw_sample_map=None,
                # Camera lens sweep에서는 projector ADC가 고정되어 있으므로
                # brightness correlation을 projector sweep처럼 강하게 감점하지 않는다.
                apply_brightness_penalty=False,
            )

            best = summarize_best_peak(candidates)

            metric_results[metric] = {
                "Candidates": candidates,
                "Best": best,
            }

        # Q마다 metric도 독립 선택
        selected_metric = max(
            CAMERA_CAL_METRICS,
            key=lambda metric: (
                metric_results[metric]["Best"]["Quality"]
            ),
        )

        selected_best = metric_results[
            selected_metric
        ]["Best"]

        analysis[q] = {
            "Metric": selected_metric,
            "LensPosition": selected_best["RefinedX"],
            "SampleLensPosition": selected_best["SampleX"],
            "SampleScore": selected_best["SampleScore"],
            "RefinedScore": selected_best["RefinedScore"],
            "Quality": selected_best["Quality"],
            "Confidence": selected_best["Confidence"],
            "ConfidenceLabel": selected_best["ConfidenceLabel"],
            "QualityMargin": selected_best["QualityMargin"],
            "RelativeProminence": selected_best[
                "RelativeProminence"
            ],
            "Curvature": selected_best["Curvature"],
            "HalfPromWidthPoints": selected_best[
                "HalfPromWidthPoints"
            ],
            "EdgePeak": selected_best["IsEdge"],
            "FitOK": selected_best["FitOK"],
            "PeakBrightness": selected_best["PeakBrightness"],
            "Warnings": selected_best["Warnings"],
            "MetricResults": metric_results,
        }

    return analysis


def calibration_run_is_good(analysis):
    for q in QUADRANT_ORDER:
        item = analysis[q]

        if item["EdgePeak"]:
            return False

        if item["Quality"] < CAMERA_CAL_QUALITY_MIN:
            return False

    return True


def calibrate_camera_lens_once(
    first_reference_already_set=False,
):
    lens_positions = build_lens_sweep_positions()

    print()
    print("============================================")
    print(" PHASE 1 : INDEPENDENT CAMERA LENS CAL")
    print("============================================")
    print(
        f"Lens sweep {lens_positions[0]:.2f} ~ "
        f"{lens_positions[-1]:.2f} D, "
        f"step {CAMERA_LENS_SWEEP_STEP:.2f}"
    )
    print(
        "각 Q는 Raw/CLAHE/Tenengrad 중 "
        "자기 curve quality가 가장 좋은 metric을 독립 선택"
    )

    all_runs = []

    for run_index, reference_adc in enumerate(
        CAMERA_CAL_REFERENCE_ADC_SEQUENCE
    ):
        run_data = run_camera_lens_sweep(
            reference_adc=reference_adc,
            lens_positions=lens_positions,
            projector_already_at_reference=(
                first_reference_already_set
                and run_index == 0
            ),
        )

        analysis = analyze_camera_lens_run(
            run_data
        )

        all_runs.append({
            "ReferenceADC": reference_adc,
            "Data": run_data,
            "Analysis": analysis,
        })

        print()
        print("--------------------------------------------")
        print(
            f"LENS CAL RESULT @ ADC {reference_adc}"
        )
        print("--------------------------------------------")

        for q in QUADRANT_ORDER:
            item = analysis[q]

            warnings = (
                ",".join(item["Warnings"])
                if item["Warnings"]
                else "OK"
            )

            print(
                f"{q}: metric={item['Metric']:16s} | "
                f"lens={item['LensPosition']:7.3f} D | "
                f"quality={item['Quality']:.3f} | "
                f"conf={item['ConfidenceLabel']} | "
                f"B={item['PeakBrightness']:.1f} | "
                f"{warnings}"
            )

        if calibration_run_is_good(analysis):
            print()
            print(
                "모든 Q camera lens curve quality가 통과하여 "
                "추가 reference sweep을 생략합니다."
            )
            break

        print()
        print(
            "일부 Q의 camera lens peak quality가 낮아 "
            "다음 reference ADC에서 재검증합니다."
        )

    # 각 Q는 다른 Q와 무관하게 가장 quality 높은 run을 선택
    selected = {}

    for q in QUADRANT_ORDER:
        best_run = max(
            all_runs,
            key=lambda run: (
                run["Analysis"][q]["Quality"]
            ),
        )

        item = best_run["Analysis"][q]

        selected[q] = {
            "LensPosition": item["LensPosition"],
            "SampleLensPosition": item[
                "SampleLensPosition"
            ],
            "ReferenceADC": best_run["ReferenceADC"],
            "Metric": item["Metric"],
            "Quality": item["Quality"],
            "Confidence": item["Confidence"],
            "ConfidenceLabel": item[
                "ConfidenceLabel"
            ],
            "QualityMargin": item[
                "QualityMargin"
            ],
            "RelativeProminence": item[
                "RelativeProminence"
            ],
            "Curvature": item["Curvature"],
            "HalfPromWidthPoints": item[
                "HalfPromWidthPoints"
            ],
            "EdgePeak": item["EdgePeak"],
            "FitOK": item["FitOK"],
            "PeakBrightness": item[
                "PeakBrightness"
            ],
            "Warnings": item["Warnings"],
        }

    print()
    print("============================================")
    print(" SELECTED CAMERA LENS POSITION")
    print("============================================")

    for q in QUADRANT_ORDER:
        s = selected[q]
        warnings = (
            ",".join(s["Warnings"])
            if s["Warnings"]
            else "OK"
        )

        print(
            f"{q} = {s['LensPosition']:.4f} D | "
            f"metric={s['Metric']} | "
            f"refADC={s['ReferenceADC']} | "
            f"quality={s['Quality']:.3f} | "
            f"conf={s['ConfidenceLabel']} | {warnings}"
        )

    return selected, all_runs


def camera_lens_pair_consistency(selected):
    """
    최종 선택된 Camera LensPosition의 좌/우 반쪽 일관성을 검사한다.

    사용자 사분면:
        Q2 | Q1
       ---------
        Q3 | Q4

    LEFT  pair = Q2, Q3
    RIGHT pair = Q1, Q4

    통과 조건:
        abs(Q2 - Q3) < CAMERA_PAIR_MAX_DELTA
        abs(Q1 - Q4) < CAMERA_PAIR_MAX_DELTA
    """
    left_delta = abs(
        float(selected["Q2"]["LensPosition"])
        - float(selected["Q3"]["LensPosition"])
    )

    right_delta = abs(
        float(selected["Q1"]["LensPosition"])
        - float(selected["Q4"]["LensPosition"])
    )

    left_ok = left_delta < CAMERA_PAIR_MAX_DELTA
    right_ok = right_delta < CAMERA_PAIR_MAX_DELTA

    return {
        "LeftDelta": float(left_delta),
        "RightDelta": float(right_delta),
        "LeftOK": bool(left_ok),
        "RightOK": bool(right_ok),
        "OK": bool(left_ok and right_ok),
        "WorstDelta": float(max(left_delta, right_delta)),
    }


def calibrate_camera_lens(
    first_reference_already_set=False,
):
    """
    기존 Camera Lens calibration을 그대로 수행하되,
    최종 Q별 LensPosition이 같은 좌/우 반쪽에서 지나치게 벌어지면
    Camera Lens calibration 단계만 다시 수행한다.

    Projector ADC sweep(PHASE 2)은 이 함수가 통과한 뒤에만 시작한다.
    따라서 실패한 Camera Lens calibration 때문에 전체 projector sweep을
    다시 할 필요는 없다.

    무한 반복을 막기 위해 CAMERA_PAIR_MAX_ATTEMPTS까지만 시도하며,
    끝까지 0.2 D 기준을 만족하지 못하면 잘못된 값을 억지로 사용하지 않고
    RuntimeError로 중단한다.
    """
    combined_runs = []
    attempt_summaries = []

    for attempt in range(1, CAMERA_PAIR_MAX_ATTEMPTS + 1):
        print()
        print("############################################")
        print(
            f" CAMERA LENS CAL ATTEMPT "
            f"{attempt}/{CAMERA_PAIR_MAX_ATTEMPTS}"
        )
        print("############################################")

        selected, runs = calibrate_camera_lens_once(
            first_reference_already_set=(
                first_reference_already_set
                and attempt == 1
            )
        )

        # CSV에 재시도 데이터도 모두 남길 수 있도록 시도 번호를 기록한다.
        for run in runs:
            tagged_run = dict(run)
            tagged_run["PairRetryAttempt"] = attempt
            combined_runs.append(tagged_run)

        check = camera_lens_pair_consistency(selected)

        attempt_summaries.append({
            "Attempt": attempt,
            "Selected": selected,
            **check,
        })

        print()
        print("============================================")
        print(" CAMERA LENS PAIR CONSISTENCY")
        print("============================================")
        print(
            f"LEFT  Q2-Q3 = "
            f"{selected['Q2']['LensPosition']:.4f} / "
            f"{selected['Q3']['LensPosition']:.4f} D "
            f"| delta={check['LeftDelta']:.4f} D "
            f"| {'PASS' if check['LeftOK'] else 'RETRY'}"
        )
        print(
            f"RIGHT Q1-Q4 = "
            f"{selected['Q1']['LensPosition']:.4f} / "
            f"{selected['Q4']['LensPosition']:.4f} D "
            f"| delta={check['RightDelta']:.4f} D "
            f"| {'PASS' if check['RightOK'] else 'RETRY'}"
        )
        print(
            f"Required: BOTH deltas < "
            f"{CAMERA_PAIR_MAX_DELTA:.3f} D"
        )

        if check["OK"]:
            print()
            print(
                "좌/우 pair consistency 통과. "
                "이 Camera Lens calibration 결과를 사용합니다."
            )

            # 선택 결과에도 검사 정보를 남겨 이후 CSV/디버깅에서 확인 가능
            for q in QUADRANT_ORDER:
                selected[q]["PairRetryAttempt"] = attempt

            selected["Q2"]["PairDelta"] = check["LeftDelta"]
            selected["Q3"]["PairDelta"] = check["LeftDelta"]
            selected["Q1"]["PairDelta"] = check["RightDelta"]
            selected["Q4"]["PairDelta"] = check["RightDelta"]

            return selected, combined_runs

        if attempt < CAMERA_PAIR_MAX_ATTEMPTS:
            failed_pairs = []

            if not check["LeftOK"]:
                failed_pairs.append(
                    f"LEFT(Q2-Q3) {check['LeftDelta']:.3f}D"
                )

            if not check["RightOK"]:
                failed_pairs.append(
                    f"RIGHT(Q1-Q4) {check['RightDelta']:.3f}D"
                )

            print()
            print(
                "Pair consistency 실패 -> "
                "Camera Lens calibration만 다시 측정합니다."
            )
            print("Failed: " + ", ".join(failed_pairs))

            # 다음 재시도 전에 lens를 park 위치로 보내고 잠깐 안정화.
            picam2.set_controls({
                "AfMode": controls.AfModeEnum.Manual,
                "LensPosition": float(
                    np.clip(
                        CAMERA_LENS_PARK_POSITION,
                        CAMERA_LENS_MIN,
                        CAMERA_LENS_MAX,
                    )
                ),
            })
            time.sleep(0.7)

    # --------------------------------------------------------
    # 여기까지 왔으면 모든 시도가 기준 실패.
    # 잘못된 pair를 억지로 평균내거나 사용하지 않고 중단한다.
    # 가장 좋았던 시도는 오류 메시지에 표시해 진단 가능하게 한다.
    # --------------------------------------------------------
    best_attempt = min(
        attempt_summaries,
        key=lambda row: (
            row["WorstDelta"],
            row["LeftDelta"] + row["RightDelta"],
        ),
    )

    raise RuntimeError(
        "Camera Lens pair consistency가 "
        f"{CAMERA_PAIR_MAX_ATTEMPTS}회 모두 실패했습니다. "
        f"기준: Q2-Q3, Q1-Q4 각각 < "
        f"{CAMERA_PAIR_MAX_DELTA:.3f} D. "
        f"가장 일관된 시도 #{best_attempt['Attempt']}: "
        f"LEFT delta={best_attempt['LeftDelta']:.4f} D, "
        f"RIGHT delta={best_attempt['RightDelta']:.4f} D. "
        "Projector ADC sweep은 시작하지 않았습니다."
    )


# ============================================================
# Projector focus measurement
# ============================================================
def measure_projector_focus_once(
    quadrant_name,
    adc,
    camera_lens_position,
):
    frame = picam2.capture_array()
    gray = cv2.cvtColor(
        frame,
        cv2.COLOR_RGB2GRAY,
    )
    q_image = extract_quadrant(
        gray,
        quadrant_name,
    )
    metrics = calculate_focus_metrics(q_image)

    show_dashboard(
        frame=frame,
        active_quadrant=quadrant_name,
        current_adc=adc,
        lens_position=camera_lens_position,
        metrics=metrics,
        phase="PROJECTOR FOCUS MEASUREMENT",
        progress_text=(
            f"ADC {adc} | {quadrant_name} | "
            f"selection={PROJECTOR_SELECTION_METRIC}"
        ),
    )

    return metrics


def measure_projector_median(
    quadrant_name,
    adc,
    camera_lens_position,
):
    raw_samples = []

    for _ in range(PROJECTOR_SAMPLES):
        raw_samples.append(
            measure_projector_focus_once(
                quadrant_name,
                adc,
                camera_lens_position,
            )
        )
        time.sleep(PROJECTOR_FRAME_INTERVAL)

    return median_dict(raw_samples), raw_samples


# ============================================================
# Projector independent peak analysis
# ============================================================
def analyze_projector_quadrant(
    quadrant_name,
    results,
):
    x = [
        row["ADC"]
        for row in results
    ]

    brightness = [
        row["Median"]["Brightness"]
        for row in results
    ]

    metric_analysis = {}

    for metric in [
        "RawLaplacian",
        "ClaheLaplacian",
        "Tenengrad",
    ]:
        y = [
            row["Median"][metric]
            for row in results
        ]

        raw_sample_map = {}

        for row in results:
            raw_sample_map[float(row["ADC"])] = [
                sample[metric]
                for sample in row["Raw"]
            ]

        candidates = analyze_local_peak_candidates(
            x_values=x,
            y_values=y,
            brightness_values=brightness,
            raw_sample_map=raw_sample_map,
            # Projector sweep에서는 brightness coupling penalty 적용
            apply_brightness_penalty=True,
        )

        best = summarize_best_peak(candidates)

        metric_analysis[metric] = {
            "Candidates": candidates,
            "Best": best,
            "GlobalBrightnessCorr": safe_corrcoef(
                y,
                brightness,
            ),
        }

    # 최종 선택은 Raw Laplacian local peak quality
    selected = metric_analysis[
        PROJECTOR_SELECTION_METRIC
    ]["Best"]

    return {
        "Quadrant": quadrant_name,
        "MetricAnalysis": metric_analysis,
        "SelectedMetric": PROJECTOR_SELECTION_METRIC,
        "Selected": selected,
    }


def cluster_plane_candidates(projector_analysis):
    """
    OPTIONAL post-hoc 1D clustering.

    IMPORTANT:
    이 결과는 초점 선택에 사용하지 않는다.
    Q별 BEST ADC가 전부 결정된 뒤 참고용으로만 생성한다.
    """
    items = []

    for q in QUADRANT_ORDER:
        items.append({
            "Quadrant": q,
            "ADC": float(
                projector_analysis[q][
                    "Selected"
                ]["RefinedX"]
            ),
        })

    items.sort(key=lambda row: row["ADC"])

    clusters = []

    for item in items:
        assigned = False

        for cluster in clusters:
            center = float(
                np.mean([
                    member["ADC"]
                    for member in cluster
                ])
            )

            if (
                abs(item["ADC"] - center)
                <= PLANE_CLUSTER_TOLERANCE_ADC
            ):
                cluster.append(item)
                assigned = True
                break

        if not assigned:
            clusters.append([item])

    cluster_map = {}

    for cluster_index, cluster in enumerate(
        clusters,
        start=1,
    ):
        for item in cluster:
            cluster_map[item["Quadrant"]] = (
                f"PlaneCandidate{cluster_index}"
            )

    return clusters, cluster_map


# ============================================================
# CSV helpers
# ============================================================
def save_camera_calibration_csv(
    selected_camera_lens,
    all_calibration_runs,
):
    raw_file = "camera_lens_calibration_v4_2.csv"

    with open(raw_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "PairRetryAttempt",
            "ReferenceADC",
            "Quadrant",
            "LensPosition",
            "Brightness",
            "RawLaplacian",
            "ClaheLaplacian",
            "Tenengrad",
        ])

        for run in all_calibration_runs:
            for q in QUADRANT_ORDER:
                for row in run["Data"][q]:
                    writer.writerow([
                        run.get("PairRetryAttempt", 1),
                        run["ReferenceADC"],
                        q,
                        row["LensPosition"],
                        row["Brightness"],
                        row["RawLaplacian"],
                        row["ClaheLaplacian"],
                        row["Tenengrad"],
                    ])

    summary_file = (
        "camera_lens_calibration_summary_v4_2.csv"
    )

    with open(summary_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "Quadrant",
            "Position",
            "SelectedLensPosition",
            "SampleLensPosition",
            "ReferenceADC",
            "SelectedMetric",
            "Quality",
            "Confidence",
            "ConfidenceLabel",
            "QualityMargin",
            "RelativeProminence",
            "Curvature",
            "HalfPromWidthPoints",
            "PeakBrightness",
            "EdgePeak",
            "FitOK",
            "Warnings",
            "PairRetryAttempt",
            "PairDelta",
        ])

        for q in QUADRANT_ORDER:
            s = selected_camera_lens[q]
            writer.writerow([
                q,
                QUADRANT_NAMES[q],
                s["LensPosition"],
                s["SampleLensPosition"],
                s["ReferenceADC"],
                s["Metric"],
                s["Quality"],
                s["Confidence"],
                s["ConfidenceLabel"],
                s["QualityMargin"],
                s["RelativeProminence"],
                s["Curvature"],
                s["HalfPromWidthPoints"],
                s["PeakBrightness"],
                s["EdgePeak"],
                s["FitOK"],
                "|".join(s["Warnings"]),
                s.get("PairRetryAttempt", 1),
                s.get("PairDelta", ""),
            ])

    print(f"saved {raw_file}")
    print(f"saved {summary_file}")


def save_camera_stability_csv(stability_log):
    file_name = "camera_switch_stability_v4_2.csv"

    with open(file_name, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "ADC",
            "Quadrant",
            "CameraLensPosition",
            "StabilitySampleIndex",
            "Brightness",
            "RawLaplacian",
            "ClaheLaplacian",
            "Tenengrad",
            "RelativeSpread",
            "Stable",
        ])

        for row in stability_log:
            writer.writerow([
                row["ADC"],
                row["Quadrant"],
                row["CameraLensPosition"],
                row["StabilitySampleIndex"],
                row["Brightness"],
                row["RawLaplacian"],
                row["ClaheLaplacian"],
                row["Tenengrad"],
                row["RelativeSpread"],
                row["Stable"],
            ])

    print(f"saved {file_name}")


def save_projector_settle_csv(settle_log):
    """
    ST 기반 정착 대기 기록.
    스텝마다 실제 대기 시간과 조기 종료 여부를 남긴다.
    """
    file_name = "projector_settle_v4_2.csv"

    with open(file_name, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "ADC",
            "SettledByST",
            "ElapsedSeconds",
            "FixedSettleTime",
            "SavedSeconds",
        ])

        for row in settle_log:
            writer.writerow([
                row["ADC"],
                row["Settled"],
                f"{row['Elapsed']:.3f}",
                f"{PROJECTOR_SETTLE_TIME:.3f}",
                f"{max(0.0, PROJECTOR_SETTLE_TIME - row['Elapsed']):.3f}",
            ])

    print(f"saved {file_name}")


def save_projector_csvs(
    adc_values,
    quadrant_results,
    selected_camera_lens,
    projector_analysis,
    cluster_map,
):
    median_file = "projector_focus_median_v4_2.csv"

    with open(median_file, "w", newline="") as f:
        writer = csv.writer(f)
        header = ["ADC"]

        for q in QUADRANT_ORDER:
            for metric in [
                "Brightness",
                "RawLaplacian",
                "ClaheLaplacian",
                "Tenengrad",
            ]:
                header.append(f"{q}_{metric}")

        writer.writerow(header)

        for index, adc in enumerate(adc_values):
            row = [adc]

            for q in QUADRANT_ORDER:
                med = quadrant_results[q][index][
                    "Median"
                ]

                for metric in [
                    "Brightness",
                    "RawLaplacian",
                    "ClaheLaplacian",
                    "Tenengrad",
                ]:
                    row.append(med[metric])

            writer.writerow(row)

    raw_file = "projector_focus_raw_v4_2.csv"

    with open(raw_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "Quadrant",
            "CameraLensPosition",
            "ADC",
            "SampleIndex",
            "Brightness",
            "RawLaplacian",
            "ClaheLaplacian",
            "Tenengrad",
        ])

        for q in QUADRANT_ORDER:
            lens = selected_camera_lens[q][
                "LensPosition"
            ]

            for result in quadrant_results[q]:
                for sample_index, metrics in enumerate(
                    result["Raw"],
                    start=1,
                ):
                    writer.writerow([
                        q,
                        lens,
                        result["ADC"],
                        sample_index,
                        metrics["Brightness"],
                        metrics["RawLaplacian"],
                        metrics["ClaheLaplacian"],
                        metrics["Tenengrad"],
                    ])

    # 모든 local peak 후보 저장
    candidate_file = "projector_peak_candidates_v4_2.csv"

    with open(candidate_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "Quadrant",
            "Metric",
            "CandidateRank",
            "SampleADC",
            "RefinedADC",
            "SampleScore",
            "RefinedScore",
            "Quality",
            "Baseline",
            "AbsoluteProminence",
            "RelativeProminence",
            "Curvature",
            "HalfPromWidthPoints",
            "LocalBrightnessCorr",
            "NoiseSigma",
            "PeakSNR",
            "ShapeScore",
            "BrightnessPenaltyFactor",
            "EdgeDistance",
            "EdgePenaltyFactor",
            "PeakBrightness",
            "FitOK",
        ])

        for q in QUADRANT_ORDER:
            for metric in [
                "RawLaplacian",
                "ClaheLaplacian",
                "Tenengrad",
            ]:
                candidates = projector_analysis[q][
                    "MetricAnalysis"
                ][metric]["Candidates"]

                for rank, c in enumerate(
                    candidates,
                    start=1,
                ):
                    writer.writerow([
                        q,
                        metric,
                        rank,
                        c["SampleX"],
                        c["RefinedX"],
                        c["SampleScore"],
                        c["RefinedScore"],
                        c["Quality"],
                        c["Baseline"],
                        c["AbsoluteProminence"],
                        c["RelativeProminence"],
                        c["Curvature"],
                        c["HalfPromWidthPoints"],
                        c["LocalBrightnessCorr"],
                        c["NoiseSigma"],
                        c["PeakSNR"],
                        c["ShapeScore"],
                        c["BrightnessPenaltyFactor"],
                        c["EdgeDistance"],
                        c["EdgePenaltyFactor"],
                        c["PeakBrightness"],
                        c["FitOK"],
                    ])

    summary_file = "projector_focus_summary_v4_2.csv"

    with open(summary_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "Quadrant",
            "Position",
            "CameraLensPosition",
            "CameraLensReferenceADC",
            "CameraLensMetric",
            "SelectedProjectorMetric",
            "SampleBestADC",
            "RefinedBestADC",
            "SampleBestScore",
            "RefinedBestScore",
            "PeakQuality",
            "Confidence",
            "ConfidenceLabel",
            "QualityMargin",
            "RelativeProminence",
            "Curvature",
            "HalfPromWidthPoints",
            "LocalBrightnessCorr",
            "GlobalBrightnessCorr",
            "PeakSNR",
            "PeakBrightness",
            "EdgePeak",
            "FitOK",
            "Warnings",
            "OptionalPlaneCluster",
        ])

        for q in QUADRANT_ORDER:
            analysis = projector_analysis[q]
            best = analysis["Selected"]
            metric = analysis["SelectedMetric"]
            global_corr = analysis[
                "MetricAnalysis"
            ][metric]["GlobalBrightnessCorr"]

            writer.writerow([
                q,
                QUADRANT_NAMES[q],
                selected_camera_lens[q][
                    "LensPosition"
                ],
                selected_camera_lens[q][
                    "ReferenceADC"
                ],
                selected_camera_lens[q][
                    "Metric"
                ],
                metric,
                best["SampleX"],
                best["RefinedX"],
                best["SampleScore"],
                best["RefinedScore"],
                best["Quality"],
                best["Confidence"],
                best["ConfidenceLabel"],
                best["QualityMargin"],
                best["RelativeProminence"],
                best["Curvature"],
                best["HalfPromWidthPoints"],
                best["LocalBrightnessCorr"],
                global_corr,
                best["PeakSNR"],
                best["PeakBrightness"],
                best["IsEdge"],
                best["FitOK"],
                "|".join(best["Warnings"]),
                cluster_map[q],
            ])

    print(f"saved {median_file}")
    print(f"saved {raw_file}")
    print(f"saved {candidate_file}")
    print(f"saved {summary_file}")



# ============================================================
# Mango Comet image-split calibration JSON
# ============================================================
def save_mango_calibration_json(
    selected_adc_map,
    highest_q,
    lowest_q,
):
    """
    최종 Q별 BEST ADC 중 가장 높은/낮은 영역을
    이미지 분할 앱이 읽는 JSON 형식으로 저장한다.

    Q1=우상, Q2=좌상, Q3=좌하, Q4=우하
    """
    high_quadrant = int(highest_q[1:])
    low_quadrant = int(lowest_q[1:])

    result = {
        "high": {
            "quadrant": high_quadrant,
            "adc": float(selected_adc_map[highest_q]),
        },
        "low": {
            "quadrant": low_quadrant,
            "adc": float(selected_adc_map[lowest_q]),
        },
    }

    if result["high"]["quadrant"] == result["low"]["quadrant"]:
        raise RuntimeError(
            "High/Low ADC 사분면이 같아서 이미지 분할 방향을 정할 수 없습니다."
        )

    if result["high"]["adc"] <= result["low"]["adc"]:
        raise RuntimeError(
            "High ADC가 Low ADC보다 크지 않습니다."
        )

    destination = os.path.expanduser(
        "~/mango_comet_adc_prism_calibration.json"
    )
    destination_dir = os.path.dirname(destination) or "."
    os.makedirs(destination_dir, exist_ok=True)

    temp_name = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=destination_dir,
            prefix="mango_adc_",
            suffix=".json.tmp",
            delete=False,
        ) as tmp:
            temp_name = tmp.name
            json.dump(
                result,
                tmp,
                ensure_ascii=False,
                indent=2,
                allow_nan=False,
            )
            tmp.write("\n")
            tmp.flush()
            os.fsync(tmp.fileno())

        # 완성된 JSON만 한 번에 보이도록 원자적 교체
        os.replace(temp_name, destination)
        temp_name = None

    finally:
        if temp_name and os.path.exists(temp_name):
            try:
                os.remove(temp_name)
            except OSError:
                pass

    print()
    print("============================================")
    print(" IMAGE SPLIT CALIBRATION JSON SAVED")
    print("============================================")
    print(f"File = {destination}")
    print(
        f"HIGH = {highest_q} / "
        f"quadrant {high_quadrant} / "
        f"ADC {result['high']['adc']:.3f}"
    )
    print(
        f"LOW  = {lowest_q} / "
        f"quadrant {low_quadrant} / "
        f"ADC {result['low']['adc']:.3f}"
    )

    return result


# ============================================================
# Data containers
# ============================================================
quadrant_results = {
    q: [] for q in QUADRANT_ORDER
}

camera_stability_log = []
projector_settle_log = []
selected_camera_lens = {}
all_calibration_runs = []
projector_analysis = {}
initial_brightness = {}
exposure_info = {}


# ============================================================
# MAIN
# ============================================================
try:
    # Background UART heartbeat는 current target이 설정되기 전에는
    # 아무 것도 보내지 않는다. 첫 send_target() 이후부터 반복 송신.
    start_uart_target_heartbeat()

    initialize_debug_window()
    wait_for_user_start()

    # --------------------------------------------------------
    # PHASE -1 : ATmega 로부터 ADC 범위 수신
    # --------------------------------------------------------
    measured_min, measured_max, range_ok = request_adc_range()

    ADC_START, ADC_END, used_margin = build_sweep_range(
        measured_min,
        measured_max,
    )

    # 먼저 기존 전체 유효구간을 기준으로 step을 결정한다.
    # 낮은 ADC 쪽 start/margin은 그대로 둔다.
    step = ADC_STEP

    while ((ADC_END - ADC_START) // step) + 1 > SWEEP_MAX_STEPS:
        step += 1

    ADC_STEP = step

    # 높은 ADC 쪽 마지막 10개 측정점은 사용하지 않는다.
    # 예: 24 step -> 앞쪽 14 step만 실제 sweep.
    full_sweep_end = ADC_END
    (
        ADC_END,
        full_sweep_count,
        kept_sweep_count,
        skipped_high_steps,
    ) = trim_high_adc_tail(
        ADC_START,
        full_sweep_end,
        ADC_STEP,
        SWEEP_HIGH_ADC_TAIL_SKIP_STEPS,
    )

    # Camera lens calibration reference도 실제 사용할 ADC 범위 안에서만 잡는다.
    CAMERA_CAL_REFERENCE_ADC_SEQUENCE = (
        build_camera_cal_references(ADC_START, ADC_END)
    )

    print()
    print("============================================")
    print(" SWEEP RANGE DECIDED")
    print("============================================")
    print(
        f"Measured        = {measured_min} ~ {measured_max} "
        f"({'from ATmega' if range_ok else 'FALLBACK'})"
    )
    print(f"Margin trimmed  = {used_margin} each side")
    print(
        f"Full sweep      = {ADC_START} ~ {full_sweep_end}, "
        f"step {ADC_STEP} ({full_sweep_count} points)"
    )
    print(
        f"High-ADC trim   = last {skipped_high_steps} points skipped"
    )
    print(
        f"Actual sweep    = {ADC_START} ~ {ADC_END}, "
        f"step {ADC_STEP} ({kept_sweep_count} points)"
    )
    print(
        f"Camera cal refs = "
        f"{CAMERA_CAL_REFERENCE_ADC_SEQUENCE}"
    )

    print()
    print("============================================")
    print(" MULTI-FOCUS PROJECTOR V4.3 - AUTO RANGE")
    print("============================================")
    print(
        f"Projector selection metric = "
        f"{PROJECTOR_SELECTION_METRIC}"
    )
    print("Q1/Q2/Q3/Q4 = completely independent")

    if _st_handle is not None:
        print(
            f"ST settle = GPIO{ST_GPIO_PIN} polling, "
            f"floor {ST_MIN_SETTLE_TIME:.2f}s / "
            f"timeout {PROJECTOR_SETTLE_TIME:.2f}s"
        )
    else:
        print(
            f"ST settle = DISABLED, "
            f"fixed {PROJECTOR_SETTLE_TIME:.2f}s"
        )

    # --------------------------------------------------------
    # PHASE 0
    # --------------------------------------------------------
    first_reference_adc = (
        CAMERA_CAL_REFERENCE_ADC_SEQUENCE[0]
    )

    send_target(first_reference_adc)

    live_wait(
        PROJECTOR_SETTLE_TIME,
        current_adc=first_reference_adc,
        phase="INITIAL PROJECTOR SETTLING",
        progress_text="Before exposure lock (fixed wait)",
    )

    exposure_info = converge_and_lock_exposure()

    picam2.set_controls({
        "AfMode": controls.AfModeEnum.Manual,
        "LensPosition": float(
            np.clip(
                CAMERA_LENS_PARK_POSITION,
                CAMERA_LENS_MIN,
                CAMERA_LENS_MAX,
            )
        ),
    })
    time.sleep(0.5)

    initial_brightness = brightness_check(
        first_reference_adc
    )

    # --------------------------------------------------------
    # PHASE 1 : Q별 독립 Camera Lens calibration
    # --------------------------------------------------------
    (
        selected_camera_lens,
        all_calibration_runs,
    ) = calibrate_camera_lens(
        first_reference_already_set=True
    )

    save_camera_calibration_csv(
        selected_camera_lens,
        all_calibration_runs,
    )

    # --------------------------------------------------------
    # PHASE 2 : Projector ADC single sweep
    # --------------------------------------------------------
    print()
    print("============================================")
    print(" PHASE 2 : PROJECTOR VCM SWEEP")
    print("============================================")

    adc_values = build_adc_values()

    for adc_index, adc in enumerate(
        adc_values,
        start=1,
    ):
        print()
        print("############################################")
        print(
            f"ADC {adc} "
            f"({adc_index}/{len(adc_values)})"
        )
        print("############################################")

        send_target(adc)

        settled, settle_elapsed = settle_projector(
            adc=adc,
            phase="PROJECTOR VCM SETTLING",
            progress_text=(
                f"ADC {adc} | "
                f"{adc_index}/{len(adc_values)}"
            ),
        )

        projector_settle_log.append({
            "ADC": adc,
            "Settled": settled,
            "Elapsed": settle_elapsed,
        })

        # Q끼리 정보를 공유하지 않는다.
        # 같은 ADC에서 camera lens command만 Q별 독립 위치로 바꾼다.
        for q in QUADRANT_ORDER:
            target_lens = selected_camera_lens[q][
                "LensPosition"
            ]

            print()
            print(
                f"{q} Camera Lens -> "
                f"{target_lens:.4f} D"
            )

            lens_command = set_camera_lens_direct(
                target_position=target_lens,
                quadrant_name=q,
                current_adc=adc,
                phase="CAMERA LENS SWITCH",
                use_park=(
                    CAMERA_SWITCH_USE_UNIDIRECTIONAL_APPROACH
                ),
            )

            stability = wait_for_camera_focus_stability(
                quadrant_name=q,
                current_adc=adc,
                camera_lens_position=lens_command,
                stability_log=camera_stability_log,
            )

            median_metrics, raw_metrics = (
                measure_projector_median(
                    quadrant_name=q,
                    adc=adc,
                    camera_lens_position=lens_command,
                )
            )

            print(
                f"{q} median | "
                f"B={median_metrics['Brightness']:.1f} | "
                f"RawLap={median_metrics['RawLaplacian']:.2f} | "
                f"CLap={median_metrics['ClaheLaplacian']:.2f} | "
                f"Ten={median_metrics['Tenengrad']:.2f}"
            )

            quadrant_results[q].append({
                "ADC": adc,
                "Median": median_metrics,
                "Raw": raw_metrics,
                "Stability": stability,
            })

    # --------------------------------------------------------
    # PHASE 3 : Q별 완전 독립 local peak analysis
    # --------------------------------------------------------
    print()
    print("============================================")
    print(" PHASE 3 : INDEPENDENT LOCAL PEAK ANALYSIS")
    print("============================================")

    for q in QUADRANT_ORDER:
        projector_analysis[q] = (
            analyze_projector_quadrant(
                q,
                quadrant_results[q],
            )
        )

    # 선택 완료 후에만 optional clustering
    clusters, cluster_map = (
        cluster_plane_candidates(
            projector_analysis
        )
    )

    # --------------------------------------------------------
    # FINAL
    # --------------------------------------------------------
    print()
    print("============================================")
    print(" FINAL RESULT - INDEPENDENT Q")
    print("============================================")

    selected_adc_map = {}

    for q in QUADRANT_ORDER:
        analysis = projector_analysis[q]
        best = analysis["Selected"]
        metric = analysis["SelectedMetric"]
        global_corr = analysis[
            "MetricAnalysis"
        ][metric]["GlobalBrightnessCorr"]

        selected_adc_map[q] = float(
            best["RefinedX"]
        )

        warning_text = (
            ", ".join(best["Warnings"])
            if best["Warnings"]
            else "OK"
        )

        print()
        print(
            f"{q} ({QUADRANT_NAMES[q]})"
        )
        print(
            f"  Camera Lens      = "
            f"{selected_camera_lens[q]['LensPosition']:.4f} D "
            f"[{selected_camera_lens[q]['Metric']}]"
        )
        print(
            f"  Sample BEST ADC  = {best['SampleX']:.1f}"
        )
        print(
            f"  Refined BEST ADC = {best['RefinedX']:.3f}"
        )
        print(
            f"  Peak Quality     = {best['Quality']:.3f}"
        )
        print(
            f"  Confidence       = "
            f"{best['Confidence']:.3f} "
            f"({best['ConfidenceLabel']})"
        )
        print(
            f"  Rel Prominence   = "
            f"{best['RelativeProminence']:.3f}"
        )
        print(
            f"  Curvature        = {best['Curvature']:.3f}"
        )
        print(
            f"  HalfProm Width   = "
            f"{best['HalfPromWidthPoints']} points"
        )
        print(
            f"  Local B Corr     = "
            f"{best['LocalBrightnessCorr']:.3f}"
        )
        print(
            f"  Global B Corr    = {global_corr:.3f}"
        )
        print(
            f"  Peak SNR         = {best['PeakSNR']:.2f}"
        )
        print(
            f"  Peak Brightness  = "
            f"{best['PeakBrightness']:.1f}"
        )
        print(
            f"  Verdict          = {warning_text}"
        )

        # top candidate 3개도 터미널에 표시
        print("  Candidate ranking:")
        candidates = analysis[
            "MetricAnalysis"
        ][metric]["Candidates"]

        for rank, candidate in enumerate(
            candidates[:3],
            start=1,
        ):
            print(
                f"    #{rank} ADC "
                f"{candidate['SampleX']:.1f} -> "
                f"{candidate['RefinedX']:.3f} | "
                f"Q={candidate['Quality']:.3f} | "
                f"prom={candidate['RelativeProminence']:.3f} | "
                f"corrB={candidate['LocalBrightnessCorr']:.3f}"
            )

    highest_q = max(
        selected_adc_map,
        key=selected_adc_map.get,
    )
    lowest_q = min(
        selected_adc_map,
        key=selected_adc_map.get,
    )

    print()
    print("--------------------------------------------")
    print(
        f"Highest ADC Area = {highest_q} "
        f"({selected_adc_map[highest_q]:.3f})"
    )
    print(
        f"Lowest ADC Area  = {lowest_q} "
        f"({selected_adc_map[lowest_q]:.3f})"
    )
    print("--------------------------------------------")

    # 이미지 분할 앱에서 사용할 최종 calibration JSON 저장
    save_mango_calibration_json(
        selected_adc_map=selected_adc_map,
        highest_q=highest_q,
        lowest_q=lowest_q,
    )

    print()
    print("OPTIONAL POST-HOC PLANE CANDIDATES")
    print(
        "※ 아래 grouping은 BEST ADC 계산에 전혀 사용되지 않았습니다."
    )

    for cluster_index, cluster in enumerate(
        clusters,
        start=1,
    ):
        members = ", ".join(
            f"{item['Quadrant']}({item['ADC']:.2f})"
            for item in cluster
        )
        print(
            f"  PlaneCandidate{cluster_index}: {members}"
        )

    # ST 절약 시간 요약
    print_st_summary()

    # --------------------------------------------------------
    # Save CSVs
    # --------------------------------------------------------
    save_camera_stability_csv(
        camera_stability_log
    )

    save_projector_settle_csv(
        projector_settle_log
    )

    save_projector_csvs(
        adc_values=adc_values,
        quadrant_results=quadrant_results,
        selected_camera_lens=selected_camera_lens,
        projector_analysis=projector_analysis,
        cluster_map=cluster_map,
    )

    print()
    print("측정 완료.")
    print("Debug Window에서 Q를 누르면 종료.")

    while True:
        frame = picam2.capture_array()
        key = show_dashboard(
            frame=frame,
            phase="TEST COMPLETE - PRESS Q",
            progress_text=(
                f"V4.2 RawLap + ST settle | "
                f"Highest={highest_q} | Lowest={lowest_q}"
            ),
        )

        if key == ord("q"):
            break


# ============================================================
# Cleanup
# ============================================================
except KeyboardInterrupt:
    print()
    print("사용자가 중단했습니다.")

except Exception as e:
    print()
    print("ERROR:")
    print(e)

    import traceback
    traceback.print_exc()

finally:
    # serial port를 닫기 전에 heartbeat thread부터 반드시 종료
    try:
        stop_uart_target_heartbeat()
    except Exception:
        pass

    try:
        picam2.stop()
    except Exception:
        pass

    try:
        picam2.close()
    except Exception:
        pass

    try:
        ser.close()
    except Exception:
        pass

    try:
        if _st_handle is not None:
            lgpio.gpiochip_close(_st_handle)
    except Exception:
        pass

    cv2.destroyAllWindows()

    print()
    print("Camera stopped")
    print("UART closed")
    print("ST GPIO closed")
    print("Done")
