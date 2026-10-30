"""Mango Comet ADC Prism — one Python file, no installer.

Run on the Pi: /usr/bin/python3 -s -B THIS_FILE.py
Open on the laptop: http://pi5-dlp.local:5088/

Calibration input (read only):
  ~/mango_comet_adc_prism_calibration.json
  {"high": {"quadrant": 1, "adc": 750},
   "low":  {"quadrant": 3, "adc": 300}}
The numbers above are examples, never built-in defaults.
Quadrants: top-left=2, top-right=1, bottom-left=3, bottom-right=4.
The calibration program must publish its actual completed results to that file.
Use write-to-temp + os.replace to publish all four fields together.

Producer example: copy this function into the calibration program and call it
with its real result variables once calibration is complete (no web app needed):

    def publish_mango_result(high_q, high_adc, low_q, low_adc):
        import json, os, tempfile
        from pathlib import Path
        destination = Path.home() / 'mango_comet_adc_prism_calibration.json'
        result = {'high': {'quadrant': int(high_q), 'adc': float(high_adc)},
                  'low': {'quadrant': int(low_q), 'adc': float(low_adc)}}
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8',
                dir=destination.parent, prefix='mango_adc_', delete=False) as tmp:
            json.dump(result, tmp, allow_nan=False)
        os.replace(tmp.name, destination)

Only this new app's image copies and its own renderer are managed here.
The original capstone files/processes and GPIO27 are untouched. The display manager is temporarily stopped only during KMSDRM flip output and restored afterward.
GPIO17 phase: HIGH=FRONT(high-ADC image), LOW=BACK(low-ADC image).
This app configures the Adaptive-Base ATmega FRONT/BACK targets over UART before flip start.
GPIO17 is phase-advanced before each image swap; actual swap cadence is measured with perf_counter().
Calibration is snapshotted at Start; new results apply to the next run.
"""

import os
import sys
import json
import csv
import gc
import math
import time
import signal
import pwd
import secrets
import threading
import subprocess
import base64
import zlib
import builtins
from collections import deque
from functools import lru_cache
from pathlib import Path


PORT = 5088

# Integrated autofocus/calibration child. No external auto_focus_test_division20.py is required.
CAL_PREVIEW_PATH = Path('/tmp/mango_integrated_calibration_preview.jpg')
CAL_PREVIEW_MAX_AGE = 0

# ================================================================
# [3-button web settings]
# ================================================================
# Checkerboard shown on the projector during calibration.
# If the file is missing, calibration runs without it (old behavior).
CALIBRATION_IMAGE = '/home/piuser/checkerboard.png'

# Used only if the embedded autofocus code fails its integrity check.
CALIBRATION_FALLBACK_SCRIPT = '/home/piuser/auto_focus_test_division20.py'

# Remembers the last selected flip image across restarts.
WEB_STATE_FILE = Path('/home/piuser/.mango_web_state.json')

# FLIP button settings (lead time = GPIO_PHASE_LEAD_MS_DEFAULT below).
QUICK_FLIP_FPS = 60.0

# Order of the two numbers in T<a>,<b> sent to the ATmega.
#   'high_low' : T<HIGH ADC>,<LOW ADC>   (mango_file19 original)
#   'low_high' : T<LOW ADC>,<HIGH ADC>   (VCM_Unified.c: 2nd value = GPIO17 HIGH)
# GPIO17 HIGH shows the HIGH-ADC image, so the firmware must drive the
# HIGH ADC while GPIO17 is HIGH. Pick the order that matches your firmware.
PAIR_ORDER = 'high_low'

# Max wait for the checkerboard to appear (seconds).
CALIBRATION_DISPLAY_WAIT = 10.0



def resolve_runtime_user_home():
    """
    sudo로 Mango 전체를 실행해도 calibration/image 경로가 /root로
    바뀌지 않도록 실제 로그인 사용자의 home을 사용한다.
    """
    sudo_user = os.environ.get('SUDO_USER', '').strip()

    if (
        os.geteuid() == 0
        and sudo_user
        and sudo_user != 'root'
    ):
        try:
            return Path(
                pwd.getpwnam(sudo_user).pw_dir
            )
        except KeyError:
            pass

    return Path.home()


USER_HOME = resolve_runtime_user_home()

CALIBRATION_FILE = USER_HOME / 'mango_comet_adc_prism_calibration.json'

# Camera Module 3 lens calibration result written by the ORIGINAL
# embedded calibration code. The separate projector-sweep button reuses
# these Q1~Q4 positions without running camera-lens calibration again.
CAMERA_LENS_SUMMARY_FILE = USER_HOME / 'camera_lens_calibration_summary_v4_2.csv'

DATA_DIR = Path(__file__).resolve().with_suffix('') / 'images'
IMAGE_DIR = USER_HOME / 'images'
PREVIEW_SIZE = (640, 360)
MAX_IMAGE_BYTES = 32 * 1024 * 1024
IMAGE_EXTENSIONS = {'.png', '.jpg', '.jpeg', '.bmp', '.webp'}

# Fixed-position focus mode
UART_PORT = '/dev/ttyAMA2'
UART_BAUD = 115200
HOLD_HEARTBEAT_SECONDS = 0.20
FOCUS_CSV_NAME = 'projector_focus_median_v4_2.csv'

# "앞에서 봐줄 만한" 범위의 기본 기준.
# 앞 ROI RawLaplacian이 앞 BEST 점수의 이 비율 이상인 연속 구간만 허용한다.
FRONT_ACCEPTABLE_RATIO_DEFAULT = 0.90

# Image/VCM timing
GPIO_PHASE_LEAD_MS_DEFAULT = 6.0
GPIO_PHASE_LEAD_MS_MAX = 10.0
TIMING_REPORT_INTERVAL = 1.0

# Adaptive FLIP only:
# BACK is the LOW-ADC optical plane.
# Command slightly farther toward lower ADC so the moving lens crosses
# the real BACK focus point earlier. Fixed-focus/CAL modes do not use this.
BACK_FAKE_TARGET_OFFSET_DEFAULT = 2.0
BACK_FAKE_TARGET_OFFSET_MAX = 8.0

# KMSDRM / display-manager handoff
AUTO_DISPLAY_MANAGER_HANDOFF = False
DISPLAY_MANAGER_UNIT = 'display-manager.service'
DISPLAY_MANAGER_STOP_TIMEOUT = 8.0
DISPLAY_MANAGER_START_TIMEOUT = 12.0
DRM_RELEASE_SETTLE_SECONDS = 1.0

LEFT_QUADRANTS = {2, 3}
RIGHT_QUADRANTS = {1, 4}


def validate_calibration(data):
    """기존 JSON은 유지하되 분할은 좌/우로만 해석한다."""
    if not isinstance(data, dict):
        raise ValueError('캘리브레이션 결과는 JSON 객체여야 합니다.')

    result = {}
    for phase in ('high', 'low'):
        item = data.get(phase)
        if not isinstance(item, dict):
            raise ValueError(f'캘리브레이션에 {phase} 정보가 없습니다.')

        q = item.get('quadrant')
        adc = item.get('adc')

        if type(q) is not int or q not in {1, 2, 3, 4}:
            raise ValueError(f'{phase}.quadrant는 1~4 정수여야 합니다.')
        if type(adc) not in (int, float) or not math.isfinite(adc) or adc < 0:
            raise ValueError(f'{phase}.adc는 0 이상의 유한한 숫자여야 합니다.')

        result[phase] = {
            'quadrant': q,
            'adc': adc,
            'side': 'left' if q in LEFT_QUADRANTS else 'right',
        }

    if result['high']['adc'] <= result['low']['adc']:
        raise ValueError('high.adc가 low.adc보다 커야 합니다.')

    if result['high']['side'] == result['low']['side']:
        raise ValueError(
            'HIGH/LOW가 같은 좌우 영역입니다. '
            '좌우 분할은 한쪽이 Q2/Q3, 다른쪽이 Q1/Q4일 때만 가능합니다.'
        )

    return result


def read_calibration(path=None):
    path = CALIBRATION_FILE if path is None else Path(path)
    try:
        with path.open('rb') as stream:
            raw = stream.read(16385)
    except FileNotFoundError:
        raise ValueError('캘리브레이션 결과 대기: ' + str(path)) from None
    if len(raw) > 16384:
        raise ValueError('캘리브레이션 JSON 파일이 너무 큽니다.')
    try:
        return validate_calibration(json.loads(raw))
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise ValueError('캘리브레이션 JSON을 읽지 못했습니다. 저장 완료 후 다시 확인합니다.') from exc


def read_saved_camera_lens_calibration(path=None):
    """
    Read the last successful ORIGINAL camera-lens calibration.

    Required output from camera_lens_calibration_summary_v4_2.csv:
      Q1~Q4 SelectedLensPosition
      ReferenceADC
      SelectedMetric

    The projector-sweep mode only needs LensPosition for control, but
    ReferenceADC/Metric are preserved for CSV/report compatibility.
    """
    path = CAMERA_LENS_SUMMARY_FILE if path is None else Path(path)

    try:
        stream = path.open('r', encoding='utf-8-sig', newline='')
    except FileNotFoundError:
        raise ValueError(
            '저장된 렌즈 캘리브레이션이 없습니다. '
            '먼저 "렌즈 캘리브레이션"을 실행하세요.'
        ) from None

    with stream:
        reader = csv.DictReader(stream)

        required = {
            'Quadrant',
            'SelectedLensPosition',
            'ReferenceADC',
            'SelectedMetric',
        }
        missing = required.difference(reader.fieldnames or [])

        if missing:
            raise ValueError(
                '렌즈 캘리브레이션 CSV 형식이 맞지 않습니다: '
                + ', '.join(sorted(missing))
            )

        result = {}

        for row in reader:
            q = str(row.get('Quadrant') or '').strip()

            if q not in {'Q1', 'Q2', 'Q3', 'Q4'}:
                continue

            try:
                lens = float(row['SelectedLensPosition'])
                reference_adc = float(row['ReferenceADC'])
            except (TypeError, ValueError):
                raise ValueError(
                    f'{q} 저장 렌즈값의 숫자 형식이 잘못되었습니다.'
                ) from None

            if not math.isfinite(lens) or not math.isfinite(reference_adc):
                raise ValueError(
                    f'{q} 저장 렌즈값에 유효하지 않은 숫자가 있습니다.'
                )

            result[q] = {
                'LensPosition': float(lens),
                'ReferenceADC': int(round(reference_adc)),
                'Metric': str(row.get('SelectedMetric') or 'REUSED_SAVED'),
            }

    missing_q = [
        q for q in ('Q1', 'Q2', 'Q3', 'Q4')
        if q not in result
    ]

    if missing_q:
        raise ValueError(
            '저장된 렌즈 캘리브레이션에 값이 부족합니다: '
            + ', '.join(missing_q)
        )

    return result


def saved_camera_lens_status():
    try:
        values = read_saved_camera_lens_calibration()

        return {
            'available': True,
            'file': str(CAMERA_LENS_SUMMARY_FILE),
            'values': values,
        }

    except Exception as exc:
        return {
            'available': False,
            'file': str(CAMERA_LENS_SUMMARY_FILE),
            'values': {},
            'error': str(exc),
        }


@lru_cache(maxsize=48)
def split_rows(width, height, high_side):
    """중앙 수직선 기준 좌/우 50:50 분할만 사용."""
    if width < 1 or height < 1:
        raise ValueError('이미지 크기는 양수여야 합니다.')
    if high_side not in {'left', 'right'}:
        raise ValueError('high_side는 left 또는 right여야 합니다.')

    middle = width // 2
    span = (0, middle) if high_side == 'left' else (middle, width)
    return tuple(span for _ in range(height))


def calibration_view(calibration):
    return {
        **calibration,
        # Display slice mapping is intentionally opposite to the old UI:
        # FRONT/HIGH shows the LOW-side half, BACK/LOW shows the HIGH-side half.
        'high_side': calibration['low']['side'],
        'low_side': calibration['high']['side'],
        'label': '중앙 수직선 · 앞/뒤 초점 분할',
        # rows now describe the FRONT/HIGH DISPLAY slice.
        'rows': split_rows(*PREVIEW_SIZE, calibration['low']['side']),
    }


def make_pictures(pygame, source, calibration):
    width, height = source.get_size()
    high = pygame.Surface((width, height))
    low = pygame.Surface((width, height))
    high.fill((0, 0, 0))
    low.fill((0, 0, 0))
    rows = split_rows(width, height, calibration['high']['side'])
    for y, (left, right) in enumerate(rows):
        if right > left:
            high.blit(source, (left, y), (left, y, right - left, 1))
        if left:
            low.blit(source, (0, y), (0, y, left, 1))
        if right < width:
            low.blit(source, (right, y), (right, y, width - right, 1))
    # IMPORTANT:
    # Calibration HIGH/LOW tells us the lens ADC planes, but the actual
    # image half associated with FRONT/BACK is the opposite side from the
    # old UI interpretation.
    #
    # Swap image slices only:
    #   pictures[0] = FRONT / HIGH ADC -> opposite half
    #   pictures[1] = BACK  / LOW  ADC -> former HIGH-side half
    #
    # ADC targets, GPIO phase and calibration values are NOT swapped.
    return low, high



def _finite_float(value, name):
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise ValueError(f'{name} 값이 숫자가 아닙니다.') from None
    if not math.isfinite(number):
        raise ValueError(f'{name} 값이 유한하지 않습니다.')
    return number


def _normalize_curve(values):
    lo = min(values)
    hi = max(values)
    span = hi - lo
    if span <= 1e-12:
        raise ValueError('초점 스코어 곡선이 평평해서 대비 ADC를 정할 수 없습니다.')
    return [(value - lo) / span for value in values]


def find_focus_csv(path_hint=''):
    """Find the projector sweep CSV without changing the calibration program."""
    candidates = []

    if isinstance(path_hint, str) and path_hint.strip():
        candidates.append(Path(path_hint).expanduser())

    # Common places used by this project.
    candidates.extend([
        Path.home() / FOCUS_CSV_NAME,
        Path(__file__).resolve().parent / FOCUS_CSV_NAME,
        Path.cwd() / FOCUS_CSV_NAME,
        Path('/home/piuser/capstone') / FOCUS_CSV_NAME,
    ])

    seen = set()
    for path in candidates:
        try:
            resolved = path.resolve()
        except OSError:
            resolved = path

        key = str(resolved)
        if key in seen:
            continue
        seen.add(key)

        if resolved.is_file():
            return resolved

    attempted = '\n'.join('  - ' + str(p) for p in candidates)
    raise ValueError(
        '자동초점 스윕 CSV를 찾지 못했습니다.\n'
        f'필요 파일: {FOCUS_CSV_NAME}\n'
        '확인한 위치:\n' + attempted
    )


def choose_front_focus_margin_away_from_back(
    calibration,
    csv_path,
    acceptable_ratio=FRONT_ACCEPTABLE_RATIO_DEFAULT,
):
    """
    목표:
      1) FRONT의 BEST ADC를 기준으로, 앞 초점이 아직 '봐줄 만한' 연속 구간을 찾는다.
      2) 그 허용 구간 안에서 BACK BEST ADC와 가장 멀리 떨어진 ADC를 고른다.

    예:
      FRONT BEST = 290
      BACK  BEST = 270
      FRONT 허용 구간 = 284 ~ 296
      -> 270에서 가장 먼 296 선택

    FRONT/BACK 기준:
      calibration['high'] = FRONT
      calibration['low']  = BACK

    허용 범위:
      FRONT RawLaplacian >= FRONT peak * acceptable_ratio
      단, 잡음 때문에 떨어진 다른 봉우리를 고르지 않도록
      FRONT BEST sample을 포함하는 '연속된 구간'만 사용한다.
    """
    ratio = float(acceptable_ratio)

    if not math.isfinite(ratio) or not 0.50 <= ratio <= 1.00:
        raise ValueError('앞 초점 허용 비율은 0.50~1.00 사이여야 합니다.')

    front_q = int(calibration['high']['quadrant'])
    back_q = int(calibration['low']['quadrant'])

    front_best_adc = float(calibration['high']['adc'])
    back_best_adc = float(calibration['low']['adc'])

    if front_q == back_q:
        raise ValueError('앞/뒤 기준 사분면이 같습니다.')

    front_column = f'Q{front_q}_RawLaplacian'

    rows = []

    with Path(csv_path).open('r', encoding='utf-8-sig', newline='') as stream:
        reader = csv.DictReader(stream)

        required = {'ADC', front_column}
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(
                '스윕 CSV에 필요한 열이 없습니다: '
                + ', '.join(sorted(missing))
            )

        for line_no, row in enumerate(reader, start=2):
            try:
                adc = _finite_float(
                    row.get('ADC'),
                    f'ADC(line {line_no})',
                )
                front_score = _finite_float(
                    row.get(front_column),
                    f'{front_column}(line {line_no})',
                )
            except ValueError:
                continue

            rows.append({
                'adc': float(adc),
                'front_score': float(front_score),
            })

    if len(rows) < 3:
        raise ValueError('유효한 스윕 데이터가 3개 미만입니다.')

    rows.sort(key=lambda item: item['adc'])

    # JSON의 refined front BEST ADC와 가장 가까운 실제 sweep sample을 중심점으로 사용.
    best_index = min(
        range(len(rows)),
        key=lambda i: abs(rows[i]['adc'] - front_best_adc),
    )

    # 실제 sample curve의 local FRONT peak를 기준으로 threshold를 잡는다.
    # refined ADC가 sample 사이에 있더라도 안정적으로 동작한다.
    local_peak_score = max(
        rows[best_index]['front_score'],
        max(item['front_score'] for item in rows),
    )
    threshold = local_peak_score * ratio

    # BEST sample을 포함하는 연속 threshold 구간만 확장한다.
    left = best_index
    right = best_index

    while (
        left - 1 >= 0
        and rows[left - 1]['front_score'] >= threshold
    ):
        left -= 1

    while (
        right + 1 < len(rows)
        and rows[right + 1]['front_score'] >= threshold
    ):
        right += 1

    acceptable = rows[left:right + 1]

    if not acceptable:
        raise ValueError('앞 초점 허용 구간을 찾지 못했습니다.')

    # 핵심: FRONT 허용 구간 안에서 BACK BEST ADC와 가장 멀리 떨어진 ADC.
    # 동률이면 FRONT score가 더 높은 쪽을 선택.
    chosen = max(
        acceptable,
        key=lambda item: (
            abs(item['adc'] - back_best_adc),
            item['front_score'],
        ),
    )

    return {
        'adc': int(round(chosen['adc'])),
        'front_quadrant': front_q,
        'back_quadrant': back_q,
        'front_best_adc': float(front_best_adc),
        'back_best_adc': float(back_best_adc),
        'front_score': float(chosen['front_score']),
        'front_peak_score': float(local_peak_score),
        'acceptable_ratio': float(ratio),
        'acceptable_threshold': float(threshold),
        'acceptable_adc_min': float(acceptable[0]['adc']),
        'acceptable_adc_max': float(acceptable[-1]['adc']),
        'distance_from_back_best': float(
            abs(chosen['adc'] - back_best_adc)
        ),
        'csv': str(Path(csv_path)),
        'samples': len(rows),
    }


def uart_owner_process(device=UART_PORT):
    """
    Best-effort check for another process already holding the Pi UART.
    Avoids fighting with the autofocus/calibration script.
    """
    target = str(Path(device))
    me = os.getpid()

    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue

        pid = int(proc.name)
        if pid == me:
            continue

        fd_dir = proc / 'fd'
        try:
            fds = list(fd_dir.iterdir())
        except OSError:
            continue

        for fd in fds:
            try:
                link = os.readlink(fd)
            except OSError:
                continue

            if link != target:
                continue

            try:
                args = [
                    item.decode(errors='replace')
                    for item in (proc / 'cmdline').read_bytes().split(b'\0')
                    if item
                ]
                command = ' '.join(args)
            except OSError:
                command = ''

            return f'PID {pid}' + (f': {command}' if command else '')

    return ''



def send_adaptive_target_pair(front_adc, back_adc):
    """
    Adaptive Base ATmega UART1 protocol:
        T<FRONT_ADC>,<BACK_ADC>\n

    현재 시스템의 물리 매핑:
        FRONT = HIGH ADC plane
        BACK  = LOW ADC plane

    Mango GPIO17:
        HIGH -> HIGH ADC image -> FRONT
        LOW  -> LOW ADC image  -> BACK
    """
    import serial

    front_adc = int(round(front_adc))
    back_adc = int(round(back_adc))

    for name, adc in (('FRONT', front_adc), ('BACK', back_adc)):
        if not 100 <= adc <= 1023:
            raise ValueError(
                f'{name} ADC {adc}가 ATmega 허용 범위(100~1023)를 벗어났습니다.'
            )

    owner = uart_owner_process()
    if owner:
        raise ValueError(
            'UART가 다른 프로그램에서 사용 중입니다. '
            '자동초점/캘리브레이션 프로그램을 완전히 종료한 뒤 실행하세요.\n'
            + owner
        )

    command = f'T{front_adc},{back_adc}\n'.encode()

    port = serial.Serial(
        port=UART_PORT,
        baudrate=UART_BAUD,
        timeout=0.15,
        write_timeout=0.5,
    )

    try:
        try:
            port.reset_input_buffer()
        except Exception:
            pass

        port.write(command)
        port.flush()

        response = ''
        try:
            raw = port.readline()
            if raw:
                response = raw.decode(errors='replace').strip()
        except Exception:
            response = ''

        return {
            'front_adc': front_adc,
            'back_adc': back_adc,
            'command': command.decode().strip(),
            'response': response,
        }
    finally:
        port.close()


def find_gpiochip(dev_dir=Path('/dev')):
    import fcntl

    errors = []
    for device in sorted(dev_dir.glob('gpiochip*')):
        number = device.name.removeprefix('gpiochip')
        if not number.isdigit():
            continue
        try:
            # GPIO_GET_CHIPINFO_IOCTL: name[32], label[32], lines(uint32).
            info = bytearray(68)
            with device.open('rb', buffering=0) as stream:
                fcntl.ioctl(stream.fileno(), 0x8044B401, info, True)
            label = info[32:64].split(b'\0', 1)[0].decode(errors='replace')
            if 'pinctrl-rp1' in label:
                return int(number)
        except OSError as exc:
            errors.append(f'{device.name}: {exc}')
    detail = '; '.join(errors) if errors else 'pinctrl-rp1 칩이 조회되지 않았습니다.'
    raise RuntimeError('Pi 5 GPIO 장치를 찾지 못했습니다. ' + detail)


def existing_output():
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            args = [a.decode(errors='replace') for a in
                    (proc / 'cmdline').read_bytes().split(b'\0') if a]
            if not args or Path(args[0]).name in {'bash', 'sh', 'sshd'}:
                continue
            names = [Path(a).name for a in args]
            original = any(a.startswith('/home/piuser/capstone/') for a in args)
            if 'app.py' in names:
                original |= os.readlink(proc / 'cwd').startswith('/home/piuser/capstone')
            flip = any(n in {'image_flip.py', 'imageflip.py', 'motor_control.py',
                             'mango_comet_pixels.py', 'dlp160cp_uart_setup.py'}
                       or n.startswith('split_test') and n.endswith('.py') for n in names)
            another = '--mango-display' in args
            if original or flip or another:
                return f'PID {proc.name}: ' + ' '.join(args)
        except OSError:
            continue
    return ''



def _systemctl_base_command():
    """
    root면 systemctl 직접 사용.
    일반 사용자면 비대화형 sudo(-n)를 사용한다.

    sudo password prompt가 필요한 상태에서는 웹 요청이 멈추지 않고
    즉시 오류를 내도록 -n을 쓴다.
    """
    if os.geteuid() == 0:
        return ['systemctl']

    return ['sudo', '-n', 'systemctl']


def _run_systemctl(*args, timeout=8.0):
    command = _systemctl_base_command() + list(args)

    try:
        result = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=False,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        return (
            124,
            '',
            f"{' '.join(command)} timeout ({timeout:.1f}s)",
        )
    except Exception as exc:
        return (
            125,
            '',
            str(exc),
        )

    return (
        result.returncode,
        result.stdout.strip(),
        result.stderr.strip(),
    )


def _display_manager_is_active():
    code, stdout, stderr = _run_systemctl(
        'is-active',
        DISPLAY_MANAGER_UNIT,
        timeout=3.0,
    )

    # systemctl is-active:
    # 0 -> active
    # 3 -> inactive/failed
    if code == 0:
        return True

    if stdout in {
        'inactive',
        'failed',
        'unknown',
        'deactivating',
    }:
        return False

    # sudo permission failure should be explicit.
    if (
        'password is required' in stderr.lower()
        or 'a password is required' in stderr.lower()
        or 'sudo:' in stderr.lower()
    ):
        raise RuntimeError(
            'display-manager 자동 제어 권한이 없습니다. '
            'Mango를 SSH/TTY에서 sudo -E로 한 번 실행하면 '
            '플립마다 stop/start는 자동 처리됩니다. '
            f'상세: {stderr or stdout}'
        )

    # Alias/unit absent: treat as inactive only if clearly missing.
    missing_text = (stdout + ' ' + stderr).lower()

    if (
        'could not be found' in missing_text
        or 'not found' in missing_text
        or 'no such file' in missing_text
    ):
        return False

    raise RuntimeError(
        'display-manager 상태 확인 실패: '
        + (stderr or stdout or f'exit={code}')
    )


def _wait_display_manager_state(
    want_active,
    timeout,
):
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        try:
            active = _display_manager_is_active()
        except RuntimeError:
            active = not want_active

        if active == want_active:
            return True

        time.sleep(0.15)

    return False


def stop_display_manager_for_kms():
    """
    KMSDRM이 DRM master를 잡을 수 있도록 graphical display-manager를
    플립 시작 직전에 자동 정지한다.

    반환값:
      {
        'was_active': bool,
        'stopped_by_us': bool
      }
    """
    state = {
        'was_active': False,
        'stopped_by_us': False,
    }

    if not AUTO_DISPLAY_MANAGER_HANDOFF:
        return state

    active = _display_manager_is_active()
    state['was_active'] = bool(active)

    if not active:
        print(
            '[DM] display-manager already inactive',
            flush=True,
        )
        return state

    print(
        '[DM] display-manager STOP 요청',
        flush=True,
    )

    code, stdout, stderr = _run_systemctl(
        'stop',
        DISPLAY_MANAGER_UNIT,
        timeout=DISPLAY_MANAGER_STOP_TIMEOUT,
    )

    if code != 0:
        raise RuntimeError(
            'display-manager 자동 STOP 실패: '
            + (stderr or stdout or f'exit={code}')
        )

    if not _wait_display_manager_state(
        want_active=False,
        timeout=DISPLAY_MANAGER_STOP_TIMEOUT,
    ):
        raise RuntimeError(
            'display-manager가 제한 시간 안에 종료되지 않았습니다.'
        )

    state['stopped_by_us'] = True

    print(
        f'[DM] STOP 완료 · DRM release '
        f'{DRM_RELEASE_SETTLE_SECONDS:.1f}s 대기',
        flush=True,
    )

    time.sleep(
        DRM_RELEASE_SETTLE_SECONDS
    )

    return state


def restore_display_manager(dm_state):
    """
    이번 renderer가 실제로 끈 display-manager만 다시 시작한다.
    """
    if not dm_state:
        return

    if not dm_state.get('stopped_by_us'):
        return

    print(
        '[DM] display-manager START 요청',
        flush=True,
    )

    code, stdout, stderr = _run_systemctl(
        'start',
        DISPLAY_MANAGER_UNIT,
        timeout=DISPLAY_MANAGER_START_TIMEOUT,
    )

    if code != 0:
        print(
            '[DM] START 실패: '
            + (stderr or stdout or f'exit={code}'),
            flush=True,
        )
        return

    if _wait_display_manager_state(
        want_active=True,
        timeout=DISPLAY_MANAGER_START_TIMEOUT,
    ):
        print(
            '[DM] START 완료',
            flush=True,
        )
    else:
        print(
            '[DM] START 명령은 성공했지만 active 확인 timeout',
            flush=True,
        )


def ensure_safe_kms_launch_context():
    """
    데스크톱 터미널에서 실행된 Mango가 자기 display-manager를 끄면
    웹서버 자체가 같이 종료될 수 있다.

    SSH/TTY/systemd/root 독립 실행이면 안전.
    """
    if not AUTO_DISPLAY_MANAGER_HANDOFF:
        return

    has_ssh = bool(
        os.environ.get('SSH_CONNECTION')
        or os.environ.get('SSH_TTY')
    )

    graphical_env = bool(
        os.environ.get('WAYLAND_DISPLAY')
        or os.environ.get('DISPLAY')
    )

    if graphical_env and not has_ssh:
        raise RuntimeError(
            '현재 Mango 웹서버가 그래픽 데스크톱 세션에서 실행 중입니다. '
            '이 상태에서 display-manager를 자동 종료하면 Mango 자체가 '
            '같이 종료될 수 있습니다. '
            'SSH/TTY에서 Mango를 한 번 실행한 뒤 사용하세요.'
        )


def display_images(config):
    stop = threading.Event()
    pygame = None
    gpio = None
    chip = None
    claimed = False
    dm_state = None

    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())

    def control_pipe():
        sys.stdin.readline()  # STOP command or EOF when the parent exits.
        stop.set()

    def wait_until(deadline):
        """
        60 Hz phase timing용 wait.
        긴 구간은 sleep하고 마지막 짧은 구간만 perf_counter polling한다.
        """
        while not stop.is_set():
            remaining = deadline - time.perf_counter()

            if remaining <= 0:
                return

            if remaining > 0.002:
                time.sleep(max(0.0, remaining - 0.001))
            elif remaining > 0.00035:
                time.sleep(0)
            else:
                # 마지막 수백 us만 짧게 spin해 scheduler jitter를 줄인다.
                pass

    def window_stats(values):
        values = list(values)

        if not values:
            return 0.0, 0.0, 0.0, 0.0

        mean = sum(values) / len(values)
        variance = sum((value - mean) ** 2 for value in values) / len(values)

        return (
            mean,
            math.sqrt(max(0.0, variance)),
            min(values),
            max(values),
        )

    try:
        threading.Thread(target=control_pipe, daemon=True).start()

        # ----------------------------------------------------
        # Wayland/labwc가 DRM master를 잡고 있으면 KMSDRM이 열리지 않으므로
        # 플립 시작 직전에 display-manager를 자동 정지한다.
        # ----------------------------------------------------
        dm_state = stop_display_manager_for_kms()

        # ----------------------------------------------------
        # Direct KMS/DRM output
        #
        # Wayland/labwc compositor를 거치지 않고 SDL KMSDRM backend가
        # DRM/KMS에 직접 page-flip 하도록 한다.
        #
        # DISPLAY / WAYLAND_DISPLAY를 남겨두면 SDL이 desktop backend로
        # 돌아갈 여지가 있으므로 child renderer에서 명시적으로 제거한다.
        # ----------------------------------------------------
        os.environ.pop('DISPLAY', None)
        os.environ.pop('WAYLAND_DISPLAY', None)
        os.environ.pop('XDG_SESSION_TYPE', None)

        os.environ.update(
            SDL_VIDEODRIVER='kmsdrm',
            SDL_AUDIODRIVER='dummy',
            PYGAME_HIDE_SUPPORT_PROMPT='1',
        )

        print(
            '[VIDEO] backend 요청: KMSDRM direct output '
            '(Wayland/labwc compositor bypass)',
            flush=True,
        )

        import pygame as pg
        pygame = pg

        try:
            pygame.init()
            pygame.display.init()
            pygame.mouse.set_visible(False)
        except Exception as exc:
            raise RuntimeError(
                'KMSDRM display 초기화 실패: '
                f'{exc}'
            )

        # 프로젝터 display mode:
        # DOUBLEBUF + FULLSCREEN, vsync=0
        flags = pygame.DOUBLEBUF | pygame.FULLSCREEN
        size = (0, 0)

        try:
            desktop_sizes = pygame.display.get_desktop_sizes()
        except Exception:
            desktop_sizes = []

        print(
            f'[VIDEO] pygame driver={pygame.display.get_driver()} '
            f'| desktop_sizes={desktop_sizes}',
            flush=True,
        )

        if pygame.display.get_driver().lower() != 'kmsdrm':
            raise RuntimeError(
                'SDL이 KMSDRM backend로 시작되지 않았습니다. '
                f'현재 driver={pygame.display.get_driver()}'
            )

        # ----------------------------------------------------
        # KMSDRM 출력에서는 VSync를 요청하지 않는다.
        # 화면 속도는 pygame.time.Clock().tick(fps)로 제어한다.
        # ----------------------------------------------------
        vsync_requested = False
        vsync_set_mode_ok = True

        try:
            screen = pygame.display.set_mode(
                size,
                flags,
                vsync=0,
            )
        except TypeError:
            screen = pygame.display.set_mode(
                size,
                flags,
            )

        pygame.display.set_caption('Mango Comet · ADC Prism')

        source = pygame.transform.smoothscale(
            pygame.image.load(config['image']).convert(),
            screen.get_size(),
        )

        render_mode = config.get('render_mode', 'flip')

        # ----------------------------------------------------
        # Fixed-focus display:
        # 렌즈 제어는 parent의 UART hold thread가 담당하고,
        # 이 child는 선택한 이미지를 분할 없이 전체 화면에 계속 표시한다.
        # ----------------------------------------------------
        if render_mode == 'fixed_image':
            screen.blit(source, (0, 0))
            pygame.display.flip()

            print(
                '[FIXED DISPLAY] 선택 이미지 전체 화면 표시 시작',
                flush=True,
            )

            start_wall = time.monotonic()
            while not stop.is_set():
                if (
                    config.get('seconds', 0)
                    and time.monotonic() - start_wall >= float(config['seconds'])
                ):
                    break

                for event in pygame.event.get():
                    if (
                        event.type == pygame.QUIT
                        or (
                            event.type == pygame.KEYDOWN
                            and event.key in (
                                pygame.K_ESCAPE,
                                pygame.K_q,
                            )
                        )
                    ):
                        stop.set()

                if not stop.is_set():
                    time.sleep(0.02)

            print('[FIXED DISPLAY] 종료', flush=True)
            return 0

        calibration = validate_calibration(config['calibration'])

        pictures = make_pictures(
            pygame,
            source,
            calibration,
        )

        if stop.is_set():
            return 0

        if config['gpio']:
            import lgpio
            gpio = lgpio

            chip_number = find_gpiochip()
            chip = gpio.gpiochip_open(chip_number)

            result = gpio.gpio_claim_output(
                chip,
                17,
                0,
            )

            if result < 0:
                raise RuntimeError('GPIO17을 확보하지 못했습니다.')

            claimed = True

        target_fps = float(config['fps'])
        gpio_phase_lead_ms = float(config['gpio_phase_lead_ms'])

        if target_fps <= 0:
            raise ValueError('fps는 0보다 커야 합니다.')

        if not 0.0 <= gpio_phase_lead_ms <= GPIO_PHASE_LEAD_MS_MAX:
            raise ValueError(
                'GPIO 선행 시간은 '
                f'0~{GPIO_PHASE_LEAD_MS_MAX:g} ms 범위여야 합니다.'
            )

        print(
            f'출력 시작: driver={pygame.display.get_driver()}, '
            f'size={screen.get_size()}, target={target_fps:.3f} Hz',
            flush=True,
        )
        print(
            '[TIMING] known-good mode: pygame Clock / '
            'KMSDRM / VSync OFF / GC OFF',
            flush=True,
        )
        print(
            '[TIMING] GPIO17을 다음 이미지 상태로 먼저 전환한 뒤 '
            f'{gpio_phase_lead_ms:.3f} ms 후 display.flip()을 실행합니다.',
            flush=True,
        )
        print(
            '[TIMING] 60 flip/s로 HIGH/LOW를 교대하면 '
            '각 초점면은 약 30 presentation/s 입니다.',
            flush=True,
        )
        print(
            f"HIGH: {calibration['high']} / LOW: {calibration['low']}",
            flush=True,
        )
        print(
            calibration_view(calibration)['label'],
            flush=True,
        )

        start_wall = time.monotonic()
        last_report = time.perf_counter()
        frame = 0
        clock = pygame.time.Clock()

        timing_window_size = max(
            30,
            int(round(target_fps * 2.0)) + 2,
        )

        flip_times = deque(maxlen=timing_window_size)
        intervals = deque(maxlen=timing_window_size)
        flip_block_samples = deque(maxlen=timing_window_size)
        gpio_lead_samples = deque(maxlen=timing_window_size)

        previous_flip_time = None
        gc_was_enabled = gc.isenabled()

        if gc_was_enabled:
            gc.disable()

        try:
            while not stop.is_set():
                now_wall = time.monotonic()

                if (
                    config['seconds']
                    and now_wall - start_wall >= config['seconds']
                ):
                    break

                for event in pygame.event.get():
                    if (
                        event.type == pygame.QUIT
                        or (
                            event.type == pygame.KEYDOWN
                            and event.key in (
                                pygame.K_ESCAPE,
                                pygame.K_q,
                            )
                        )
                    ):
                        stop.set()

                if stop.is_set():
                    break

                index = frame % 2

                # GPIO phase-lead 순서:
                # 1) GPIO17을 다음 image 상태로 먼저 변경
                # 2) 사용자가 설정한 선행 시간만큼 대기
                # 3) image blit + display.flip()
                # 4) Clock.tick(fps)
                if claimed:
                    gpio.gpio_write(
                        chip,
                        17,
                        1 if index == 0 else 0,
                    )

                gpio_done_time = time.perf_counter()
                wait_until(
                    gpio_done_time + gpio_phase_lead_ms / 1000.0
                )

                if stop.is_set():
                    break

                screen.blit(
                    pictures[index],
                    (0, 0),
                )

                flip_call_time = time.perf_counter()
                pygame.display.flip()
                flip_done_time = time.perf_counter()

                flip_block_samples.append(
                    (flip_done_time - flip_call_time) * 1000.0
                )

                gpio_lead_samples.append(
                    (flip_done_time - gpio_done_time) * 1000.0
                )

                flip_times.append(
                    flip_done_time
                )

                if previous_flip_time is not None:
                    intervals.append(
                        flip_done_time - previous_flip_time
                    )

                previous_flip_time = flip_done_time
                frame += 1

                # known-good pacing
                clock.tick(target_fps)

                now_perf = time.perf_counter()

                if now_perf - last_report >= TIMING_REPORT_INTERVAL:
                    last_report = now_perf

                    actual_hz = 0.0

                    if (
                        len(flip_times) >= 2
                        and flip_times[-1] > flip_times[0]
                    ):
                        actual_hz = (
                            (len(flip_times) - 1)
                            / (flip_times[-1] - flip_times[0])
                        )

                    avg_interval, jitter_s, min_interval, max_interval = (
                        window_stats(intervals)
                    )

                    avg_flip_block_ms, flip_block_jitter_ms, _, _ = (
                        window_stats(flip_block_samples)
                    )

                    avg_gpio_lead_ms, gpio_lead_jitter_ms, _, _ = (
                        window_stats(gpio_lead_samples)
                    )

                    metric = {
                        'target_hz': target_fps,
                        'actual_hz': actual_hz,
                        'per_plane_hz': actual_hz / 2.0,
                        'avg_frame_ms': avg_interval * 1000.0,
                        'jitter_ms': jitter_s * 1000.0,
                        'min_frame_ms': min_interval * 1000.0,
                        'max_frame_ms': max_interval * 1000.0,
                        'gpio_phase_lead_target_ms': gpio_phase_lead_ms,
                        'gpio_phase_lead_actual_ms': avg_gpio_lead_ms,
                        'gpio_phase_lead_jitter_ms': gpio_lead_jitter_ms,
                        'flip_block_ms': avg_flip_block_ms,
                        'flip_block_jitter_ms': flip_block_jitter_ms,
                        'timing_mode': 'friend_clock',
                        'vsync_requested': False,
                        'vsync_set_mode_ok': True,
                        'video_driver': pygame.display.get_driver(),
                        'frame_count': frame,
                    }

                    print(
                        '@@FLIP_METRIC@@'
                        + json.dumps(
                            metric,
                            separators=(',', ':'),
                        ),
                        flush=True,
                    )

                    print(
                        '[TIMING] '
                        f'flip={actual_hz:6.2f} Hz / target={target_fps:.2f} | '
                        f'frame={avg_interval * 1000.0:6.2f} ms | '
                        f'jitter={jitter_s * 1000.0:5.2f} ms | '
                        f'plane={actual_hz / 2.0:5.2f} Hz | '
                        f'flip block={avg_flip_block_ms:5.2f} ms | '
                        f'GPIO lead={avg_gpio_lead_ms:4.2f} ms '
                        f'(set {gpio_phase_lead_ms:.2f} ms)',
                        flush=True,
                    )

        finally:
            if gc_was_enabled and not gc.isenabled():
                gc.enable()

        print(
            '출력 종료',
            flush=True,
        )

        return 0

    except Exception as exc:
        print(
            '오류: ' + str(exc),
            flush=True,
        )
        return 1

    finally:
        if claimed:
            try:
                gpio.gpio_write(
                    chip,
                    17,
                    0,
                )
                gpio.gpio_free(
                    chip,
                    17,
                )
            except Exception as exc:
                print(
                    'GPIO 해제 오류:',
                    exc,
                    flush=True,
                )

        if chip is not None and chip >= 0:
            gpio.gpiochip_close(
                chip
            )

        if pygame is not None:
            pygame.quit()

        # KMSDRM/DRM 자원을 먼저 모두 반납한 뒤 GUI를 복구한다.
        restore_display_manager(
            dm_state
        )



# ============================================================
# Embedded autofocus source
# Source SHA256: 151ac43a2bb5c1b186ccd7334765c13d0792dce28e41087f40526a323cb2b3d3
# Integrated autofocus source; exhibition variant changes its overlay label only.
# ============================================================
_EMBEDDED_AUTOFOCUS_B64 = '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'
_EMBEDDED_AUTOFOCUS_SHA256 = '2c2188e148b7374d42c177bedc501ab1cee58d854147b099ac36060fb13c7504'


def _embedded_autofocus_source():
    raw = zlib.decompress(base64.b64decode(_EMBEDDED_AUTOFOCUS_B64)).decode('utf-8')
    import hashlib
    actual = hashlib.sha256(raw.encode('utf-8')).hexdigest()
    if actual != _EMBEDDED_AUTOFOCUS_SHA256:
        raise RuntimeError('내장 자동초점 코드 무결성 확인 실패')
    return raw


def _autofocus_source_for_split_calibration(source, mode):
    """
    Keep the ORIGINAL calibration algorithms intact and only split where
    execution stops/starts.

    mode == 'lens_only'
      - PHASE -1 / PHASE 0 run normally
      - PHASE 1 Camera Lens calibration runs ORIGINAL code unchanged
      - ORIGINAL CSV save runs
      - stop before PHASE 2

    mode == 'sweep_only'
      - PHASE -1 / PHASE 0 run normally
      - PHASE 1 lens sweep is skipped
      - saved Q1~Q4 LensPosition values are loaded
      - PHASE 2 Projector ADC sweep and all following analysis run normally
    """

    phase1_block = """    # --------------------------------------------------------
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
"""

    if phase1_block not in source:
        raise RuntimeError(
            '내장 자동초점 코드의 원본 PHASE 1 블록을 찾지 못했습니다.'
        )

    if mode == 'lens_only':
        # Do not change one line of the actual camera-lens algorithm.
        # Simply stop after the original save_camera_calibration_csv().
        replacement = phase1_block + """
    print()
    print("============================================")
    print(" MANGO LENS CALIBRATION ONLY COMPLETE")
    print("============================================")
    print(
        "Camera Lens Q1~Q4 calibration 저장 완료. "
        "Projector ADC sweep은 실행하지 않습니다."
    )
    raise SystemExit(0)
"""

    elif mode == 'sweep_only':
        replacement = """    # --------------------------------------------------------
    # PHASE 1 : SKIPPED FOR MANGO PROJECTOR-SWEEP MODE
    # --------------------------------------------------------
    print()
    print("============================================")
    print(" PHASE 1 : CAMERA LENS CALIBRATION SKIPPED")
    print("============================================")
    print(
        "저장된 Q1~Q4 Camera LensPosition을 사용하고 "
        "PHASE 2 Projector ADC sweep부터 진행합니다."
    )

    _mango_saved_lens_json = os.environ.get(
        "MANGO_SAVED_LENS_JSON",
        "",
    ).strip()

    if not _mango_saved_lens_json:
        raise RuntimeError(
            "저장된 Camera Lens 값 snapshot이 전달되지 않았습니다."
        )

    try:
        _mango_saved_lens = json.loads(
            _mango_saved_lens_json
        )
    except Exception as exc:
        raise RuntimeError(
            "저장 Camera Lens JSON을 읽지 못했습니다: "
            + str(exc)
        )

    selected_camera_lens = {}
    all_calibration_runs = []

    for q in QUADRANT_ORDER:
        if q not in _mango_saved_lens:
            raise RuntimeError(
                "저장 Camera Lens 값에 " + q + "가 없습니다."
            )

        item = _mango_saved_lens[q]

        try:
            lens_position = float(
                item["LensPosition"]
            )
            reference_adc = int(
                round(float(item["ReferenceADC"]))
            )
            metric_name = str(
                item.get("Metric")
                or "REUSED_SAVED"
            )
        except Exception:
            raise RuntimeError(
                q + " 저장 Camera Lens 값 형식이 잘못되었습니다."
            )

        if not np.isfinite(lens_position):
            raise RuntimeError(
                q + " 저장 Camera Lens 값이 유효하지 않습니다."
            )

        if not (
            CAMERA_LENS_MIN
            <= lens_position
            <= CAMERA_LENS_MAX
        ):
            raise RuntimeError(
                f"{q} 저장 LensPosition "
                f"{lens_position:.4f} D가 "
                f"현재 허용 범위 "
                f"{CAMERA_LENS_MIN:.4f}~"
                f"{CAMERA_LENS_MAX:.4f} D를 "
                "벗어났습니다."
            )

        selected_camera_lens[q] = {
            "LensPosition": lens_position,
            "ReferenceADC": reference_adc,
            "Metric": metric_name,
        }

        print(
            f"{q}: LensPosition={lens_position:.4f} D | "
            f"saved refADC={reference_adc} | "
            f"metric={metric_name}"
        )

    print()
    print(
        "저장 렌즈값 로드 완료 -> "
        "PHASE 2 Projector VCM sweep 시작"
    )
"""

    else:
        raise RuntimeError(
            '알 수 없는 calibration mode: ' + str(mode)
        )

    patched = source.replace(
        phase1_block,
        replacement,
        1,
    )

    compile(
        patched,
        '<mango-split-calibration>',
        'exec',
    )

    return patched


def run_integrated_calibration(preview_path):
    """Run the embedded autofocus code as the camera/UART/GPIO22 owner."""
    calibration_mode = os.environ.get(
        'MANGO_CALIBRATION_MODE',
        'lens_only',
    ).strip()

    if calibration_mode not in {
        'lens_only',
        'sweep_only',
    }:
        raise RuntimeError(
            '지원하지 않는 calibration mode: '
            + calibration_mode
        )

    import cv2
    preview = Path(preview_path)
    preview.parent.mkdir(parents=True, exist_ok=True)
    start_key_sent = False
    completed = False
    last_publish = 0.0
    publish_lock = threading.Lock()
    original_print = builtins.print
    original_cv = {
        'imshow': cv2.imshow, 'waitKey': cv2.waitKey, 'namedWindow': cv2.namedWindow,
        'moveWindow': cv2.moveWindow, 'resizeWindow': getattr(cv2, 'resizeWindow', None),
        'destroyWindow': getattr(cv2, 'destroyWindow', None),
        'destroyAllWindows': cv2.destroyAllWindows,
    }

    def atomic_jpeg_write(frame):
        nonlocal last_publish
        now = time.monotonic()
        if now - last_publish < 0.10:
            return
        with publish_lock:
            now = time.monotonic()
            if now - last_publish < 0.10:
                return
            ok, data = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if not ok:
                return
            tmp = Path(str(preview) + '.tmp')
            tmp.write_bytes(data.tobytes())
            os.replace(tmp, preview)
            last_publish = now

    def web_imshow(_name, frame):
        try:
            atomic_jpeg_write(frame)
        except Exception:
            pass

    def web_print(*args, **kwargs):
        nonlocal completed
        text_value = ' '.join(str(item) for item in args)
        original_print(*args, **kwargs)
        if '측정 완료.' in text_value:
            completed = True

    def web_wait_key(delay=1):
        nonlocal start_key_sent
        try:
            delay_value = int(delay)
        except Exception:
            delay_value = 1
        if not start_key_sent and 20 <= delay_value <= 60:
            start_key_sent = True
            return ord('s')
        if completed:
            return ord('q')
        if delay_value > 0:
            time.sleep(min(delay_value / 1000.0, 0.01))
        return -1

    builtins.print = web_print
    cv2.imshow = web_imshow
    cv2.waitKey = web_wait_key
    cv2.namedWindow = lambda *a, **k: None
    cv2.moveWindow = lambda *a, **k: None
    if hasattr(cv2, 'resizeWindow'):
        cv2.resizeWindow = lambda *a, **k: None
    if hasattr(cv2, 'destroyWindow'):
        cv2.destroyWindow = lambda *a, **k: None
    cv2.destroyAllWindows = lambda *a, **k: None

    original_print('[WEB-CAL] integrated autofocus SHA256=' + _EMBEDDED_AUTOFOCUS_SHA256, flush=True)
    original_print('[WEB-CAL] preview=' + str(preview), flush=True)
    namespace = {'__name__': '__main__', '__file__': str(Path(__file__).resolve()), '__builtins__': __builtins__}
    try:
        try:
            source = _embedded_autofocus_source()
            source_name = '<embedded-auto-focus-fastsettle>'
        except Exception as exc:
            original_print(
                f'[WEB-CAL] embedded code unavailable ({exc}); '
                f'running {CALIBRATION_FALLBACK_SCRIPT}',
                flush=True,
            )
            source = Path(CALIBRATION_FALLBACK_SCRIPT).read_text(encoding='utf-8')
            source_name = CALIBRATION_FALLBACK_SCRIPT
        source = source.replace('CLAHE + GAUSSIAN', 'CLAHE + LAPLACIAN')
        source = _autofocus_source_for_split_calibration(
            source,
            calibration_mode,
        )
        source_name = (
            '<embedded-auto-focus-lens-only>'
            if calibration_mode == 'lens_only'
            else '<embedded-auto-focus-projector-sweep-only>'
        )

        original_print(
            '[WEB-CAL] split mode=' + calibration_mode,
            flush=True,
        )

        exec(
            compile(source, source_name, 'exec'),
            namespace,
            namespace,
        )
        return 0

    except SystemExit as exc:
        # lens_only intentionally exits immediately after the ORIGINAL
        # PHASE 1 CSV save. Treat SystemExit(0) as normal completion.
        if exc.code in (None, 0):
            return 0
        raise

    except KeyboardInterrupt:
        return 0
    finally:
        builtins.print = original_print
        for name, value in original_cv.items():
            if value is not None:
                try:
                    setattr(cv2, name, value)
                except Exception:
                    pass


PAGE = r'''<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Mango Comet · 전시 제어</title>
<style>
*{box-sizing:border-box}body{max-width:1760px;margin:20px auto;padding:0 28px;background:#15202c;color:#edf3f7;font:15px/1.6 system-ui}
h1{font-size:27px;margin-bottom:4px}h2{font-size:18px;margin-top:0}p{margin:8px 0 16px}.hint{color:#bac8d5;font-size:13px}
section{padding:22px;margin:20px 0;background:#1e2d3c;border:1px solid #34495b;border-radius:12px}
fieldset{border:0;padding:0;margin:0;min-width:0}.row{display:flex;gap:12px;align-items:end;flex-wrap:wrap}.grow{flex:1;min-width:220px}
input[type=text],input[type=number],select{display:block;width:100%;padding:10px;margin:5px 0;background:#111d28;color:inherit;border:1px solid #5e7182;border-radius:6px}
input[type=file]{display:block;margin:10px 0 18px;max-width:100%}button{padding:11px 17px;border:0;border-radius:6px;cursor:pointer;background:#b5e4dd;color:#142728;font-weight:600}
button:disabled{opacity:.45;cursor:default}#stop{background:#f4b8ac}.checks{margin:18px 0}.checks label{margin-right:20px}
.previews{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:14px}figure{margin:0}canvas{width:100%;height:auto;background:#000;display:block;border:1px solid #5e7182;border-radius:5px}figcaption{margin:6px 0;color:#d5e1eb}
.high{color:#ffca85}.low{color:#8ddcfa}#calCam{width:100%;max-width:900px;aspect-ratio:16/9;object-fit:contain;background:#000;border:1px solid #5e7182;border-radius:6px}#calRunLog{white-space:pre-wrap;background:#101922;padding:15px;max-height:260px;overflow:auto}#calibration{white-space:pre-line}#message{color:#ffdf91;white-space:pre-line;overflow-wrap:anywhere}#log{white-space:pre-wrap;background:#101922;padding:15px;max-height:220px;overflow:auto}#selection{overflow-wrap:anywhere}
@media(max-width:680px){.previews{grid-template-columns:1fr}body{padding:0 12px}section{padding:16px}}
#quick{border:2px solid #b5e4dd;min-height:calc(100svh - 110px);display:flex;flex-direction:column;gap:18px;padding:clamp(20px,3vw,40px)}
#quick h2{font-size:24px;margin:0}.bigrow{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:16px;flex:1}.focusrow{grid-template-columns:repeat(2,minmax(0,1fr))}
button.big{word-break:keep-all;font-size:clamp(20px,2vw,30px);padding:26px 10px;min-height:104px;border-radius:14px;width:100%}
button.big.focus{background:#a5cbed}button.big.release{background:#d7deea}
#quick .qstate{margin:0}#quick p{margin:0}#qInfo{font-size:15px}
/* UI previews only: projector image bytes and slice mapping are unchanged. */
#whole,#high,#low{transform:scaleX(-1)}
.effects{margin-top:20px}.effects button[aria-pressed="true"]{background:#ffca85;box-shadow:0 0 0 2px #ffca85}
@media(max-width:900px){.bigrow{grid-template-columns:repeat(2,minmax(0,1fr))}.focusrow{grid-template-columns:repeat(2,minmax(0,1fr))}button.big{font-size:20px}}
@media(max-width:520px){ #quick{min-height:0;padding:16px}.focusrow{grid-template-columns:1fr}button.big{min-height:76px;font-size:18px}.bigrow{gap:10px}}
button.big.red{background:#f4b8ac}button.big.cal{background:#ffd98a}button.big.flip{background:#b5e4dd}
.qstate{font-size:20px;font-weight:700;margin:16px 0 6px}#qInfo{white-space:pre-line}#qMsg{color:#ffdf91;white-space:pre-line}
details#advanced>summary{cursor:pointer;font-size:17px;font-weight:600;margin:18px 0}
</style></head><body>
<h1>Mango Comet · 다중 초점 프로젝터</h1>
<section id="quick"><h2>빠른 제어</h2>
<div class="bigrow">
<button id="qStop" class="big red" type="button">■ 정지</button>
<button id="qSweepCal" class="big cal" type="button">⌁ 스윕 캘리브레이션</button>
<button id="qFlip" class="big flip" type="button">▶ 플립</button>
</div>
<div class="bigrow focusrow">
<button id="qFrontFixed" class="big focus" type="button" disabled>앞 고정</button>
<button id="qBackFixed" class="big focus" type="button" disabled>뒤 고정</button>
</div>
<p id="qState" class="qstate" role="status" aria-live="polite">대기</p>
<p id="qInfo" class="hint"></p>
<p id="qMsg"></p>
</section>
<details id="advanced" open><summary>세부 제어 · 이미지 · 캘리브레이션 · 출력 설정</summary>
<p class="hint">이미지 한 장을 중앙 수직선 기준 좌/우로만 이등분하고 HIGH / LOW 영역을 교대로 표시합니다.</p>
<section><h2>0. 캘리브레이션</h2>
<p class="hint">
<b>렌즈 캘리브레이션</b>: 카메라의 각 사분면에 맞는 렌즈 위치를 측정하고 저장합니다.<br>
<b>스윕 캘리브레이션</b>: 저장된 카메라 렌즈값으로 프로젝터의 앞·뒤 초점 위치를 측정합니다.
</p>
<div class="row">
<button id="calLensStart" type="button">렌즈 캘리브레이션</button>
<button id="calSweepStart" type="button">프로젝터 스윕 캘리브레이션</button>
<button id="calStop" type="button" disabled>캘리브레이션 중지</button>
</div>
<p id="savedLensInfo" class="hint">저장된 렌즈값 확인 중…</p>
<p id="calRunState" class="hint">대기</p>
<img id="calCam" alt="캘리브레이션 카메라 프리뷰">
<pre id="calRunLog"></pre>
</section>
<section><h2>1. 이미지 불러오기</h2><fieldset id="sources">
<label for="upload">노트북에서 업로드</label>
<input id="upload" type="file" accept=".png,.jpg,.jpeg,.bmp,.webp">
<p class="hint">이미지를 불러온 뒤 네거티브와 강한 대비 효과를 선택할 수 있습니다. 미리보기는 관람 방향에 맞춰 표시합니다.</p>
<div class="row"><label class="grow" for="directory">라즈베리파이 이미지 폴더
<input id="directory" type="text" value="{{ image_dir }}"></label><button id="list" type="button">목록 보기</button></div>
<div class="row"><select id="files" class="grow" aria-label="라즈베리파이 이미지 선택"></select>
<button id="load" type="button">선택한 이미지 불러오기</button></div>
<div class="row effects">
<button id="negative" type="button" aria-pressed="false">네거티브: 꺼짐</button>
<button id="strongContrast" type="button" aria-pressed="false">강한 대비 ×4: 꺼짐</button>
<button id="resetEffects" type="button">효과 초기화</button>
</div>
<p id="effectInfo" class="hint" role="status">이미지를 불러오면 효과를 사용할 수 있습니다.</p>
<p class="hint">효과는 미리보기와 프로젝터 출력에 함께 적용됩니다. 실행 중에는 정지 또는 해제 후 변경하세요. 원본 파일은 보존됩니다.</p>
</fieldset><p id="selection" class="hint">이미지를 선택하세요.</p></section>
<section><h2>2. 캘리브레이션과 분할 미리보기</h2>
<p class="hint">이미지의 좌우 영역을 앞·뒤 초점에 나누어 표시합니다. 아래 미리보기는 관람 방향을 기준으로 하며, Q 번호는 카메라 캘리브레이션 기준입니다.</p>
<p id="calibration">캘리브레이션 결과 확인 중…</p><p id="next" class="hint"></p>
<div class="previews">
<figure><canvas id="whole" width="640" height="360"></canvas><figcaption>전체 이미지 · 중앙 수직 분할선</figcaption></figure>
<figure><canvas id="high" width="640" height="360"></canvas><figcaption class="high">앞 초점 · FRONT / HIGH ADC</figcaption></figure>
<figure><canvas id="low" width="640" height="360"></canvas><figcaption class="low">뒤 초점 · BACK / LOW ADC</figcaption></figure>
</div><p class="hint">각 영역은 원래 위치를 유지합니다. 반대 영역은 검게 표시됩니다.</p></section>
<section><h2>3. 플립 실행</h2><form id="form"><fieldset id="settings">
<div class="row"><label class="grow">목표 이미지 플립 속도 (Hz)
<input id="fps" type="number" min="1" max="240" step="any" value="60" required></label>
<label class="grow">GPIO17 선행 시간 (ms)
<input id="gpioPhaseLeadMs" type="number" min="0" max="{{ gpio_phase_lead_ms_max }}" step="0.1" value="{{ gpio_phase_lead_ms_default }}" required></label>
<label class="grow">뒤 초점 전환 보정 (ADC)
<input id="backFakeTargetOffset" type="number" min="0" max="{{ back_fake_target_offset_max }}" step="0.5" value="{{ back_fake_target_offset_default }}" required></label>
<label class="grow">실행 시간 (초, 0이면 계속)
<input id="seconds" type="number" min="0" max="86400" step="any" value="10" required></label></div>
<div class="checks"><label><input id="fullscreen" type="checkbox">전체화면</label>
<span><b>GPIO17 동기 신호: 항상 ON</b></span></div></fieldset>
<button id="start" disabled>플립 시작</button><button id="stop" type="button" disabled>이 앱의 출력 중지</button></form>
<p class="hint">앞·뒤 초점을 교대로 전환합니다. 선행 시간과 뒤 초점 보정값으로 전환 시점을 조절할 수 있습니다. 뒤 초점 보정은 교대 출력에만 적용됩니다.</p>
<p id="status">대기</p>
<div id="timingBox" style="margin:12px 0;padding:12px 14px;background:#111d28;border:1px solid #5e7182;border-radius:7px">
<b>실시간 출력 성능</b>
<div id="timing" class="hint" style="white-space:pre-line;margin-top:5px">출력 시작 후 약 1초 뒤 표시됩니다.</div>
</div>
<p id="message" role="status"></p><pre id="log"></pre></section>

<section><h2>4. 초점 고정</h2>
<p class="hint">
캘리브레이션에서 찾은 정확한 ADC를 사용합니다.
앞 고정은 HIGH ADC, 뒤 고정은 LOW ADC에 렌즈를 고정하고,
선택한 이미지는 분할하지 않고 전체 화면으로 그대로 띄웁니다.
</p>
<div class="row">
<button id="frontFixed" type="button">앞(FRONT · HIGH ADC) 고정 + 이미지 표시</button>
<button id="backFixed" type="button">뒤(BACK · LOW ADC) 고정 + 이미지 표시</button>
<button id="focusRelease" type="button" disabled>고정/화면 해제</button>
</div>
<p id="fixedStatus" class="hint">대기</p>
</section>
</details>

<script>
const $=id=>document.getElementById(id), token={{ token | tojson }};
let pending=false, refreshing=false, state=null, imageKey='', loadedImage=null;
async function call(path,data){
  const options=data===undefined?{}:{method:'POST',headers:{'X-Mango-Token':token}};
  if(data!==undefined){
    if(data instanceof FormData) options.body=data;
    else {options.headers['Content-Type']='application/json';options.body=JSON.stringify(data);}
  }
  const response=await fetch(path,options), value=await response.json();
  if(!response.ok) throw Error(value.error||'요청 오류');
  return value;
}
function controls(){
  const running=Boolean(state?.running);
  const calibrating=Boolean(state?.calibration_worker?.running);
  const holding=Boolean(state?.hold?.running);
  const disabled=running||calibrating||holding||pending;
  $('sources').disabled=disabled;$('settings').disabled=disabled;
  $('start').disabled=disabled||!state?.image||!state?.calibration;
  $('stop').disabled=!running||pending;
  const savedLensReady=Boolean(state?.saved_camera_lens?.available);
  $('calLensStart').disabled=pending||running||calibrating||holding;
  $('calSweepStart').disabled=pending||running||calibrating||holding||!savedLensReady;
  $('calStop').disabled=pending||!calibrating;
  const readyFixed=Boolean(state?.image&&state?.calibration);
  $('frontFixed').disabled=pending||running||calibrating||holding||!readyFixed;
  $('backFixed').disabled=pending||running||calibrating||holding||!readyFixed;
  $('focusRelease').disabled=pending||calibrating||(!running&&!holding);
  $('qFrontFixed').disabled=$('frontFixed').disabled;
  $('qBackFixed').disabled=$('backFixed').disabled;
  const effects=state?.image?.effects||{};
  for(const [id,key,label] of [['negative','negative','네거티브'],['strongContrast','strong_contrast','강한 대비 ×4']]){
    $(id).disabled=disabled||!state?.image;
    $(id).setAttribute('aria-pressed',String(Boolean(effects[key])));
    $(id).textContent=label+': '+(effects[key]?'켜짐':'꺼짐');
  }
  $('resetEffects').disabled=disabled||!state?.image||!(effects.negative||effects.strong_contrast);
  $('effectInfo').textContent=!state?.image?'이미지를 불러오면 효과를 사용할 수 있습니다.':
    '적용 효과: '+([effects.negative?'네거티브':'',effects.strong_contrast?'강한 대비 ×4':''].filter(Boolean).join(' + ')||'없음');
  if(state) quickRender();
}
function draw(){
  const cal=state?.running?state.active_calibration:state?.calibration;
  const canvases=['whole','high','low'].map(id=>$(id));
  const [whole,high,low]=canvases.map(c=>c.getContext('2d'));
  const w=640,h=360;
  for(const context of [whole,high,low]){context.fillStyle='#000';context.fillRect(0,0,w,h);}
  if(!loadedImage) return;
  whole.drawImage(loadedImage,0,0,w,h);
  if(!cal) return;
  high.drawImage(loadedImage,0,0,w,h);low.drawImage(loadedImage,0,0,w,h);
  cal.rows.forEach(([left,right],y)=>{
    high.fillRect(0,y,left,1);high.fillRect(right,y,w-right,1);
    low.fillRect(left,y,right-left,1);
  });
  const line=[w/2,0,w/2,h];
  whole.save();whole.strokeStyle='#b5e4dd';whole.lineWidth=2;whole.setLineDash([8,6]);
  whole.beginPath();whole.moveTo(line[0],line[1]);whole.lineTo(line[2],line[3]);whole.stroke();whole.restore();
}
function render(){
  if(!state) return;
  const cw=state.calibration_worker||{};
  const modeLabel=
    cw.mode==='lens_only'?'렌즈 캘리':
    cw.mode==='sweep_only'?'프로젝터 스윕 캘리':'';
  $('calRunState').textContent=cw.running?
    `캘리브레이션 실행 중 · ${modeLabel}`:(cw.last_result||'대기');
  $('calRunLog').textContent=cw.log||'';

  const savedLens=state.saved_camera_lens||{};
  if(savedLens.available){
    const v=savedLens.values||{};
    $('savedLensInfo').textContent=
      '저장 렌즈값 사용 가능 · '+
      ['Q1','Q2','Q3','Q4'].map(q=>
        `${q} ${Number(v[q]?.LensPosition).toFixed(3)} D`
      ).join(' / ');
  }else{
    $('savedLensInfo').textContent=
      '저장 렌즈값 없음 · 먼저 렌즈 캘리브레이션을 실행하세요. '+
      (savedLens.error||'');
  }
  if(cw.preview_available||cw.running){$('calCam').src='/calibration/preview?t='+Date.now();}
  const cal=state.running?state.active_calibration:state.calibration;
  const mode=state.output_mode||'idle';
  $('status').textContent=state.running?
    (mode==='front_fixed'?'앞(FRONT) 고정 + 전체 이미지 출력 중':
     mode==='back_fixed'?'뒤(BACK) 고정 + 전체 이미지 출력 중':
     '교대 플립 출력 중'):
    state.code?'출력 오류 — 아래 기록을 확인하세요.':'대기';
  $('log').textContent=state.log;

  const timing=state.flip_metrics||{};
  if(state.running&&timing.actual_hz!==undefined){
    const delta=Math.abs(Number(timing.actual_hz)-Number(timing.target_hz));
    const verdict=delta<=0.5?'정상':delta<=1.5?'주의':'불안정';
    $('timing').textContent=
      `실측 플립: ${Number(timing.actual_hz).toFixed(2)} Hz / 목표 ${Number(timing.target_hz).toFixed(2)} Hz · ${verdict}\n`+
      `평균 프레임: ${Number(timing.avg_frame_ms).toFixed(2)} ms · jitter σ=${Number(timing.jitter_ms).toFixed(2)} ms`+
      ` · min/max=${Number(timing.min_frame_ms).toFixed(2)}/${Number(timing.max_frame_ms).toFixed(2)} ms\n`+
      `각 초점면 표시율: 약 ${Number(timing.per_plane_hz).toFixed(2)} Hz (HIGH/LOW 교대이므로 플립 Hz의 절반)\n`+
      `flip block=${Number(timing.flip_block_ms||0).toFixed(2)} ms · GPIO lead=${Number(timing.gpio_phase_lead_actual_ms||0).toFixed(2)} ms (설정 ${Number(timing.gpio_phase_lead_target_ms||0).toFixed(2)} ms)\n`+
      `출력 backend: ${timing.video_driver||'unknown'} · timing=${timing.timing_mode||'unknown'} · VSync=OFF`;
  }else if(state.running){
    $('timing').textContent='측정 중… 약 1초 뒤 실제 플립 속도가 표시됩니다.';
  }else if(timing.actual_hz!==undefined){
    $('timing').textContent=
      `마지막 실측: ${Number(timing.actual_hz).toFixed(2)} Hz / 목표 ${Number(timing.target_hz).toFixed(2)} Hz`+
      ` · jitter σ=${Number(timing.jitter_ms).toFixed(2)} ms`;
  }else{
    $('timing').textContent='출력 시작 후 약 1초 뒤 표시됩니다.';
  }
  $('calibration').textContent=cal?
    `HIGH / 앞(FRONT): ${cal.high_side==='left'?'오른쪽':'왼쪽'} 슬라이스 · Q${cal.high.quadrant} · ADC ${cal.high.adc}\n`+
    `LOW / 뒤(BACK): ${cal.low_side==='left'?'오른쪽':'왼쪽'} 슬라이스 · Q${cal.low.quadrant} · ADC ${cal.low.adc}\n`+
    `분할: ${cal.label}`:state.calibration_error;
  const changed=state.running&&JSON.stringify(state.calibration)!==JSON.stringify(state.active_calibration);
  $('next').textContent=state.running&&!state.calibration?
    `현재 출력은 시작할 때의 값을 유지합니다. ${state.calibration_error}`:
    changed?'현재 출력은 시작할 때의 값을 유지합니다. 새 캘리브레이션 결과는 다음 시작에 적용됩니다.':'';
  $('selection').textContent=state.image?
    `${state.image.name} · ${state.image.width} × ${state.image.height}`+
    (state.image.effects?.negative?' · 네거티브':'')+
    (state.image.effects?.strong_contrast?' · 강한 대비 ×4':''):
    '이미지를 선택하세요.';
  const hold=state.hold||{};
  if(hold.running){
    const info=hold.info||{};
    $('fixedStatus').textContent=
      `${info.label||'초점'} 고정 중 · Target ADC ${hold.adc}`+
      (info.quadrant!==undefined?` · Q${info.quadrant}`:'')+
      (hold.last_line?`
ATmega: ${hold.last_line}`:'');
  }else if(hold.error){
    $('fixedStatus').textContent='오류: '+hold.error;
  }else if(hold.info){
    $('fixedStatus').textContent=`마지막 고정: ${hold.info.label||''} · ADC ${hold.info.adc}`;
  }else{
    $('fixedStatus').textContent='대기';
  }
  const key=state.image?.id||'';
  if(key!==imageKey){
    imageKey=key;loadedImage=null;
    if(key){
      const picture=new Image();
      picture.onload=()=>{if(imageKey===key){loadedImage=picture;draw();}};
      picture.onerror=()=>{if(imageKey===key)$('message').textContent='이미지 미리보기를 불러오지 못했습니다.';};
      picture.src='/image/'+encodeURIComponent(key);
    }
  }
  draw();controls();quickRender();
}
function quickRender(){
  const cw=state.calibration_worker||{};
  const cal=state.calibration;
  const calibrating=Boolean(cw.running);
  const running=Boolean(state.running);
  const holding=Boolean(state.hold&&state.hold.running);
  const mode=state.output_mode||'idle';
  let text='대기';
  if(calibrating) text='캘리브레이션 중 · '+(cw.last_result||'');
  else if(running&&mode==='flip') text='플립 출력 중';
  else if(running&&mode==='front_fixed') text='앞 초점 고정 중';
  else if(running&&mode==='back_fixed') text='뒤 초점 고정 중';
  else if(running) text='화면 출력 중';
  else if(holding) text='초점 고정 중';
  else if(cw.last_result) text=cw.last_result;
  $('qState').textContent=text;
  const info=[];
  info.push(cal?
    `저장된 초점 ADC · HIGH ${Number(cal.high.adc).toFixed(1)} (Q${cal.high.quadrant}) / LOW ${Number(cal.low.adc).toFixed(1)} (Q${cal.low.quadrant})`:
    '저장된 초점값 없음 · 캘리브레이션을 먼저 실행하세요');
  const savedLens=state.saved_camera_lens||{};
  if(savedLens.available){
    const v=savedLens.values||{};
    info.push(
      '저장 Camera Lens · '+
      ['Q1','Q2','Q3','Q4'].map(q=>
        `${q} ${Number(v[q]?.LensPosition).toFixed(3)}D`
      ).join(' / ')
    );
  }else{
    info.push('저장 Camera Lens 없음 · 렌즈 캘리를 먼저 실행하세요');
  }
  info.push(state.image?`플립 이미지: ${state.image.name}`:'플립 이미지 없음 · 아래 이미지 불러오기에서 선택하세요');
  $('qInfo').textContent=info.join('\n');
  const busy=pending||calibrating||running||holding;
  $('qSweepCal').disabled=busy||!savedLens.available;
  $('qFlip').disabled=busy||!cal||!state.image;
  $('qStop').disabled=pending;
}
async function refresh(){
  if(refreshing) return;
  refreshing=true;
  try{state=await call('/status');render();}catch(error){$('message').textContent=error.message;}
  finally{refreshing=false;}
}
async function action(work){
  if(pending) return;
  pending=true;controls();$('message').textContent='';$('qMsg').textContent='';
  try{await work();}catch(error){$('message').textContent=error.message;$('qMsg').textContent=error.message;}
  finally{pending=false;await refresh();controls();}
}
async function listImages(){
  const result=await call('/images?directory='+encodeURIComponent($('directory').value));
  $('files').replaceChildren();
  for(const file of result.files){const option=document.createElement('option');option.value=file.path;option.textContent=file.name;$('files').append(option);}
  if(!result.files.length)$('message').textContent='이 폴더에 지원하는 이미지가 없습니다.';
}
$('list').onclick=()=>action(listImages);
$('load').onclick=()=>action(async()=>{
  if(!$('files').value)throw Error('먼저 폴더 목록에서 이미지를 선택하세요.');
  await call('/select',{path:$('files').value});
});
$('upload').onchange=()=>action(async()=>{
  const file=$('upload').files[0];if(!file)return;
  const form=new FormData();form.append('image',file);await call('/upload',form);$('upload').value='';
});
$('form').onsubmit=event=>{
  event.preventDefault();action(async()=>{
    const result=await call('/start',{
      image_id:state.image.id,
      fps:Number($('fps').value),
      gpio_phase_lead_ms:Number($('gpioPhaseLeadMs').value),
      back_fake_target_offset:Number($('backFakeTargetOffset').value),
      seconds:Number($('seconds').value),
      fullscreen:$('fullscreen').checked
    });
    const u=result.uart_targets||{};
    $('message').textContent=
      `플립 시작 · 진짜 BACK ${Number(u.back_real_adc).toFixed(1)} → `+
      `명령 BACK ${Number(u.back_command_adc).toFixed(1)} ADC `+
      `(보정 ${Number(u.back_fake_target_offset).toFixed(1)})`;
  });
};
$('stop').onclick=()=>action(async()=>{await call('/stop',{});$('message').textContent='이 앱의 출력을 중지했습니다.';});
$('frontFixed').onclick=()=>action(async()=>{
  const result=await call('/focus-fixed/start',{
    plane:'front',
    image_id:state.image.id
  });
  $('message').textContent=
    `앞(FRONT) 고정 시작 · ADC ${result.adc}\n선택한 전체 이미지를 함께 표시합니다.`;
});
$('backFixed').onclick=()=>action(async()=>{
  const result=await call('/focus-fixed/start',{
    plane:'back',
    image_id:state.image.id
  });
  $('message').textContent=
    `뒤(BACK) 고정 시작 · ADC ${result.adc}\n선택한 전체 이미지를 함께 표시합니다.`;
});
$('focusRelease').onclick=()=>action(async()=>{
  await call('/stop',{});
  $('message').textContent='초점 고정과 화면 출력을 함께 해제했습니다.';
});
// Reuse the same fixed-focus endpoints and guards in both control areas.
$('qFrontFixed').onclick=()=>$('frontFixed').click();
$('qBackFixed').onclick=()=>$('backFixed').click();
async function setEffects(negative,strong_contrast){
  await call('/image-effects',{image_id:state.image.id,negative,strong_contrast});
}
$('negative').onclick=()=>action(()=>setEffects(!state.image.effects?.negative,Boolean(state.image.effects?.strong_contrast)));
$('strongContrast').onclick=()=>action(()=>setEffects(Boolean(state.image.effects?.negative),!state.image.effects?.strong_contrast));
$('resetEffects').onclick=()=>action(()=>setEffects(false,false));
$('calLensStart').onclick=()=>action(async()=>{
  await call('/calibration/lens/start',{});
  $('message').textContent=
    '렌즈 캘리브레이션을 시작했습니다.';
});
$('calSweepStart').onclick=()=>action(async()=>{
  await call('/calibration/sweep/start',{});
  $('message').textContent=
    '프로젝터 스윕 캘리 시작 · 저장된 Q1~Q4 LensPosition을 사용합니다.';
});
$('calStop').onclick=()=>action(async()=>{
  await call('/calibration/stop',{});
  $('message').textContent='캘리브레이션 중지 요청을 보냈습니다.';
});
$('qStop').onclick=()=>action(async()=>{
  const result=await call('/all-stop',{});
  $('qMsg').textContent='전체 정지 · ATmega: '+(result.uart||'');
});
$('qSweepCal').onclick=()=>action(async()=>{
  await call('/calibration/sweep/start',{});
  $('qMsg').textContent=
    '스윕 캘리 시작 · 저장 렌즈값으로 Projector ADC sweep을 수행합니다.';
});
$('qFlip').onclick=()=>action(async()=>{
  if(!state.image) throw Error('플립 이미지를 먼저 선택하세요.');
  await call('/start',{image_id:state.image.id,fps:{{ quick_flip_fps }},
    gpio_phase_lead_ms:Number($('gpioPhaseLeadMs').value),
    back_fake_target_offset:Number($('backFakeTargetOffset').value),
    seconds:0,fullscreen:true});
  $('qMsg').textContent='플립 시작 · 정지 버튼을 누를 때까지 계속';
});
refresh();action(listImages);setInterval(refresh,500);
</script></body></html>'''



def stop_display_managers_for_projection():
    """
    프로젝터 직접 출력 전에 GUI를 중지한다.
    Flask/Mango 웹서버는 SSH에서 계속 살아 있고,
    그 다음 KMSDRM child가 DRM을 직접 사용한다.
    """
    results = []

    for service in ('lightdm', 'display-manager'):
        command = [
            'sudo', '-n', 'systemctl', 'stop', service
        ]

        try:
            completed = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=False,
                timeout=8,
            )

            result = {
                'service': service,
                'returncode': completed.returncode,
                'stdout': completed.stdout.strip(),
                'stderr': completed.stderr.strip(),
            }

        except Exception as exc:
            result = {
                'service': service,
                'returncode': None,
                'stdout': '',
                'stderr': str(exc),
            }

        results.append(result)

        print(
            '[DM] STOP '
            f"{service}: rc={result['returncode']} "
            f"{result['stderr'] or result['stdout']}",
            flush=True,
        )

    failed = [
        item for item in results
        if item['returncode'] != 0
    ]

    if failed:
        detail = ' / '.join(
            f"{item['service']}: "
            f"{item['stderr'] or item['stdout'] or item['returncode']}"
            for item in failed
        )

        raise RuntimeError(
            'KMSDRM 실행 전 display-manager STOP 실패: '
            + detail
        )

    # 성공 로그에서는 systemctl stop 직후 KMSDRM child를 실행했다.
    # DRM master 해제에 아주 짧은 여유만 둔다.
    time.sleep(0.5)

    return results


def restore_display_manager_after_projection():
    """
    플립이 끝난 뒤 GUI 복구.
    플립 실행 중에는 절대로 다시 켜지지 않는다.
    """
    command = [
        'sudo', '-n', 'systemctl', 'start', 'display-manager'
    ]

    try:
        completed = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=False,
            timeout=10,
        )

        print(
            '[DM] START display-manager: '
            f'rc={completed.returncode} '
            f'{completed.stderr.strip() or completed.stdout.strip()}',
            flush=True,
        )

        # alias start가 실패하면 lightdm을 직접 시도한다.
        if completed.returncode != 0:
            fallback = subprocess.run(
                ['sudo', '-n', 'systemctl', 'start', 'lightdm'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=False,
                timeout=10,
            )

            print(
                '[DM] START lightdm fallback: '
                f'rc={fallback.returncode} '
                f'{fallback.stderr.strip() or fallback.stdout.strip()}',
                flush=True,
            )

    except Exception as exc:
        print(
            f'[DM] GUI 복구 실패: {exc}',
            flush=True,
        )


def create_app():
    from flask import Flask, request, jsonify, render_template_string, send_file
    app = Flask(__name__)
    app.config['MAX_CONTENT_LENGTH'] = MAX_IMAGE_BYTES + 1024 * 1024
    token = secrets.token_hex(24)
    lock = threading.RLock()
    worker = None
    closing = False
    log = deque(maxlen=200)
    selected = None
    image_base_path = None
    assets = {}
    active_calibration = None
    flip_metrics = {}
    output_mode = 'idle'
    display_manager_stopped = False

    # Integrated calibration child state.
    cal_worker = None
    cal_log = deque(maxlen=800)
    cal_last_result = ''
    cal_started_mtime_ns = None           # projector HIGH/LOW JSON mtime
    lens_started_mtime_ns = None          # camera-lens summary CSV mtime
    cal_preparing = False                 # checkerboard being shown before calibration
    cal_mode = 'idle'                     # lens_only / sweep_only / idle
    cal_display_ready = threading.Event()

    # Fixed-position contrast hold state.
    hold_thread = None
    hold_stop = threading.Event()
    hold_serial = None
    hold_adc = None
    hold_info = None
    hold_error = ''
    hold_last_line = ''

    def is_running():
        return worker is not None and worker.poll() is None

    def is_calibrating():
        return cal_preparing or (cal_worker is not None and cal_worker.poll() is None)

    def require_editable():
        if closing:
            raise ValueError('앱을 종료하는 중입니다.')
        if is_running():
            raise ValueError('이 앱이 출력 중입니다. 출력 종료 후 이미지를 변경하세요.')
        if is_calibrating():
            raise ValueError('자동 캘리브레이션 중입니다. 완료 또는 중지 후 변경하세요.')
        if hold_thread is not None and hold_thread.is_alive():
            raise ValueError('초점 고정 중입니다. 고정/화면 해제 후 이미지를 변경하세요.')

    def save_image_choice():
        # Atomic persistence keeps the base image and effect selection together.
        try:
            WEB_STATE_FILE.parent.mkdir(parents=True, exist_ok=True)
            temporary = WEB_STATE_FILE.with_name(WEB_STATE_FILE.name + '.tmp')
            temporary.write_text(json.dumps({
                'image_path': str(assets[selected['id']]),
                'base_image_path': str(image_base_path),
                'selected': selected,
            }), encoding='utf-8')
            os.replace(temporary, WEB_STATE_FILE)
        except Exception as exc:
            print('[WEB] could not save image choice:', exc, flush=True)

    def import_image(stream, name, mirror_horizontal=False):
        nonlocal selected, image_base_path
        if Path(name).suffix.lower() not in IMAGE_EXTENSIONS:
            raise ValueError('PNG, JPG, BMP, WEBP 이미지를 선택하세요.')
        os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
        import pygame
        # Loading/saving a Surface needs no display initialization or GPIO access.
        source = pygame.image.load(stream, name)
        width, height = source.get_size()
        if width < 1 or height < 1 or width * height > 20_000_000:
            raise ValueError('이미지는 최대 2천만 픽셀까지 지원합니다.')
        flat = pygame.Surface((width, height))
        flat.fill((0, 0, 0))
        flat.blit(source, (0, 0))

        # 노트북 업로드 이미지는 현재 실제 투사에서 좌우가 반대로 보이므로
        # 저장 단계에서 한 번 더 좌우반전해 실제 투사 방향을 보정한다.
        # Pi 로컬 이미지 선택은 기존 파일을 그대로 사용한다.
        if mirror_horizontal:
            flat = pygame.transform.flip(flat, True, False)

        DATA_DIR.mkdir(parents=True, exist_ok=True)
        image_id = secrets.token_hex(16)
        path = DATA_DIR / (image_id + '.png')
        with path.open('xb') as destination:  # Never replace an existing image.
            try:
                pygame.image.save(flat, destination, 'image.png')
            except Exception:
                path.unlink(missing_ok=True)
                raise
        assets[image_id] = path
        selected = {
            'id': image_id,
            'name': name,
            'width': width,
            'height': height,
            'mirror_corrected': bool(mirror_horizontal),
            'effects': {'negative': False, 'strong_contrast': False},
        }
        image_base_path = path
        save_image_choice()
        return selected

    def stop_contrast_hold(send_x=True):
        nonlocal hold_thread, hold_serial, hold_adc, hold_error, hold_last_line

        hold_stop.set()

        thread = hold_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)

        # If the worker did not get a chance to send X, try once more.
        if send_x:
            try:
                if hold_serial is not None and hold_serial.is_open:
                    hold_serial.write(b'X\n')
                    hold_serial.flush()
            except Exception:
                pass

        try:
            if hold_serial is not None:
                hold_serial.close()
        except Exception:
            pass

        hold_serial = None
        hold_thread = None
        hold_adc = None

    def contrast_hold_worker(adc):
        nonlocal hold_serial, hold_error, hold_last_line

        try:
            import serial

            hold_serial = serial.Serial(
                port=UART_PORT,
                baudrate=UART_BAUD,
                timeout=0.04,
                write_timeout=0.5,
            )

            try:
                hold_serial.reset_input_buffer()
            except Exception:
                pass

            # 고정 모드는 반드시 CAL 단일-target 제어기를 사용한다.
            #
            # ATmega 통합 프로토콜:
            #   T<adc>             -> MODE_CAL (settling이 잘 된 PI-D 제어)
            #   T<front>,<back>    -> MODE_ADAPT (교대 플립 전용)
            #
            # 따라서 앞/뒤 단독 고정에서는 pair를 보내면 안 된다.
            # 같은 단일 target 반복 전송은 CAL heartbeat라 controller를
            # 리셋하지 않고 현재 위치를 계속 유지한다.
            command = f'T{int(adc)}\n'.encode()

            while not hold_stop.is_set():
                hold_serial.write(command)
                hold_serial.flush()

                deadline = time.monotonic() + HOLD_HEARTBEAT_SECONDS

                while (
                    not hold_stop.is_set()
                    and time.monotonic() < deadline
                ):
                    try:
                        raw = hold_serial.readline()
                    except Exception:
                        raw = b''

                    if raw:
                        line = raw.decode(errors='replace').strip()
                        if line:
                            hold_last_line = line

                    remaining = deadline - time.monotonic()
                    if remaining > 0:
                        hold_stop.wait(min(0.02, remaining))

            # Explicit release requested by /contrast-hold/stop.
            try:
                hold_serial.write(b'X\n')
                hold_serial.flush()
            except Exception:
                pass

        except Exception as exc:
            hold_error = str(exc)

        finally:
            try:
                if hold_serial is not None:
                    hold_serial.close()
            except Exception:
                pass
            hold_serial = None

    def start_contrast_hold(info):
        nonlocal hold_thread, hold_adc, hold_info, hold_error, hold_last_line

        if hold_thread is not None and hold_thread.is_alive():
            raise ValueError('이미 ADC 위치 고정이 실행 중입니다.')

        owner = uart_owner_process()
        if owner:
            raise ValueError(
                'UART가 다른 프로그램에서 사용 중입니다. '
                '자동초점/캘리브레이션 프로그램을 종료한 뒤 실행하세요.\n'
                + owner
            )

        adc = int(info['adc'])

        if not 100 <= adc <= 1023:
            raise ValueError(
                f'계산된 ADC {adc}가 ATmega target 범위(100~1023)를 벗어났습니다.'
            )

        hold_stop.clear()
        hold_adc = adc
        hold_info = dict(info)
        hold_error = ''
        hold_last_line = ''

        hold_thread = threading.Thread(
            target=contrast_hold_worker,
            args=(adc,),
            name='front-sharp-back-blur-hold',
            daemon=True,
        )
        hold_thread.start()

    @app.before_request
    def protect_controls():
        if request.method == 'POST' and request.headers.get('X-Mango-Token') != token:
            return jsonify(error='페이지를 새로고침하세요.'), 403

    @app.errorhandler(413)
    def upload_too_large(_error):
        return jsonify(error='업로드 파일이 너무 큽니다. 32 MB 이하 이미지를 선택하세요.'), 413

    def stop_calibration_locked():
        nonlocal cal_worker, cal_preparing
        cal_preparing = False
        process = cal_worker
        if process is None or process.poll() is not None:
            return
        try:
            process.send_signal(signal.SIGINT)
        except Exception:
            pass
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            try:
                process.terminate(); process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait(timeout=2.0)

    def read_calibration_log(process, mode_name):
        nonlocal cal_last_result

        for line in process.stdout:
            cal_log.append(line)
            print('[CAL] ' + line, end='', flush=True)

        try:
            process.stdout.close()
        except Exception:
            pass

        code_value = process.poll()

        try:
            if mode_name == 'lens_only':
                try:
                    stat = CAMERA_LENS_SUMMARY_FILE.stat()
                    refreshed = (
                        lens_started_mtime_ns is None
                        or stat.st_mtime_ns > lens_started_mtime_ns
                    )
                except FileNotFoundError:
                    refreshed = False

                if refreshed:
                    values = read_saved_camera_lens_calibration()
                    cal_last_result = (
                        '렌즈 캘리 완료 · '
                        + ' / '.join(
                            f"{q} {values[q]['LensPosition']:.3f}D"
                            for q in ('Q1', 'Q2', 'Q3', 'Q4')
                        )
                    )
                elif code_value == 0:
                    cal_last_result = (
                        '렌즈 캘리 종료됨 · '
                        '새 camera lens summary CSV는 확인되지 않았습니다.'
                    )
                else:
                    cal_last_result = (
                        f'렌즈 캘리 종료 코드 {code_value}'
                    )

            elif mode_name == 'sweep_only':
                try:
                    stat = CALIBRATION_FILE.stat()
                    refreshed = (
                        cal_started_mtime_ns is None
                        or stat.st_mtime_ns > cal_started_mtime_ns
                    )
                except FileNotFoundError:
                    refreshed = False

                if refreshed:
                    result = read_calibration()
                    cal_last_result = (
                        '스윕 캘리 완료 · '
                        'HIGH Q{hq} ADC {ha:.2f} / '
                        'LOW Q{lq} ADC {la:.2f}'
                    ).format(
                        hq=result['high']['quadrant'],
                        ha=float(result['high']['adc']),
                        lq=result['low']['quadrant'],
                        la=float(result['low']['adc']),
                    )
                elif code_value == 0:
                    cal_last_result = (
                        '스윕 캘리 종료됨 · '
                        '새 HIGH/LOW calibration JSON은 확인되지 않았습니다.'
                    )
                else:
                    cal_last_result = (
                        f'스윕 캘리 종료 코드 {code_value}'
                    )

            else:
                cal_last_result = (
                    f'캘리브레이션 종료 코드 {code_value}'
                )

        except Exception as exc:
            cal_last_result = (
                '캘리브레이션 종료 · 결과 확인 실패: '
                + str(exc)
            )

        # Checkerboard off + GUI back, once calibration has finished.
        if output_mode == 'calibration_display':
            stop_worker()


    def stop_worker():
        nonlocal worker, display_manager_stopped, output_mode
        with lock:
            if worker is None or worker.poll() is not None:
                if display_manager_stopped:
                    restore_display_manager_after_projection()
                    display_manager_stopped = False
                return
            try:
                worker.stdin.write('STOP\n')
                worker.stdin.flush()
            except OSError:
                pass
            try:
                worker.wait(timeout=3)
            except subprocess.TimeoutExpired:
                worker.terminate()  # Only this app's own child process.
                try:
                    worker.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    worker.kill()
                    worker.wait(timeout=2)

            output_mode = 'idle'
            if display_manager_stopped:
                restore_display_manager_after_projection()
                display_manager_stopped = False

    def read_log(process, target):
        nonlocal flip_metrics, output_mode

        for line in process.stdout:
            if line.startswith('@@FLIP_METRIC@@'):
                payload = line[len('@@FLIP_METRIC@@'):].strip()

                try:
                    parsed = json.loads(payload)

                    if isinstance(parsed, dict):
                        flip_metrics = parsed
                except Exception:
                    target.append('Timing metric parse error: ' + payload + '\n')

                continue

            if '[FIXED DISPLAY]' in line and '시작' in line:
                cal_display_ready.set()

            target.append(line)

            # Child renderer log도 웹서버를 실행한 CMD에서 바로 보이게 한다.
            print(line, end='', flush=True)

        process.stdout.close()

        if output_mode in ('front_fixed', 'back_fixed'):
            stop_contrast_hold(send_x=True)

        output_mode = 'idle'

        # 실행 시간 만료/오류 등으로 renderer가 스스로 끝난 경우에도
        # GUI를 다시 올린다.
        nonlocal display_manager_stopped
        if display_manager_stopped:
            try:
                process.wait(timeout=1.0)
            except Exception:
                pass

            restore_display_manager_after_projection()
            display_manager_stopped = False

    @app.get('/')
    def index():
        return render_template_string(
            PAGE,
            token=token,
            image_dir=str(IMAGE_DIR),
            gpio_phase_lead_ms_default=GPIO_PHASE_LEAD_MS_DEFAULT,
            gpio_phase_lead_ms_max=GPIO_PHASE_LEAD_MS_MAX,
            back_fake_target_offset_default=BACK_FAKE_TARGET_OFFSET_DEFAULT,
            back_fake_target_offset_max=BACK_FAKE_TARGET_OFFSET_MAX,
            quick_flip_fps=QUICK_FLIP_FPS,
        )

    @app.get('/status')
    def status():
        with lock:
            code = worker.poll() if worker else None
            running = worker is not None and code is None
            calibration = None
            calibration_error = ''
            try:
                calibration = calibration_view(read_calibration())
            except Exception as exc:
                calibration_error = str(exc)
            cal_code = cal_worker.poll() if cal_worker else None
            cal_running = cal_preparing or (cal_worker is not None and cal_code is None)
            saved_lens = saved_camera_lens_status()
            return jsonify(running=running, code=code, log=''.join(list(log)),
                           output_mode=output_mode,
                           image=selected, calibration=calibration,
                           calibration_error=calibration_error,
                           flip_metrics=flip_metrics,
                           active_calibration=calibration_view(active_calibration)
                           if running and active_calibration else None,
                           calibration_worker={
                               'running': cal_running, 'code': cal_code,
                               'log': ''.join(list(cal_log)), 'last_result': cal_last_result,
                               'preview_available': CAL_PREVIEW_PATH.is_file(),
                               'mode': cal_mode,
                           },
                           saved_camera_lens=saved_lens,
                           hold={
                               'running': bool(
                                   hold_thread is not None
                                   and hold_thread.is_alive()
                               ),
                               'adc': hold_adc,
                               'info': hold_info,
                               'error': hold_error,
                               'last_line': hold_last_line,
                           })

    def start_renderer_locked(config, mode_name):
        """Stop the desktop and start the KMSDRM renderer child (lock held)."""
        nonlocal worker, log, flip_metrics, display_manager_stopped, output_mode
        dm_results = stop_display_managers_for_projection()
        display_manager_stopped = True
        print(f'[DM] pre-stop ({mode_name}): '
              + json.dumps(dm_results, ensure_ascii=False), flush=True)
        env = os.environ.copy()
        for key in ('DISPLAY', 'WAYLAND_DISPLAY', 'XDG_RUNTIME_DIR',
                    'DBUS_SESSION_BUS_ADDRESS', 'PYTHONPATH', 'PYTHONHOME',
                    'SDL_WINDOWID', 'LD_PRELOAD', 'LD_LIBRARY_PATH'):
            env.pop(key, None)
        env['SDL_VIDEODRIVER'] = 'kmsdrm'
        env['SDL_AUDIODRIVER'] = 'dummy'
        env['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
        env['SDL_VIDEO_X11_NET_WM_BYPASS_COMPOSITOR'] = '1'
        if worker and worker.stdin:
            worker.stdin.close()
        output_mode = mode_name
        worker = subprocess.Popen(
            ['sudo', '-n', '-E', '/usr/bin/python3', '-s', '-B', '-u',
             str(Path(__file__).resolve()), '--mango-display', json.dumps(config)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1, env=env, start_new_session=True,
        )
        log = deque(maxlen=200)
        flip_metrics = {}
        threading.Thread(target=read_log, args=(worker, log), daemon=True).start()

    def calibration_launch_job(mode_name, saved_lens_snapshot=None):
        """background: checkerboard on -> selected calibration child"""
        nonlocal cal_worker, cal_preparing, cal_last_result
        try:
            image = Path(CALIBRATION_IMAGE)
            if CALIBRATION_IMAGE and image.is_file():
                with lock:
                    if not cal_preparing:
                        return
                    cal_display_ready.clear()
                    cal_last_result = (
                        '체커보드 표시 중… · 렌즈 캘리'
                        if mode_name == 'lens_only'
                        else '체커보드 표시 중… · 프로젝터 스윕 캘리'
                    )
                    start_renderer_locked(
                        {'image': str(image), 'render_mode': 'fixed_image',
                         'seconds': 0, 'gpio': False},
                        'calibration_display',
                    )
                deadline = time.monotonic() + CALIBRATION_DISPLAY_WAIT
                while not cal_display_ready.is_set():
                    if not cal_preparing:
                        return
                    if worker is None or worker.poll() is not None:
                        raise RuntimeError('체커보드 표시 실패: ' + ''.join(list(log))[-300:])
                    if time.monotonic() > deadline:
                        raise RuntimeError('체커보드가 제한 시간 안에 표시되지 않았습니다.')
                    time.sleep(0.1)
                time.sleep(0.8)
            else:
                print('[WEB-CAL] checkerboard not found, running without it: '
                      + str(CALIBRATION_IMAGE), flush=True)

            with lock:
                if not cal_preparing:
                    return
                env = os.environ.copy()
                env['HOME'] = str(USER_HOME)
                env['PYTHONUNBUFFERED'] = '1'
                env['MANGO_CALIBRATION_MODE'] = mode_name

                if mode_name == 'sweep_only':
                    if not saved_lens_snapshot:
                        raise RuntimeError(
                            '스윕 캘리에 사용할 저장 렌즈값 snapshot이 없습니다.'
                        )
                    env['MANGO_SAVED_LENS_JSON'] = json.dumps(
                        saved_lens_snapshot,
                        ensure_ascii=False,
                        separators=(',', ':'),
                    )
                else:
                    env.pop('MANGO_SAVED_LENS_JSON', None)

                command = [sys.executable, '-s', '-B', '-u', str(Path(__file__).resolve()),
                           '--mango-calibrate', str(CAL_PREVIEW_PATH)]
                cal_worker = subprocess.Popen(
                    command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    stdin=subprocess.DEVNULL, text=True, bufsize=1, env=env,
                    cwd=str(USER_HOME), start_new_session=True,
                )
                cal_last_result = (
                    '렌즈 캘리 실행 중…'
                    if mode_name == 'lens_only'
                    else '프로젝터 스윕 캘리 실행 중…'
                )
                cal_preparing = False
            threading.Thread(
                target=read_calibration_log,
                args=(cal_worker, mode_name),
                daemon=True,
            ).start()
        except Exception as exc:
            with lock:
                cal_preparing = False
                cal_last_result = f'캘리브레이션 시작 실패: {exc}'
            stop_worker()

    def send_uart_stop():
        """One X line to the ATmega (skipped if another program owns the UART)."""
        owner = uart_owner_process()
        if owner:
            return 'UART 사용 중 - X 생략'
        try:
            import serial
            port = serial.Serial(UART_PORT, UART_BAUD, timeout=0.2, write_timeout=0.5)
            try:
                port.reset_input_buffer()
                port.write(b'X\n')
                port.flush()
                reply = port.readline().decode(errors='replace').strip()
            finally:
                port.close()
            return reply or 'X 전송'
        except Exception as exc:
            return f'X 전송 실패: {exc}'

    @app.post('/all-stop')
    def all_stop():
        with lock:
            stop_calibration_locked()
        stop_worker()
        stop_contrast_hold(send_x=True)
        return jsonify(ok=True, uart=send_uart_stop())

    @app.get('/calibration/preview')
    def calibration_preview():
        if not CAL_PREVIEW_PATH.is_file():
            return ('', 204)
        return send_file(CAL_PREVIEW_PATH, mimetype='image/jpeg', max_age=CAL_PREVIEW_MAX_AGE)

    def begin_calibration(mode_name):
        nonlocal cal_worker, cal_last_result
        nonlocal cal_started_mtime_ns, lens_started_mtime_ns
        nonlocal cal_preparing, cal_mode

        with lock:
            try:
                if closing:
                    raise ValueError(
                        '앱을 종료하는 중입니다.'
                    )

                if is_running():
                    raise ValueError(
                        '플립 출력 중에는 캘리브레이션을 시작할 수 없습니다.'
                    )

                if is_calibrating():
                    raise ValueError(
                        '캘리브레이션이 이미 실행 중입니다.'
                    )

                if (
                    hold_thread is not None
                    and hold_thread.is_alive()
                ):
                    raise ValueError(
                        '고정 모드를 먼저 해제하세요.'
                    )

                owner = uart_owner_process()

                if owner:
                    raise ValueError(
                        'UART가 다른 프로그램에서 사용 중입니다. '
                        '기존 자동초점/제어 프로그램을 종료하세요.\n'
                        + owner
                    )

                saved_lens_snapshot = None

                if mode_name == 'lens_only':
                    # Keep any previous valid lens CSV until a new successful
                    # lens calibration overwrites it.
                    try:
                        lens_started_mtime_ns = (
                            CAMERA_LENS_SUMMARY_FILE.stat().st_mtime_ns
                        )
                    except FileNotFoundError:
                        lens_started_mtime_ns = None

                    cal_started_mtime_ns = None

                elif mode_name == 'sweep_only':
                    # Snapshot the saved Q1~Q4 values now.
                    saved_lens_snapshot = (
                        read_saved_camera_lens_calibration()
                    )

                    try:
                        cal_started_mtime_ns = (
                            CALIBRATION_FILE.stat().st_mtime_ns
                        )
                    except FileNotFoundError:
                        cal_started_mtime_ns = None

                    lens_started_mtime_ns = None

                else:
                    raise ValueError(
                        '알 수 없는 calibration mode: '
                        + str(mode_name)
                    )

                try:
                    CAL_PREVIEW_PATH.unlink()
                except FileNotFoundError:
                    pass

                cal_log.clear()
                cal_mode = mode_name
                cal_last_result = (
                    '렌즈 캘리 시작 중…'
                    if mode_name == 'lens_only'
                    else '프로젝터 스윕 캘리 시작 중…'
                )
                cal_preparing = True

                threading.Thread(
                    target=calibration_launch_job,
                    args=(
                        mode_name,
                        saved_lens_snapshot,
                    ),
                    daemon=True,
                ).start()

                return jsonify(
                    ok=True,
                    mode=mode_name,
                    saved_lens=saved_lens_snapshot,
                )

            except Exception as exc:
                return jsonify(
                    error=str(exc)
                ), 400

    @app.post('/calibration/lens/start')
    def calibration_lens_start():
        return begin_calibration('lens_only')

    @app.post('/calibration/sweep/start')
    def calibration_sweep_start():
        return begin_calibration('sweep_only')

    # Legacy endpoint retained only so an old browser tab does not crash.
    # It now maps to LENS calibration; the UI itself never uses this route.
    @app.post('/calibration/start')
    def calibration_legacy_start():
        return begin_calibration('lens_only')

    @app.post('/calibration/stop')
    def calibration_stop():
        with lock:
            stop_calibration_locked()
        if output_mode == 'calibration_display':
            stop_worker()
        return jsonify(ok=True)

    @app.get('/images')
    def images():
        try:
            directory = Path(request.args.get('directory') or IMAGE_DIR).expanduser().resolve()
            if not directory.is_dir():
                raise ValueError('이미지 폴더가 없습니다: ' + str(directory))
            files = [{'name': p.name, 'path': str(p)} for p in sorted(directory.iterdir())
                     if p.suffix.lower() in IMAGE_EXTENSIONS and p.is_file()]
            return jsonify(files=files)
        except Exception as exc:
            return jsonify(error=str(exc)), 400

    @app.get('/image/<image_id>')
    def image_file(image_id):
        with lock:
            path = assets.get(image_id)
        if path is None:
            return jsonify(error='이미지를 찾을 수 없습니다.'), 404
        return send_file(path, mimetype='image/png', max_age=3600)

    @app.post('/image-effects')
    def image_effects():
        nonlocal selected
        with lock:
            try:
                require_editable()
                data = request.get_json(silent=True)
                if not isinstance(data, dict):
                    raise ValueError('이미지 효과 설정을 확인하세요.')
                if not selected or data.get('image_id') != selected['id']:
                    raise ValueError('현재 선택된 이미지를 확인하세요.')
                negative = data.get('negative')
                strong_contrast = data.get('strong_contrast')
                if type(negative) is not bool or type(strong_contrast) is not bool:
                    raise ValueError('이미지 효과는 켜기/끄기로 지정하세요.')
                if image_base_path is None or not image_base_path.is_file():
                    raise ValueError('효과 적용 전 이미지가 없습니다. 이미지를 다시 불러오세요.')
                if not negative and not strong_contrast:
                    path = image_base_path
                else:
                    # Always derive from the imported base: effects never accumulate.
                    suffix = f'.effect-{int(negative)}{int(strong_contrast)}.png'
                    path = image_base_path.with_name(image_base_path.stem + suffix)
                    if not path.is_file():
                        import pygame
                        source = pygame.image.load(str(image_base_path))
                        lut = []
                        for value in range(256):
                            adjusted = max(0, min(255, round((value - 127.5) * 4 + 127.5))) if strong_contrast else value
                            lut.append(255 - adjusted if negative else adjusted)
                        pixels = pygame.image.tostring(source, 'RGB').translate(bytes(lut))
                        effected = pygame.image.fromstring(pixels, source.get_size(), 'RGB')
                        temporary = path.with_name(path.name + '.tmp')
                        try:
                            with temporary.open('wb') as destination:
                                pygame.image.save(effected, destination, 'image.png')
                            os.replace(temporary, path)
                        finally:
                            temporary.unlink(missing_ok=True)
                image_id = path.stem
                assets[image_id] = path
                selected = {**selected, 'id': image_id,
                            'effects': {'negative': negative, 'strong_contrast': strong_contrast}}
                save_image_choice()
                return jsonify(ok=True, image=selected)
            except Exception as exc:
                return jsonify(error=str(exc)), 400

    @app.post('/upload')
    def upload():
        with lock:
            try:
                require_editable()
                file = request.files.get('image')
                if file is None or not file.filename:
                    raise ValueError('업로드할 이미지를 선택하세요.')
                file.stream.seek(0, 2)
                if file.stream.tell() > MAX_IMAGE_BYTES:
                    raise ValueError('32 MB 이하 이미지를 선택하세요.')
                file.stream.seek(0)
                image = import_image(file.stream, Path(file.filename).name, mirror_horizontal=True)
                return jsonify(ok=True, image=image)
            except Exception as exc:
                # Preserve Flask's upload-size response rather than masking it.
                if getattr(exc, 'code', None) == 413:
                    raise
                return jsonify(error=str(exc)), 400

    @app.post('/select')
    def select():
        with lock:
            try:
                require_editable()
                data = request.get_json(silent=True) or {}
                if not isinstance(data, dict) or not isinstance(data.get('path'), str):
                    raise ValueError('라즈베리파이 이미지 경로를 지정하세요.')
                path = Path(data['path']).expanduser().resolve()
                if not path.is_file():
                    raise ValueError('이미지 파일이 없습니다: ' + str(path))
                if path.stat().st_size > MAX_IMAGE_BYTES:
                    raise ValueError('32 MB 이하 이미지를 선택하세요.')
                with path.open('rb') as stream:
                    image = import_image(stream, path.name, mirror_horizontal=False)
                return jsonify(ok=True, image=image)
            except Exception as exc:
                return jsonify(error=str(exc)), 400

    @app.post('/start')
    def start():
        nonlocal worker, log, active_calibration, flip_metrics, display_manager_stopped, output_mode
        with lock:
            try:
                if closing:
                    raise ValueError('앱을 종료하는 중입니다.')
                if is_calibrating():
                    raise ValueError('자동 캘리브레이션이 실행 중입니다. 완료 후 플립을 시작하세요.')
                if is_running():
                    raise ValueError('이 앱이 이미 출력 중입니다.')
                busy = existing_output()
                if busy:
                    raise ValueError('기존 프로그램이 실행 중입니다: ' + busy)
                data = request.get_json(silent=True)
                if not isinstance(data, dict):
                    raise ValueError('설정 형식이 잘못되었습니다.')
                if not selected or data.get('image_id') != selected['id']:
                    raise ValueError('사용할 이미지를 먼저 불러오세요.')
                calibration = read_calibration()
                config = {'image': str(assets[selected['id']]), 'calibration': calibration}
                for key, low, high in [('fps', 1, 240), ('seconds', 0, 86400)]:
                    value = data.get(key)
                    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
                        raise ValueError(f'{key}: {low}~{high} 범위로 입력하세요.')
                    config[key] = float(value)

                gpio_phase_lead_ms = data.get('gpio_phase_lead_ms')
                if (
                    type(gpio_phase_lead_ms) not in (int, float)
                    or not math.isfinite(gpio_phase_lead_ms)
                    or not 0 <= gpio_phase_lead_ms <= GPIO_PHASE_LEAD_MS_MAX
                ):
                    raise ValueError(
                        'GPIO 선행 시간은 '
                        f'0~{GPIO_PHASE_LEAD_MS_MAX:g} ms 범위로 입력하세요.'
                    )
                config['gpio_phase_lead_ms'] = float(gpio_phase_lead_ms)

                back_fake_target_offset = data.get(
                    'back_fake_target_offset',
                    BACK_FAKE_TARGET_OFFSET_DEFAULT,
                )
                if (
                    type(back_fake_target_offset) not in (int, float)
                    or not math.isfinite(back_fake_target_offset)
                    or not 0 <= back_fake_target_offset <= BACK_FAKE_TARGET_OFFSET_MAX
                ):
                    raise ValueError(
                        '뒤 초점 전환 보정은 '
                        f'0~{BACK_FAKE_TARGET_OFFSET_MAX:g} ADC 범위로 입력하세요.'
                    )
                back_fake_target_offset = float(back_fake_target_offset)

                if type(data.get('fullscreen')) is not bool:
                    raise ValueError('전체화면 체크박스 설정이 잘못되었습니다.')

                # 이 프로젝트에서는 이미지 플립과 VCM 전환이 항상 같이 가므로
                # GPIO17 동기 신호를 선택 옵션으로 두지 않는다.
                config.update(fullscreen=data['fullscreen'], gpio=True)

                # 프로젝터 직접 출력 준비:
                # KMSDRM child 실행 전에 lightdm + display-manager를 먼저 STOP.
                dm_results = stop_display_managers_for_projection()
                display_manager_stopped = True

                print(
                    '[DM] pre-stop complete: '
                    + json.dumps(dm_results, ensure_ascii=False),
                    flush=True,
                )

                # 물리적 FRONT/BACK:
                #   FRONT = HIGH ADC
                #   BACK  = LOW ADC
                #
                # Fake-target experiment (FLIP only):
                #   real FRONT = calibration HIGH, unchanged
                #   real BACK  = calibration LOW
                #   command BACK = real BACK - offset
                #
                # VCM_Unified.c sorts unequal pair values internally, so pair
                # order remains configurable/compatible with the existing Mango.
                front_real_adc = float(calibration['high']['adc'])
                back_real_adc = float(calibration['low']['adc'])
                back_command_adc = back_real_adc - back_fake_target_offset

                if back_command_adc < 100.0:
                    raise ValueError(
                        '뒤 초점 명령값이 ATmega 최소 ADC 100보다 작아집니다. '
                        '보정값을 줄이세요.'
                    )

                if PAIR_ORDER == 'low_high':
                    pair = (back_command_adc, front_real_adc)
                else:
                    pair = (front_real_adc, back_command_adc)

                uart_target_info = send_adaptive_target_pair(*pair)
                uart_target_info.update({
                    'front_real_adc': front_real_adc,
                    'back_real_adc': back_real_adc,
                    'back_command_adc': float(back_command_adc),
                    'back_fake_target_offset': back_fake_target_offset,
                })

                print(
                    '[FAKE TARGET] '
                    f'FRONT={front_real_adc:.1f} unchanged | '
                    f'BACK real={back_real_adc:.1f} -> '
                    f'command={back_command_adc:.1f} '
                    f'(offset={back_fake_target_offset:.1f})',
                    flush=True,
                )

                env = os.environ.copy()

                # KMS 렌더러 실행 환경.
                for key in (
                    'DISPLAY',
                    'WAYLAND_DISPLAY',
                    'XDG_RUNTIME_DIR',
                    'DBUS_SESSION_BUS_ADDRESS',
                    'PYTHONPATH',
                    'PYTHONHOME',
                    'SDL_WINDOWID',
                    'LD_PRELOAD',
                    'LD_LIBRARY_PATH',
                ):
                    env.pop(key, None)

                env['SDL_VIDEODRIVER'] = 'kmsdrm'
                env['SDL_AUDIODRIVER'] = 'dummy'
                env['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
                env['SDL_VIDEO_X11_NET_WM_BYPASS_COMPOSITOR'] = '1'

                if worker and worker.stdin:
                    worker.stdin.close()

                output_mode = 'flip'
                worker = subprocess.Popen(
                    [
                        'sudo', '-n', '-E', '/usr/bin/python3',
                        '-s', '-B', '-u',
                        str(Path(__file__).resolve()),
                        '--mango-display',
                        json.dumps(config),
                    ],
                    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, bufsize=1, env=env, start_new_session=True,
                )
                log = deque(maxlen=200)
                flip_metrics = {}
                active_calibration = calibration
                threading.Thread(target=read_log, args=(worker, log), daemon=True).start()
                return jsonify(
                    ok=True,
                    uart_targets=uart_target_info,
                )
            except Exception as exc:
                if display_manager_stopped:
                    restore_display_manager_after_projection()
                    display_manager_stopped = False
                return jsonify(error=str(exc)), 400

    @app.post('/focus-fixed/start')
    def focus_fixed_start():
        nonlocal worker, log, active_calibration, flip_metrics, display_manager_stopped, output_mode

        with lock:
            try:
                if closing:
                    raise ValueError('앱을 종료하는 중입니다.')
                if is_calibrating():
                    raise ValueError('자동 캘리브레이션 중에는 고정 모드를 시작할 수 없습니다.')
                if is_running():
                    raise ValueError('현재 화면 출력이 실행 중입니다. 먼저 중지하세요.')
                if hold_thread is not None and hold_thread.is_alive():
                    raise ValueError('이미 초점 고정이 실행 중입니다.')

                data = request.get_json(silent=True) or {}
                plane = str(data.get('plane', '')).strip().lower()

                if plane not in {'front', 'back'}:
                    raise ValueError('plane은 front 또는 back이어야 합니다.')

                if not selected or data.get('image_id') != selected['id']:
                    raise ValueError('고정 모드에서 띄울 이미지를 먼저 불러오세요.')

                calibration = read_calibration()

                if plane == 'front':
                    adc = int(round(calibration['high']['adc']))
                    info = {
                        'adc': adc,
                        'plane': 'front',
                        'label': '앞(FRONT / HIGH ADC)',
                        'quadrant': int(calibration['high']['quadrant']),
                    }
                    output_mode = 'front_fixed'
                else:
                    adc = int(round(calibration['low']['adc']))
                    info = {
                        'adc': adc,
                        'plane': 'back',
                        'label': '뒤(BACK / LOW ADC)',
                        'quadrant': int(calibration['low']['quadrant']),
                    }
                    output_mode = 'back_fixed'

                # 기존에 잘 되던 고정 방식 복원:
                # 단일 T<adc> heartbeat를 계속 보내 ATmega를 MODE_CAL에 두고
                # calibration용 PI-D controller로 정확한 위치를 유지한다.
                # 고정 모드에서는 GPIO17을 사용하지 않는다.
                start_contrast_hold(info)

                dm_results = stop_display_managers_for_projection()
                display_manager_stopped = True

                print(
                    '[DM] fixed-mode pre-stop complete: '
                    + json.dumps(dm_results, ensure_ascii=False),
                    flush=True,
                )

                env = os.environ.copy()
                for key in (
                    'DISPLAY',
                    'WAYLAND_DISPLAY',
                    'XDG_RUNTIME_DIR',
                    'DBUS_SESSION_BUS_ADDRESS',
                    'PYTHONPATH',
                    'PYTHONHOME',
                    'SDL_WINDOWID',
                    'LD_PRELOAD',
                    'LD_LIBRARY_PATH',
                ):
                    env.pop(key, None)

                env['SDL_VIDEODRIVER'] = 'kmsdrm'
                env['SDL_AUDIODRIVER'] = 'dummy'
                env['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
                env['SDL_VIDEO_X11_NET_WM_BYPASS_COMPOSITOR'] = '1'

                config = {
                    'image': str(assets[selected['id']]),
                    'render_mode': 'fixed_image',
                    'seconds': 0,
                    'gpio': False,
                }

                if worker and worker.stdin:
                    worker.stdin.close()

                worker = subprocess.Popen(
                    [
                        'sudo', '-n', '-E', '/usr/bin/python3',
                        '-s', '-B', '-u',
                        str(Path(__file__).resolve()),
                        '--mango-display',
                        json.dumps(config),
                    ],
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                    env=env,
                    start_new_session=True,
                )

                log = deque(maxlen=200)
                flip_metrics = {}
                active_calibration = calibration

                threading.Thread(
                    target=read_log,
                    args=(worker, log),
                    daemon=True,
                ).start()

                return jsonify(
                    ok=True,
                    plane=plane,
                    adc=adc,
                    info=info,
                )

            except Exception as exc:
                # 고정 target thread가 시작된 뒤 화면 실행이 실패해도 반드시 해제.
                try:
                    stop_contrast_hold(send_x=True)
                except Exception:
                    pass

                output_mode = 'idle'

                if display_manager_stopped:
                    restore_display_manager_after_projection()
                    display_manager_stopped = False

                return jsonify(error=str(exc)), 400

    @app.post('/contrast-hold/start')
    def contrast_hold_start():
        with lock:
            try:
                data = request.get_json(silent=True) or {}
                if not isinstance(data, dict):
                    raise ValueError('설정 형식이 잘못되었습니다.')

                calibration = read_calibration()
                csv_path = find_focus_csv(data.get('csv_path', ''))
                ratio = data.get(
                    'acceptable_ratio',
                    FRONT_ACCEPTABLE_RATIO_DEFAULT,
                )
                info = choose_front_focus_margin_away_from_back(
                    calibration=calibration,
                    csv_path=csv_path,
                    acceptable_ratio=ratio,
                )

                start_contrast_hold(info)

                return jsonify(
                    ok=True,
                    info=info,
                )

            except Exception as exc:
                return jsonify(error=str(exc)), 400

    @app.post('/contrast-hold/stop')
    def contrast_hold_stop():
        with lock:
            stop_contrast_hold(send_x=True)
            return jsonify(ok=True)

    @app.post('/stop')
    def stop():
        stop_worker()
        stop_contrast_hold(send_x=True)
        return jsonify(ok=True)

    def shutdown():
        nonlocal closing
        with lock:
            closing = True
        with lock:
            stop_calibration_locked()
        stop_worker()
        stop_contrast_hold(send_x=True)

    # Restore the last flip image so FLIP works right after a restart.
    try:
        saved = json.loads(WEB_STATE_FILE.read_text(encoding='utf-8'))
        saved_path = Path(saved['image_path'])
        if saved_path.is_file():
            assets[saved['selected']['id']] = saved_path
            selected = dict(saved['selected'])
            image_base_path = Path(saved.get('base_image_path') or saved['image_path'])
            selected.setdefault('effects', {'negative': False, 'strong_contrast': False})
            print('[WEB] restored flip image:', selected.get('name'), flush=True)
    except FileNotFoundError:
        pass
    except Exception as exc:
        print('[WEB] could not read saved image choice:', exc, flush=True)

    return app, shutdown


def run_web():
    app, shutdown = create_app()
    print(f'Mango Comet ADC Prism: http://pi5-dlp.local:{PORT}/', flush=True)
    print('사용자 HOME:', USER_HOME, flush=True)
    print('캘리브레이션 결과 파일:', CALIBRATION_FILE, flush=True)
    print(
        '[KMS] direct projection mode: '
        'child=sudo -n -E python3 / kmsdrm / vsync OFF / clock pacing',
        flush=True,
    )
    print(
        '[KMS] 플립 시작 전 '
        'lightdm/display-manager를 자동 STOP합니다.',
        flush=True,
    )
    import logging

    class _HideStatusPolling(logging.Filter):
        def filter(self, record):
            message = record.getMessage()
            return '/status' not in message and '/calibration/preview' not in message

    logging.getLogger('werkzeug').addFilter(_HideStatusPolling())

    try:
        app.run(host='0.0.0.0', port=PORT, debug=False, use_reloader=False, threaded=True)
    finally:
        shutdown()


if __name__ == '__main__':
    if '--mango-calibrate' in sys.argv:
        raise SystemExit(run_integrated_calibration(sys.argv[-1]))
    if '--mango-display' in sys.argv:
        raise SystemExit(display_images(json.loads(sys.argv[-1])))
    run_web()
