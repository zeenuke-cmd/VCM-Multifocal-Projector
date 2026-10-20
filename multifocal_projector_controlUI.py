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

# Written by the embedded autofocus after a successful FULL camera-lens
# calibration. The quick calibration mode reuses only these Q1~Q4
# LensPosition values; projector ADC calibration is still performed again.
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


def read_previous_camera_lens_values(path=None):
    """Load the last successful Q1~Q4 Camera Module 3 lens positions."""
    path = CAMERA_LENS_SUMMARY_FILE if path is None else Path(path)

    try:
        stream = path.open('r', encoding='utf-8-sig', newline='')
    except FileNotFoundError:
        raise ValueError(
            '이전 렌즈 캘리브레이션 값이 없습니다. '
            '먼저 "풀 캘리브레이션(렌즈 포함)"을 한 번 실행하세요.'
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
                '이전 렌즈 캘리브레이션 CSV 형식이 맞지 않습니다: '
                + ', '.join(sorted(missing))
            )

        selected = {}

        for row in reader:
            q = str(row.get('Quadrant', '')).strip()
            if q not in {'Q1', 'Q2', 'Q3', 'Q4'}:
                continue

            try:
                lens = float(row['SelectedLensPosition'])
                reference_adc = float(row['ReferenceADC'])
            except (TypeError, ValueError):
                raise ValueError(
                    f'이전 렌즈값 {q}의 숫자 형식이 잘못되었습니다.'
                ) from None

            if not math.isfinite(lens) or not math.isfinite(reference_adc):
                raise ValueError(
                    f'이전 렌즈값 {q}에 유효하지 않은 숫자가 있습니다.'
                )

            selected[q] = {
                'LensPosition': lens,
                'ReferenceADC': int(round(reference_adc)),
                'Metric': str(row.get('SelectedMetric') or 'REUSED'),
            }

    missing_q = [
        q for q in ('Q1', 'Q2', 'Q3', 'Q4')
        if q not in selected
    ]
    if missing_q:
        raise ValueError(
            '이전 렌즈 캘리브레이션에 사분면 값이 부족합니다: '
            + ', '.join(missing_q)
        )

    return selected


def previous_camera_lens_status():
    try:
        values = read_previous_camera_lens_values()
        return {
            'available': True,
            'file': str(CAMERA_LENS_SUMMARY_FILE),
            'values': values,
        }
    except Exception as exc:
        return {
            'available': False,
            'file': str(CAMERA_LENS_SUMMARY_FILE),
            'error': str(exc),
            'values': {},
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
        'label': '가운데 수직선 · FRONT/BACK 슬라이스 좌우 교환 적용',
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

        # 친구의 known-good image_flip.py와 동일한 display mode:
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
        # 친구의 known-good kmsdrm 방식과 동일하게 VSync는 요청하지 않는다.
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
# Exact auto_focus_test_division20_fastsettle.py source, compressed into Mango.
# ============================================================
_EMBEDDED_AUTOFOCUS_B64 = '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'
_EMBEDDED_AUTOFOCUS_SHA256 = '372f6f0eca559c43e7131101f8bf4544500f4ba9f8778c3faa9ce52ab2469ee7'


def _embedded_autofocus_source():
    raw = zlib.decompress(base64.b64decode(_EMBEDDED_AUTOFOCUS_B64)).decode('utf-8')
    import hashlib
    actual = hashlib.sha256(raw.encode('utf-8')).hexdigest()
    if actual != _EMBEDDED_AUTOFOCUS_SHA256:
        raise RuntimeError('내장 자동초점 코드 무결성 확인 실패')
    return raw


def _autofocus_source_reusing_previous_lens(source):
    """
    Full calibration executes the embedded source byte-for-byte unchanged.

    Quick calibration changes only PHASE 1:
      - skip Camera Module 3 lens sweep
      - load Q1~Q4 LensPosition from MANGO_PREVIOUS_LENS_JSON
      - continue with the normal projector ADC sweep / analysis / JSON output
    """
    old = """    # --------------------------------------------------------
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

    new = """    # --------------------------------------------------------
    # PHASE 1 : Camera Lens calibration OR previous-value reuse
    # --------------------------------------------------------
    _mango_previous_lens_json = os.environ.get(
        "MANGO_PREVIOUS_LENS_JSON",
        "",
    ).strip()

    if _mango_previous_lens_json:
        print()
        print("============================================")
        print(" PHASE 1 : CAMERA LENS CAL SKIPPED")
        print("============================================")
        print(
            "이전 Camera Lens calibration 값을 사용하고 "
            "Projector ADC calibration만 새로 수행합니다."
        )

        try:
            _mango_previous_lens = json.loads(
                _mango_previous_lens_json
            )
        except Exception as exc:
            raise RuntimeError(
                "이전 Camera Lens 값 JSON을 읽지 못했습니다: "
                + str(exc)
            )

        selected_camera_lens = {}
        all_calibration_runs = []

        for q in QUADRANT_ORDER:
            if q not in _mango_previous_lens:
                raise RuntimeError(
                    "이전 Camera Lens 값에 " + q + "가 없습니다."
                )

            item = _mango_previous_lens[q]

            try:
                lens_position = float(item["LensPosition"])
                reference_adc = int(
                    round(float(item["ReferenceADC"]))
                )
            except Exception:
                raise RuntimeError(
                    q + " 이전 Camera Lens 값 형식이 잘못되었습니다."
                )

            if not np.isfinite(lens_position):
                raise RuntimeError(
                    q + " 이전 Camera Lens 값이 유효하지 않습니다."
                )

            if not (
                CAMERA_LENS_MIN
                <= lens_position
                <= CAMERA_LENS_MAX
            ):
                raise RuntimeError(
                    f"{q} 이전 Camera Lens {lens_position:.4f} D가 "
                    f"현재 카메라 허용 범위 "
                    f"{CAMERA_LENS_MIN:.4f}~{CAMERA_LENS_MAX:.4f} D를 "
                    "벗어났습니다."
                )

            selected_camera_lens[q] = {
                "LensPosition": lens_position,
                "ReferenceADC": reference_adc,
                "Metric": str(
                    item.get("Metric")
                    or "REUSED_PREVIOUS"
                ),
            }

            print(
                f"{q} = {lens_position:.4f} D | "
                f"previous metric="
                f"{selected_camera_lens[q]['Metric']} | "
                f"previous refADC={reference_adc}"
            )

        print()
        print(
            "Camera Lens sweep 생략 완료 -> "
            "PHASE 2 Projector VCM sweep으로 진행합니다."
        )

        # IMPORTANT:
        # Do NOT overwrite camera_lens_calibration_summary_v4_2.csv here.
        # That file is the golden previous lens calibration used by this mode.
    else:
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

    if old not in source:
        raise RuntimeError(
            '내장 자동초점 코드에서 Camera Lens PHASE 1 블록을 찾지 못했습니다.'
        )

    patched = source.replace(old, new, 1)
    compile(
        patched,
        '<embedded-auto-focus-reuse-lens>',
        'exec',
    )
    return patched


def run_integrated_calibration(preview_path):
    """Run the embedded autofocus code as the camera/UART/GPIO22 owner."""
    reuse_previous_lens = (
        os.environ.get('MANGO_REUSE_PREVIOUS_LENS', '') == '1'
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
        if reuse_previous_lens:
            original_print(
                '[WEB-CAL] mode=PROJECTOR CAL ONLY · previous camera lens values',
                flush=True,
            )
            source = _autofocus_source_reusing_previous_lens(source)
            source_name = '<embedded-auto-focus-reuse-lens>'
        else:
            original_print(
                '[WEB-CAL] mode=FULL CAL · camera lens + projector',
                flush=True,
            )

        exec(compile(source, source_name, 'exec'), namespace, namespace)
        return 0
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
<title>Mango Comet · ADC Prism</title>
<style>
*{box-sizing:border-box}body{max-width:1080px;margin:32px auto;padding:0 20px;background:#15202c;color:#edf3f7;font:15px/1.6 system-ui}
h1{font-size:27px;margin-bottom:4px}h2{font-size:18px;margin-top:0}p{margin:8px 0 16px}.hint{color:#bac8d5;font-size:13px}
section{padding:22px;margin:20px 0;background:#1e2d3c;border:1px solid #34495b;border-radius:12px}
fieldset{border:0;padding:0;margin:0;min-width:0}.row{display:flex;gap:12px;align-items:end;flex-wrap:wrap}.grow{flex:1;min-width:220px}
input[type=text],input[type=number],select{display:block;width:100%;padding:10px;margin:5px 0;background:#111d28;color:inherit;border:1px solid #5e7182;border-radius:6px}
input[type=file]{display:block;margin:10px 0 18px;max-width:100%}button{padding:11px 17px;border:0;border-radius:6px;cursor:pointer;background:#b5e4dd;color:#142728;font-weight:600}
button:disabled{opacity:.45;cursor:default}#stop{background:#f4b8ac}.checks{margin:18px 0}.checks label{margin-right:20px}
.previews{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:14px}figure{margin:0}canvas{width:100%;height:auto;background:#000;display:block;border:1px solid #5e7182;border-radius:5px}figcaption{margin:6px 0;color:#d5e1eb}
.high{color:#ffca85}.low{color:#8ddcfa}#calCam{width:100%;max-width:900px;aspect-ratio:16/9;object-fit:contain;background:#000;border:1px solid #5e7182;border-radius:6px}#calRunLog{white-space:pre-wrap;background:#101922;padding:15px;max-height:260px;overflow:auto}#calibration{white-space:pre-line}#message{color:#ffdf91;white-space:pre-line;overflow-wrap:anywhere}#log{white-space:pre-wrap;background:#101922;padding:15px;max-height:220px;overflow:auto}#selection{overflow-wrap:anywhere}
@media(max-width:680px){.previews{grid-template-columns:1fr}body{padding:0 12px}section{padding:16px}}
#quick{border:2px solid #b5e4dd}.bigrow{display:grid;grid-template-columns:1fr;gap:14px}
@media(min-width:680px){.bigrow{grid-template-columns:repeat(3,1fr)}}
button.big{font-size:24px;padding:26px 10px;border-radius:14px;width:100%}
button.big.red{background:#f4b8ac}button.big.cal{background:#ffd98a}button.big.flip{background:#b5e4dd}
.quick-secondary{display:flex;justify-content:flex-end;margin-top:10px}
button.full-cal-small{font-size:13px;padding:8px 12px;border:1px solid #76a9dc;background:#2f6598;color:#f4f8fb;border-radius:7px;font-weight:600}
button.full-cal-small:hover{background:#3b76ad}
.qstate{font-size:20px;font-weight:700;margin:16px 0 6px}#qInfo{white-space:pre-line}#qMsg{color:#ffdf91;white-space:pre-line}
details#advanced>summary{cursor:pointer;font-size:17px;font-weight:600;margin:18px 0}
</style></head><body>
<h1>Mango Comet · ADC Prism</h1>
<section id="quick"><h2>빠른 제어</h2>
<div class="bigrow">
<button id="qStop" class="big red" type="button">■ 정지</button>
<button id="qCalReuse" class="big cal" type="button">◎ 캘리브레이션</button>
<button id="qFlip" class="big flip" type="button">▶ 플립</button>
</div>
<div class="quick-secondary">
<button id="qCalFull" class="full-cal-small" type="button">카메라 렌즈까지 다시 측정 · 풀 캘리브레이션</button>
</div>
<p id="qState" class="qstate">대기</p>
<p id="qInfo" class="hint"></p>
<p id="qMsg"></p>
</section>
<details id="advanced"><summary>고급 설정 · 기존 화면 (이미지 선택, 미리보기, 고정 모드)</summary>
<p class="hint">이미지 한 장을 중앙 수직선 기준 좌/우로만 이등분하고 HIGH / LOW 영역을 교대로 표시합니다.</p>
<section><h2>0. 자동 캘리브레이션</h2>
<p class="hint">캘리브레이션은 마지막으로 저장된 Q1~Q4 LensPosition을 그대로 사용하고 Camera Lens sweep만 생략한 뒤 Projector ADC sweep부터 새로 수행합니다. 카메라 위치가 바뀌었을 때만 풀 캘리브레이션으로 Camera Module 3 렌즈 위치(Q1~Q4)부터 다시 찾습니다. 두 방식 모두 완료되면 HIGH/LOW ADC JSON은 새 결과로 갱신됩니다.</p>
<div class="row">
<button id="calStartReuse" type="button">캘리브레이션</button>
<button id="calStartFull" class="full-cal-small" type="button">풀 캘리브레이션 · 렌즈 포함</button>
<button id="calStop" type="button" disabled>캘리브레이션 중지</button>
</div>
<p id="previousLensInfo" class="hint">이전 렌즈 캘리브레이션 값 확인 중…</p>
<p id="calRunState" class="hint">대기</p>
<img id="calCam" alt="캘리브레이션 카메라 프리뷰">
<pre id="calRunLog"></pre>
</section>
<section><h2>1. 이미지 불러오기</h2><fieldset id="sources">
<label for="upload">노트북에서 업로드</label>
<input id="upload" type="file" accept=".png,.jpg,.jpeg,.bmp,.webp">
<p class="hint">노트북 업로드 이미지는 실제 투사 방향 보정을 위해 저장 시 좌우반전 1회를 자동 적용합니다. 아래 미리보기는 보정 후 실제 출력 기준입니다.</p>
<div class="row"><label class="grow" for="directory">라즈베리파이 이미지 폴더
<input id="directory" type="text" value="{{ image_dir }}"></label><button id="list" type="button">목록 보기</button></div>
<div class="row"><select id="files" class="grow" aria-label="라즈베리파이 이미지 선택"></select>
<button id="load" type="button">선택한 이미지 불러오기</button></div>
</fieldset><p id="selection" class="hint">이미지를 선택하세요.</p></section>
<section><h2>2. 캘리브레이션과 분할 미리보기</h2>
<p class="hint">좌우 전용 분할입니다. 캘리브레이션의 ADC/사분면 값은 그대로 사용하되, 실제 투사 FRONT/BACK 이미지 슬라이스는 좌우를 서로 교환해서 표시합니다.</p>
<p id="calibration">캘리브레이션 결과 확인 중…</p><p id="next" class="hint"></p>
<div class="previews">
<figure><canvas id="whole" width="640" height="360"></canvas><figcaption>실제 전체 이미지 · 중앙 수직 분할선</figcaption></figure>
<figure><canvas id="high" width="640" height="360"></canvas><figcaption class="high">앞 FRONT · HIGH ADC · 교환된 실제 표시 슬라이스</figcaption></figure>
<figure><canvas id="low" width="640" height="360"></canvas><figcaption class="low">뒤 BACK · LOW ADC · 교환된 실제 표시 슬라이스</figcaption></figure>
</div><p class="hint">각 영역은 원래 위치를 유지합니다. 반대 영역은 검게 표시됩니다.</p></section>
<section><h2>3. 플립 실행</h2><form id="form"><fieldset id="settings">
<div class="row"><label class="grow">목표 이미지 플립 속도 (Hz)
<input id="fps" type="number" min="1" max="240" step="any" value="60" required></label>
<label class="grow">GPIO17 선행 시간 (ms)
<input id="gpioPhaseLeadMs" type="number" min="0" max="{{ gpio_phase_lead_ms_max }}" step="0.1" value="{{ gpio_phase_lead_ms_default }}" required></label>
<label class="grow">BACK 가짜타겟 보정 (ADC)
<input id="backFakeTargetOffset" type="number" min="0" max="{{ back_fake_target_offset_max }}" step="0.5" value="{{ back_fake_target_offset_default }}" required></label>
<label class="grow">실행 시간 (초, 0이면 계속)
<input id="seconds" type="number" min="0" max="86400" step="any" value="10" required></label></div>
<div class="checks"><label><input id="fullscreen" type="checkbox">전체화면</label>
<span><b>GPIO17 동기 신호: 항상 ON</b></span></div></fieldset>
<button id="start" disabled>플립 시작</button><button id="stop" type="button" disabled>이 앱의 출력 중지</button></form>
<p class="hint">친구의 실제 성공 기록과 동일하게 플립 시작 직전에 lightdm/display-manager를 OFF한 뒤 KMSDRM direct + VSync OFF + pygame Clock pacing을 사용합니다. GPIO17은 항상 사용하며 HIGH=FRONT(HIGH ADC), LOW=BACK(LOW ADC)입니다. BACK 가짜타겟 보정은 FLIP에서만 진짜 LOW/BACK ADC보다 낮은 값을 명령해 실제 BACK 초점점을 더 일찍 통과시키는 실험값입니다. 고정/캘리브레이션에는 적용되지 않습니다.</p>
<p id="status">대기</p>
<div id="timingBox" style="margin:12px 0;padding:12px 14px;background:#111d28;border:1px solid #5e7182;border-radius:7px">
<b>실시간 플립 측정 · 친구 known-good Clock 방식</b>
<div id="timing" class="hint" style="white-space:pre-line;margin-top:5px">출력 시작 후 약 1초 뒤 표시됩니다.</div>
</div>
<p id="message" role="status"></p><pre id="log"></pre></section>

<section><h2>4. 초점 하나만 고정해서 보기</h2>
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
  const disabled=running||calibrating||pending;
  $('sources').disabled=disabled;$('settings').disabled=disabled;
  $('start').disabled=disabled||!state?.image||!state?.calibration;
  $('stop').disabled=!running||pending;
  const previousLensReady=Boolean(state?.previous_lens_calibration?.available);
  $('calStartFull').disabled=pending||running||calibrating||holding;
  $('calStartReuse').disabled=pending||running||calibrating||holding||!previousLensReady;
  $('calStop').disabled=pending||!calibrating;
  const readyFixed=Boolean(state?.image&&state?.calibration);
  $('frontFixed').disabled=pending||running||calibrating||holding||!readyFixed;
  $('backFixed').disabled=pending||running||calibrating||holding||!readyFixed;
  $('focusRelease').disabled=pending||(!running&&!holding);
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
  const calModeLabel=cw.mode==='reuse_lens'?'캘리브레이션':'풀 캘리브레이션';
  $('calRunState').textContent=cw.running?
    `캘리브레이션 실행 중 · ${calModeLabel}`:(cw.last_result||'대기');
  $('calRunLog').textContent=cw.log||'';

  const previousLens=state.previous_lens_calibration||{};
  if(previousLens.available){
    const v=previousLens.values||{};
    $('previousLensInfo').textContent=
      '이전 렌즈값 사용 가능 · '+
      ['Q1','Q2','Q3','Q4'].map(q=>
        `${q} ${Number(v[q]?.LensPosition).toFixed(3)} D`
      ).join(' / ');
  }else{
    $('previousLensInfo').textContent=
      '이전 렌즈값 사용 불가 · '+(previousLens.error||'저장값 없음');
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
    `HIGH / 앞(FRONT): ${cal.high_side==='left'?'왼쪽':'오른쪽'} 슬라이스 · Q${cal.high.quadrant} · ADC ${cal.high.adc}\n`+
    `LOW / 뒤(BACK): ${cal.low_side==='left'?'왼쪽':'오른쪽'} 슬라이스 · Q${cal.low.quadrant} · ADC ${cal.low.adc}\n`+
    `분할: ${cal.label}`:state.calibration_error;
  const changed=state.running&&JSON.stringify(state.calibration)!==JSON.stringify(state.active_calibration);
  $('next').textContent=state.running&&!state.calibration?
    `현재 출력은 시작할 때의 값을 유지합니다. ${state.calibration_error}`:
    changed?'현재 출력은 시작할 때의 값을 유지합니다. 새 캘리브레이션 결과는 다음 시작에 적용됩니다.':'';
  $('selection').textContent=state.image?
    `${state.image.name} · ${state.image.width} × ${state.image.height}`+
    (state.image.mirror_corrected?' · 노트북 업로드 좌우반전 보정 적용':''):
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
  else if(running) text='화면 출력 중';
  else if(holding) text='초점 고정 중';
  else if(cw.last_result) text=cw.last_result;
  $('qState').textContent=text;
  const info=[];
  info.push(cal?
    `기억된 BEST ADC · HIGH ${Number(cal.high.adc).toFixed(1)} (Q${cal.high.quadrant}) / LOW ${Number(cal.low.adc).toFixed(1)} (Q${cal.low.quadrant})`:
    'BEST ADC 없음 · 캘리브레이션을 먼저 하세요');
  info.push(state.image?`플립 이미지: ${state.image.name}`:'플립 이미지 없음 · 아래 고급 설정 1번에서 한 번만 선택하세요');
  const previousLens=state.previous_lens_calibration||{};
  if(previousLens.available){
    const v=previousLens.values||{};
    info.push(
      '이전 카메라 렌즈값 · '+
      ['Q1','Q2','Q3','Q4'].map(q=>
        `${q} ${Number(v[q]?.LensPosition).toFixed(3)}D`
      ).join(' / ')
    );
  }else{
    info.push('이전 카메라 렌즈값 없음 · 풀 캘리브레이션을 먼저 실행하세요');
  }
  $('qInfo').textContent=info.join('\n');
  const busy=pending||calibrating||running||holding;
  $('qCalFull').disabled=busy;
  $('qCalReuse').disabled=busy||!previousLens.available;
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
$('calStartFull').onclick=()=>action(async()=>{
  await call('/calibration/start',{});
  $('message').textContent='풀 캘리브레이션 시작 · Camera Lens + Projector ADC를 모두 새로 측정합니다.';
});
$('calStartReuse').onclick=()=>action(async()=>{
  await call('/calibration/start-reuse-lens',{});
  $('message').textContent='캘리브레이션 시작 · 저장된 Camera Lens 값을 사용하고 Projector ADC를 새로 측정합니다.';
});
$('calStop').onclick=()=>action(async()=>{
  await call('/calibration/stop',{});
  $('message').textContent='캘리브레이션 중지 요청을 보냈습니다.';
});
$('qStop').onclick=()=>action(async()=>{
  const result=await call('/all-stop',{});
  $('qMsg').textContent='전체 정지 · ATmega: '+(result.uart||'');
});
$('qCalFull').onclick=()=>action(async()=>{
  await call('/calibration/start',{});
  $('qMsg').textContent='풀 캘리브레이션 시작 · 렌즈 캘리브레이션부터 새로 측정합니다.';
});
$('qCalReuse').onclick=()=>action(async()=>{
  await call('/calibration/start-reuse-lens',{});
  $('qMsg').textContent='캘리브레이션 시작 · 저장된 Camera Lens 값을 사용하고 Projector ADC를 새로 측정합니다.';
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



def friend_stop_display_managers():
    """
    친구 app.py 성공 로그와 같은 순서로 GUI를 내린다.
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


def friend_restore_display_manager():
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
    assets = {}
    active_calibration = None
    flip_metrics = {}
    output_mode = 'idle'
    display_manager_stopped = False

    # Integrated calibration child state.
    cal_worker = None
    cal_log = deque(maxlen=800)
    cal_last_result = ''
    cal_started_mtime_ns = None
    cal_preparing = False                 # checkerboard being shown before calibration
    cal_mode = 'full'                     # 'full' or 'reuse_lens'
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

    def import_image(stream, name, mirror_horizontal=False):
        nonlocal selected
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
        }
        # Remember the flip image so the FLIP button works after a restart.
        try:
            WEB_STATE_FILE.write_text(
                json.dumps({'image_path': str(path), 'selected': selected}),
                encoding='utf-8',
            )
        except Exception as exc:
            print('[WEB] could not save image choice:', exc, flush=True)
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

    def read_calibration_log(process):
        nonlocal cal_last_result
        for line in process.stdout:
            cal_log.append(line)
            print('[CAL] ' + line, end='', flush=True)
        try:
            process.stdout.close()
        except Exception:
            pass
        code = process.poll()
        try:
            stat = CALIBRATION_FILE.stat()
            refreshed = cal_started_mtime_ns is None or stat.st_mtime_ns > cal_started_mtime_ns
            if refreshed:
                result = read_calibration()
                cal_last_result = ('완료 · HIGH Q{hq} ADC {ha:.2f} / LOW Q{lq} ADC {la:.2f}').format(
                    hq=result['high']['quadrant'], ha=float(result['high']['adc']),
                    lq=result['low']['quadrant'], la=float(result['low']['adc']))
            elif code == 0:
                cal_last_result = '종료됨 · 새 calibration JSON은 확인되지 않았습니다.'
            else:
                cal_last_result = f'캘리브레이션 종료 코드 {code}'
        except Exception as exc:
            cal_last_result = f'캘리브레이션 종료 · 결과 확인 실패: {exc}'

        # Checkerboard off + GUI back, once calibration has finished.
        if output_mode == 'calibration_display':
            stop_worker()

    def stop_worker():
        nonlocal worker, display_manager_stopped, output_mode
        with lock:
            if worker is None or worker.poll() is not None:
                if display_manager_stopped:
                    friend_restore_display_manager()
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
                friend_restore_display_manager()
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

            friend_restore_display_manager()
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
            previous_lens = previous_camera_lens_status()
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
                           previous_lens_calibration=previous_lens,
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
        dm_results = friend_stop_display_managers()
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

    def calibration_launch_job(mode_name='full', previous_lens=None):
        """background: checkerboard on -> embedded autofocus child"""
        nonlocal cal_worker, cal_preparing, cal_last_result
        try:
            image = Path(CALIBRATION_IMAGE)
            if CALIBRATION_IMAGE and image.is_file():
                with lock:
                    if not cal_preparing:
                        return
                    cal_display_ready.clear()
                    cal_last_result = (
                        '체커보드 표시 중… · 이전 렌즈값 사용'
                        if mode_name == 'reuse_lens'
                        else '체커보드 표시 중… · 풀 캘리브레이션'
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

                if mode_name == 'reuse_lens':
                    if not previous_lens:
                        raise RuntimeError(
                            '이전 Camera Lens 값 snapshot이 없습니다.'
                        )
                    env['MANGO_REUSE_PREVIOUS_LENS'] = '1'
                    env['MANGO_PREVIOUS_LENS_JSON'] = json.dumps(
                        previous_lens,
                        ensure_ascii=False,
                        separators=(',', ':'),
                    )
                else:
                    env.pop('MANGO_REUSE_PREVIOUS_LENS', None)
                    env.pop('MANGO_PREVIOUS_LENS_JSON', None)

                command = [sys.executable, '-s', '-B', '-u', str(Path(__file__).resolve()),
                           '--mango-calibrate', str(CAL_PREVIEW_PATH)]
                cal_worker = subprocess.Popen(
                    command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    stdin=subprocess.DEVNULL, text=True, bufsize=1, env=env,
                    cwd=str(USER_HOME), start_new_session=True,
                )
                cal_last_result = (
                    '캘리브레이션 실행 중… · Camera Lens 이전값 사용'
                    if mode_name == 'reuse_lens'
                    else '캘리브레이션 실행 중… · Camera Lens 포함'
                )
                cal_preparing = False
            threading.Thread(target=read_calibration_log, args=(cal_worker,), daemon=True).start()
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
        nonlocal cal_worker, cal_last_result, cal_started_mtime_ns
        nonlocal cal_preparing, cal_mode

        with lock:
            try:
                if closing:
                    raise ValueError('앱을 종료하는 중입니다.')
                if is_running():
                    raise ValueError('플립 출력 중에는 캘리브레이션을 시작할 수 없습니다.')
                if is_calibrating():
                    raise ValueError('자동 캘리브레이션이 이미 실행 중입니다.')
                if hold_thread is not None and hold_thread.is_alive():
                    raise ValueError('고정 모드를 먼저 해제하세요.')

                owner = uart_owner_process()
                if owner:
                    raise ValueError(
                        'UART가 다른 프로그램에서 사용 중입니다. '
                        '기존 자동초점/제어 프로그램을 종료하세요.\n'
                        + owner
                    )

                previous_lens = None

                if mode_name == 'reuse_lens':
                    # Snapshot now. The child receives these exact values,
                    # so a file change during calibration cannot alter them.
                    previous_lens = read_previous_camera_lens_values()

                try:
                    cal_started_mtime_ns = CALIBRATION_FILE.stat().st_mtime_ns
                except FileNotFoundError:
                    cal_started_mtime_ns = None

                try:
                    CAL_PREVIEW_PATH.unlink()
                except FileNotFoundError:
                    pass

                cal_log.clear()
                cal_mode = mode_name
                cal_last_result = (
                    '시작 중… · 이전 Camera Lens 값 사용'
                    if mode_name == 'reuse_lens'
                    else '시작 중… · 풀 캘리브레이션'
                )
                cal_preparing = True

                threading.Thread(
                    target=calibration_launch_job,
                    args=(mode_name, previous_lens),
                    daemon=True,
                ).start()

                return jsonify(
                    ok=True,
                    mode=mode_name,
                    previous_lens=previous_lens,
                )

            except Exception as exc:
                return jsonify(error=str(exc)), 400

    @app.post('/calibration/start')
    def calibration_start():
        return begin_calibration('full')

    @app.post('/calibration/start-reuse-lens')
    def calibration_start_reuse_lens():
        return begin_calibration('reuse_lens')

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
                        'BACK 가짜타겟 보정은 '
                        f'0~{BACK_FAKE_TARGET_OFFSET_MAX:g} ADC 범위로 입력하세요.'
                    )
                back_fake_target_offset = float(back_fake_target_offset)

                if type(data.get('fullscreen')) is not bool:
                    raise ValueError('전체화면 체크박스 설정이 잘못되었습니다.')

                # 이 프로젝트에서는 이미지 플립과 VCM 전환이 항상 같이 가므로
                # GPIO17 동기 신호를 선택 옵션으로 두지 않는다.
                config.update(fullscreen=data['fullscreen'], gpio=True)

                # 친구의 실제 성공 로그와 동일:
                # KMSDRM child 실행 전에 lightdm + display-manager를 먼저 STOP.
                dm_results = friend_stop_display_managers()
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
                        'BACK 가짜타겟이 ATmega 최소 ADC 100보다 작아집니다. '
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

                # 친구 app.py/image_flip.py와 같은 KMS child 환경.
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
                    friend_restore_display_manager()
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

                dm_results = friend_stop_display_managers()
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
                    friend_restore_display_manager()
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
        '[KMS] friend-known-good mode: '
        'child=sudo -n -E python3 / kmsdrm / vsync OFF / clock pacing',
        flush=True,
    )
    print(
        '[KMS] 친구 성공 로그대로 플립 시작 전 '
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