from picamera2 import Picamera2
import cv2
import time
import numpy as np
from collections import deque

WIDTH = 1280
HEIGHT = 720

# 최근 9프레임 저장
MEDIAN_FRAMES = 9
score_buffer = deque(maxlen=MEDIAN_FRAMES)

picam2 = Picamera2()

config = picam2.create_video_configuration(
    main={
        "size": (WIDTH, HEIGHT),
        "format": "RGB888"
    }
)

picam2.configure(config)
picam2.start()

time.sleep(2)

print("Focus Score 측정 시작")
print("최근 9프레임 중앙값 실시간 계산")
print("q를 누르면 종료")

while True:
    frame = picam2.capture_array()

    # RGB -> Gray
    gray = cv2.cvtColor(
        frame,
        cv2.COLOR_RGB2GRAY
    )

    h, w = gray.shape

    # 중앙 50% 영역
    x1 = w // 4
    x2 = w * 3 // 4
    y1 = h // 4
    y2 = h * 3 // 4

    roi = gray[y1:y2, x1:x2]

    # 현재 프레임 Laplacian Focus Score
    raw_score = cv2.Laplacian(
        roi,
        cv2.CV_64F
    ).var()

    # 최근 score 저장
    score_buffer.append(raw_score)

    # 최근 프레임들의 중앙값
    median_score = np.median(score_buffer)

    # ROI 박스 표시
    cv2.rectangle(
        frame,
        (x1, y1),
        (x2, y2),
        (0, 255, 0),
        3
    )

    # 현재 1프레임 Raw Score
    cv2.putText(
        frame,
        f"Raw Score : {raw_score:.1f}",
        (30, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (0, 255, 0),
        2
    )

    # 최근 9프레임 중앙값
    cv2.putText(
        frame,
        f"Median(9) : {median_score:.1f}",
        (30, 110),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (0, 255, 255),
        2
    )

    # 화면 표시
    display = cv2.cvtColor(
        frame,
        cv2.COLOR_RGB2BGR
    )

    cv2.imshow(
        "Focus Monitor",
        display
    )

    # 터미널 출력
    print(
        f"\rRaw : {raw_score:.1f} | "
        f"Median(9) : {median_score:.1f}",
        end="",
        flush=True
    )

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

picam2.stop()
cv2.destroyAllWindows()
