"""BGR 이미지에서 파랑/초록을 찾아 색 ID 를 돌려준다.

ROS 에 의존하지 않는다. 이미지 한 장만 있으면 테스트할 수 있다.
"""

import cv2
import numpy as np

NONE_ID = 0
BLUE_ID = 1
GREEN_ID = 2

# OpenCV 의 H 범위는 0~179 다 (0~360 아님)
BLUE_LOW = np.array([100, 80, 50])
BLUE_HIGH = np.array([130, 255, 255])

GREEN_LOW = np.array([40, 80, 50])
GREEN_HIGH = np.array([85, 255, 255])

# 이보다 작으면 노이즈로 보고 미감지 처리한다
MIN_AREA = 300


def classify(bgr_image):
    """(color_id, blue_area, green_area) 를 돌려준다."""
    hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

    blue_area = int(cv2.countNonZero(cv2.inRange(hsv, BLUE_LOW, BLUE_HIGH)))
    green_area = int(cv2.countNonZero(cv2.inRange(hsv, GREEN_LOW, GREEN_HIGH)))

    if max(blue_area, green_area) < MIN_AREA:
        return NONE_ID, blue_area, green_area

    if blue_area >= green_area:
        return BLUE_ID, blue_area, green_area

    return GREEN_ID, blue_area, green_area
