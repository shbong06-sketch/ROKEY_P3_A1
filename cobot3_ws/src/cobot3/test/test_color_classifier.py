"""color_classifier 를 합성 이미지로 검증한다.

아이작심 없이 돌아간다. 큐브가 카메라에 안 보여도 판정 로직을 확인할 수 있다.
"""

import numpy as np

from cobot3.color_classifier import BLUE_ID, GREEN_ID, NONE_ID, classify

WHITE = (255, 255, 255)
BLUE = (255, 0, 0)     # BGR
GREEN = (0, 255, 0)    # BGR


def make_image(color=None):
    """흰 배경 640x640 에 100x100 사각형을 하나 그린다."""
    image = np.full((640, 640, 3), WHITE, dtype=np.uint8)
    if color is not None:
        image[270:370, 270:370] = color
    return image


def test_파란_사각형은_1():
    color_id, blue_area, green_area = classify(make_image(BLUE))
    assert color_id == BLUE_ID
    assert blue_area == 100 * 100
    assert green_area == 0


def test_초록_사각형은_2():
    color_id, blue_area, green_area = classify(make_image(GREEN))
    assert color_id == GREEN_ID
    assert green_area == 100 * 100
    assert blue_area == 0


def test_흰_화면은_0():
    color_id, blue_area, green_area = classify(make_image())
    assert color_id == NONE_ID
    assert blue_area == 0
    assert green_area == 0


def test_작은_점은_노이즈로_보고_0():
    image = np.full((640, 640, 3), WHITE, dtype=np.uint8)
    image[300:310, 300:310] = GREEN      # 100 픽셀. MIN_AREA 300 미만
    color_id, _, green_area = classify(image)
    assert color_id == NONE_ID
    assert green_area == 100
