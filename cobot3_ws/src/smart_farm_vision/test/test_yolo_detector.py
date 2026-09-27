"""YOLO detector의 모델 독립 단위 테스트."""

import numpy as np
import pytest
import torch

import smart_farm_vision.yolo_detector as detector_module
from smart_farm_vision.yolo_detector import (
    CudaUnavailableError,
    DetectorConfig,
    DeviceSelectionError,
    InvalidImageError,
    ModelClassMismatchError,
    ModelLoadError,
    YoloDetector,
)


EXPECTED_NAMES = {
    0: "lettuce_dark_green",
    1: "lettuce_yellow",
    2: "lettuce_brown",
}


class FakeBoxes:
    """Ultralytics Boxes에서 사용하는 최소 속성만 제공한다."""

    def __init__(self) -> None:
        self.xyxy = torch.tensor(
            [
                [10.0, 20.0, 110.0, 220.0],
                [300.0, 100.0, 500.0, 400.0],
            ]
        )
        self.conf = torch.tensor([0.9, 0.75])
        self.cls = torch.tensor([0.0, 2.0])

    def __len__(self) -> int:
        return len(self.conf)


class FakeResult:
    """Ultralytics Result에서 사용하는 최소 속성만 제공한다."""

    def __init__(self) -> None:
        self.boxes = FakeBoxes()


class FakeModel:
    """모델 로드 및 predict 호출 횟수를 검사하기 위한 fake."""

    names = EXPECTED_NAMES

    def __init__(self) -> None:
        self.predict_calls = []

    def predict(self, **kwargs):
        self.predict_calls.append(kwargs)
        return [FakeResult()]


def create_model_file(tmp_path):
    """YOLO factory를 mock할 때 사용할 빈 모델 파일을 만든다."""
    model_path = tmp_path / "best.pt"
    model_path.touch()
    return model_path


def test_missing_model_raises_file_not_found(tmp_path):
    with pytest.raises(FileNotFoundError):
        YoloDetector(
            DetectorConfig(
                model_path=tmp_path / "missing.pt",
                device="cpu",
            )
        )


def test_invalid_device_raises_device_error(tmp_path):
    model_path = create_model_file(tmp_path)

    with pytest.raises(DeviceSelectionError):
        YoloDetector(
            DetectorConfig(
                model_path=model_path,
                device="gpu",
            )
        )


def test_cuda_unavailable_without_fallback_fails(
    tmp_path,
    monkeypatch,
):
    model_path = create_model_file(tmp_path)
    monkeypatch.setattr(
        detector_module.torch.cuda,
        "is_available",
        lambda: False,
    )

    with pytest.raises(CudaUnavailableError):
        YoloDetector(
            DetectorConfig(
                model_path=model_path,
                device="cuda:0",
                allow_cpu_fallback=False,
            )
        )


def test_cuda_unavailable_with_fallback_uses_cpu(
    tmp_path,
    monkeypatch,
):
    model_path = create_model_file(tmp_path)
    fake_model = FakeModel()

    monkeypatch.setattr(
        detector_module.torch.cuda,
        "is_available",
        lambda: False,
    )
    monkeypatch.setattr(
        detector_module,
        "YOLO",
        lambda _: fake_model,
    )

    detector = YoloDetector(
        DetectorConfig(
            model_path=model_path,
            device="cuda:0",
            allow_cpu_fallback=True,
        )
    )

    assert detector.device == "cpu"


def test_model_load_failure_is_distinguished(
    tmp_path,
    monkeypatch,
):
    model_path = create_model_file(tmp_path)

    def fail_to_load(_):
        raise RuntimeError("invalid model")

    monkeypatch.setattr(
        detector_module,
        "YOLO",
        fail_to_load,
    )

    with pytest.raises(ModelLoadError):
        YoloDetector(
            DetectorConfig(
                model_path=model_path,
                device="cpu",
            )
        )


def test_unexpected_model_classes_are_rejected(
    tmp_path,
    monkeypatch,
):
    model_path = create_model_file(tmp_path)
    fake_model = FakeModel()
    fake_model.names = {
        0: "person",
        1: "car",
    }

    monkeypatch.setattr(
        detector_module,
        "YOLO",
        lambda _: fake_model,
    )

    with pytest.raises(ModelClassMismatchError):
        YoloDetector(
            DetectorConfig(
                model_path=model_path,
                device="cpu",
            )
        )


@pytest.mark.parametrize(
    "invalid_image",
    [
        None,
        np.array([], dtype=np.uint8),
        np.zeros((10, 10), dtype=np.uint8),
        np.zeros((10, 10, 4), dtype=np.uint8),
        np.zeros((10, 10, 3), dtype=np.float32),
    ],
)
def test_invalid_image_is_rejected(
    invalid_image,
    tmp_path,
    monkeypatch,
):
    model_path = create_model_file(tmp_path)

    monkeypatch.setattr(
        detector_module,
        "YOLO",
        lambda _: FakeModel(),
    )

    detector = YoloDetector(
        DetectorConfig(
            model_path=model_path,
            device="cpu",
        )
    )

    with pytest.raises(InvalidImageError):
        detector.detect(invalid_image)


def test_model_is_loaded_once_and_results_are_converted(
    tmp_path,
    monkeypatch,
):
    model_path = create_model_file(tmp_path)
    created_models = []

    def create_fake_model(_):
        model = FakeModel()
        created_models.append(model)
        return model

    monkeypatch.setattr(
        detector_module,
        "YOLO",
        create_fake_model,
    )

    detector = YoloDetector(
        DetectorConfig(
            model_path=model_path,
            device="cpu",
            confidence_threshold=0.5,
            iou_threshold=0.45,
            image_size=640,
            class_filter=(0, 1, 2),
        )
    )

    image = np.zeros((720, 1280, 3), dtype=np.uint8)

    first_detections = detector.detect(image)
    second_detections = detector.detect(image)

    assert len(created_models) == 1
    assert len(created_models[0].predict_calls) == 2

    assert len(first_detections) == 2
    assert len(second_detections) == 2

    first = first_detections[0]
    assert first.class_id == 0
    assert first.class_name == "lettuce_dark_green"
    assert first.confidence == pytest.approx(0.9)
    assert first.bbox_xyxy == pytest.approx(
        (10.0, 20.0, 110.0, 220.0)
    )

    second = first_detections[1]
    assert second.class_id == 2
    assert second.class_name == "lettuce_brown"
    assert second.confidence == pytest.approx(0.75)
    assert second.bbox_xyxy == pytest.approx(
        (300.0, 100.0, 500.0, 400.0)
    )

    predict_call = created_models[0].predict_calls[0]
    assert predict_call["source"] is image
    assert predict_call["device"] == "cpu"
    assert predict_call["conf"] == 0.5
    assert predict_call["iou"] == 0.45
    assert predict_call["imgsz"] == 640
    assert predict_call["classes"] == (0, 1, 2)
    assert predict_call["verbose"] is False
