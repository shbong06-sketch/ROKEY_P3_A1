from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch
from ultralytics import YOLO


EXPECTED_CLASS_NAMES = {
    0: "lettuce_dark_green",
    1: "lettuce_yellow",
    2: "lettuce_brown",
}


class DetectorError(RuntimeError):
    """YOLO detector 공통 오류."""


class ModelLoadError(DetectorError):
    """모델 파일을 읽거나 초기화하지 못한 경우."""


class ModelClassMismatchError(ModelLoadError):
    """모델 클래스 정의가 검사 모델 계약과 다른 경우."""


class DeviceSelectionError(DetectorError):
    """device 설정이 잘못되었거나 존재하지 않는 경우."""


class CudaUnavailableError(DeviceSelectionError):
    """CUDA 요청을 충족할 수 없는 경우."""


class InvalidImageError(DetectorError):
    """추론 입력 이미지가 요구 형식과 다른 경우."""


class InferenceError(DetectorError):
    """모델 추론 도중 오류가 발생한 경우."""


@dataclass(frozen=True)
class DetectorConfig:
    """YOLO detector 초기화 및 추론 설정."""

    model_path: str | Path
    device: str = "cuda:0"
    confidence_threshold: float = 0.5
    iou_threshold: float = 0.45
    image_size: int = 640
    class_filter: tuple[int, ...] | None = (0, 1, 2)
    allow_cpu_fallback: bool = False


@dataclass(frozen=True)
class Detection:
    """Ultralytics 객체와 독립적인 단일 검출 결과."""

    class_id: int
    class_name: str
    confidence: float
    bbox_xyxy: tuple[float, float, float, float]


class YoloDetector:
    """YOLO 모델을 한 번 로드하고 반복 추론한다."""

    def __init__(self, config: DetectorConfig) -> None:
        self._validate_config(config)

        model_path = Path(config.model_path).expanduser()
        if not model_path.is_file():
            raise FileNotFoundError(f"YOLO model not found: {model_path}")

        self.config = config
        self.model_path = model_path
        self.device = self._resolve_device(
            config.device,
            config.allow_cpu_fallback,
        )

        try:
            self._model = YOLO(str(model_path))
        except Exception as exc:
            raise ModelLoadError(
                f"Failed to load YOLO model: {model_path}"
            ) from exc

        self.class_names = self._normalize_class_names(
            getattr(self._model, "names", None)
        )
        self._validate_model_classes(self.class_names)

    def detect(self, image: np.ndarray) -> list[Detection]:
        """OpenCV BGR 이미지를 추론하고 내부 검출 구조로 반환한다."""
        self._validate_image(image)

        try:
            results = self._model.predict(
                source=image,
                device=self.device,
                conf=self.config.confidence_threshold,
                iou=self.config.iou_threshold,
                imgsz=self.config.image_size,
                classes=self.config.class_filter,
                verbose=False,
            )
        except Exception as exc:
            raise InferenceError("YOLO inference failed") from exc

        if not results:
            return []

        result = results[0]
        boxes = result.boxes

        if boxes is None or len(boxes) == 0:
            return []

        xyxy_values = boxes.xyxy.detach().cpu().tolist()
        confidence_values = boxes.conf.detach().cpu().tolist()
        class_values = boxes.cls.detach().cpu().tolist()

        detections: list[Detection] = []

        for xyxy, confidence, class_value in zip(
            xyxy_values,
            confidence_values,
            class_values,
        ):
            class_id = int(class_value)

            try:
                class_name = self.class_names[class_id]
            except KeyError as exc:
                raise InferenceError(
                    f"Unknown class id returned by model: {class_id}"
                ) from exc

            detections.append(
                Detection(
                    class_id=class_id,
                    class_name=class_name,
                    confidence=float(confidence),
                    bbox_xyxy=tuple(float(value) for value in xyxy),
                )
            )

        return detections

    @staticmethod
    def _validate_config(config: DetectorConfig) -> None:
        if not 0.0 < config.confidence_threshold <= 1.0:
            raise ValueError(
                "confidence_threshold must be greater than 0 and at most 1"
            )

        if not 0.0 <= config.iou_threshold <= 1.0:
            raise ValueError(
                "iou_threshold must be between 0 and 1"
            )

        if config.image_size <= 0:
            raise ValueError("image_size must be greater than 0")

        if config.class_filter is not None:
            invalid_classes = (
                set(config.class_filter) - set(EXPECTED_CLASS_NAMES)
            )
            if invalid_classes:
                raise ValueError(
                    f"Unsupported class ids: {sorted(invalid_classes)}"
                )

    @staticmethod
    def _resolve_device(
        requested_device: str,
        allow_cpu_fallback: bool,
    ) -> str:
        device = requested_device.strip().lower()

        if device == "cpu":
            return "cpu"

        match = re.fullmatch(r"cuda(?::(\d+))?", device)
        if match is None:
            raise DeviceSelectionError(
                "device must be 'cpu', 'cuda', or 'cuda:N'"
            )

        device_index = int(match.group(1) or 0)

        if not torch.cuda.is_available():
            if allow_cpu_fallback:
                return "cpu"

            raise CudaUnavailableError(
                f"CUDA requested but unavailable: cuda:{device_index}"
            )

        device_count = torch.cuda.device_count()
        if device_index >= device_count:
            raise DeviceSelectionError(
                f"CUDA device cuda:{device_index} does not exist; "
                f"available device count: {device_count}"
            )

        return f"cuda:{device_index}"

    @staticmethod
    def _validate_image(image: np.ndarray) -> None:
        if not isinstance(image, np.ndarray):
            raise InvalidImageError(
                "image must be a numpy.ndarray"
            )

        if image.size == 0:
            raise InvalidImageError("image must not be empty")

        if image.ndim != 3 or image.shape[2] != 3:
            raise InvalidImageError(
                "image must have HxWx3 BGR shape"
            )

        if image.dtype != np.uint8:
            raise InvalidImageError(
                "image dtype must be uint8"
            )

    @staticmethod
    def _normalize_class_names(names: Any) -> dict[int, str]:
        if isinstance(names, dict):
            return {
                int(class_id): str(class_name)
                for class_id, class_name in names.items()
            }

        if isinstance(names, (list, tuple)):
            return {
                class_id: str(class_name)
                for class_id, class_name in enumerate(names)
            }

        raise ModelLoadError(
            "YOLO model does not provide a valid class-name mapping"
        )

    @staticmethod
    def _validate_model_classes(
        class_names: dict[int, str],
    ) -> None:
        if class_names != EXPECTED_CLASS_NAMES:
            raise ModelClassMismatchError(
                "Unexpected YOLO classes: "
                f"expected={EXPECTED_CLASS_NAMES}, actual={class_names}"
            )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run one smart-farm YOLO image inference."
    )
    parser.add_argument(
        "--model",
        required=True,
        help="YOLO .pt model path",
    )
    parser.add_argument(
        "--image",
        required=True,
        help="Input image path",
    )
    parser.add_argument(
        "--device",
        default="cuda:0",
        help="cpu, cuda, or cuda:N",
    )
    parser.add_argument(
        "--confidence",
        type=float,
        default=0.5,
    )
    parser.add_argument(
        "--iou",
        type=float,
        default=0.45,
    )
    parser.add_argument(
        "--image-size",
        type=int,
        default=640,
    )
    parser.add_argument(
        "--classes",
        nargs="+",
        type=int,
        default=[0, 1, 2],
    )
    parser.add_argument(
        "--allow-cpu-fallback",
        action="store_true",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()

    try:
        detector = YoloDetector(
            DetectorConfig(
                model_path=args.model,
                device=args.device,
                confidence_threshold=args.confidence,
                iou_threshold=args.iou,
                image_size=args.image_size,
                class_filter=tuple(args.classes),
                allow_cpu_fallback=args.allow_cpu_fallback,
            )
        )

        image_path = Path(args.image).expanduser()
        if not image_path.is_file():
            raise FileNotFoundError(
                f"Input image not found: {image_path}"
            )

        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise InvalidImageError(
                f"OpenCV failed to read image: {image_path}"
            )

        detections = detector.detect(image)
        print(
            json.dumps(
                [asdict(detection) for detection in detections],
                ensure_ascii=False,
                indent=2,
            )
        )
        return 0

    except (
        DetectorError,
        FileNotFoundError,
        ValueError,
    ) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
