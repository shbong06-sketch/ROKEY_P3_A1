"""[올인원 2026-09-25] best.pt 추론 전용 하위 프로세스.

Isaac Sim 파이썬 안에 ultralytics 를 섞으면 numpy·torch 판이 부딪혀서, 추론만 따로 띄운다.
inspection_cull_station.py 가 이 파일을 한 번 띄워 두고 한 줄씩 주고받는다.

    python inspection_yolo_worker.py WEIGHTS [CONF]
    stdin  한 줄: {"npy": 프레임(.npy, RGB 또는 RGBA), "annotated": 결과 그림 경로 | null, "raw": 원본 그림 경로 | null}
    stdout 한 줄: {"detections": [{"class_name", "conf", "xyxy"}]}
첫 줄로 {"ready": true, "names": {...}} 를 낸다.
"""
import json
import sys

import numpy as np
from ultralytics import YOLO


def main():
    model = YOLO(sys.argv[1])
    conf = float(sys.argv[2]) if len(sys.argv) > 2 else 0.25
    model.predict(np.zeros((640, 640, 3), np.uint8), imgsz=640, conf=conf, verbose=False)   # 첫 추론 지연을 미리 치른다
    print(json.dumps({"ready": True, "names": {int(k): v for k, v in model.names.items()}}), flush=True)
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            request = json.loads(line)
            frame = np.load(request["npy"])
            bgr = np.ascontiguousarray(frame[..., :3][..., ::-1])
            result = model.predict(bgr, imgsz=640, conf=conf, verbose=False)[0]
            detections = [
                {"class_name": model.names[int(c)], "conf": round(float(s), 4), "xyxy": [round(float(v), 1) for v in box]}
                for box, s, c in zip(result.boxes.xyxy.tolist(), result.boxes.conf.tolist(), result.boxes.cls.tolist())
            ]
            if request.get("annotated") or request.get("raw"):
                import cv2
                if request.get("annotated"):
                    cv2.imwrite(request["annotated"], result.plot())
                if request.get("raw"):
                    cv2.imwrite(request["raw"], bgr)
            print(json.dumps({"detections": detections}), flush=True)
        except Exception as error:  # noqa: BLE001 - 한 장 실패로 워커가 죽지 않게
            print(json.dumps({"error": str(error), "detections": []}), flush=True)


if __name__ == "__main__":
    main()
