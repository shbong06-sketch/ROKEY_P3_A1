"""Score one 96x96 cabbage crop from an MP4 with the trained PatchCore model."""

import argparse
import io
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import PIL.Image
import PIL.ImageDraw
import torch
from torchvision import transforms


MODEL_DIR = Path(__file__).resolve().parents[1] / 'models' / 'patchcore'
THRESHOLD = 0.32702351  # validation balanced-accuracy threshold


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('video', type=Path)
    parser.add_argument('--time', type=float, required=True, help='seconds from video start')
    parser.add_argument('--center', type=int, nargs=2, metavar=('X', 'Y'),
                        required=True, help='cabbage center in the 640x640 frame')
    parser.add_argument('--output', type=Path, default=Path('results/videos/scored_frame.png'))
    args = parser.parse_args()
    if args.time < 0 or not args.video.is_file():
        parser.error('video must exist and --time must be nonnegative')

    frame_bytes = subprocess.check_output([
        'ffmpeg', '-v', 'error', '-ss', str(args.time), '-i', str(args.video),
        '-frames:v', '1', '-f', 'image2pipe', '-vcodec', 'png', 'pipe:1',
    ])
    if not frame_bytes:
        parser.error('no frame at the requested time')
    frame = PIL.Image.open(io.BytesIO(frame_bytes)).convert('RGB')
    x, y = args.center
    box = (x - 48, y - 48, x + 48, y + 48)
    if box[0] < 0 or box[1] < 0 or box[2] > frame.width or box[3] > frame.height:
        parser.error('96x96 crop extends outside the frame')
    crop = frame.crop(box)

    metadata = json.loads((MODEL_DIR / 'training_metadata.json').read_text())
    sys.path.insert(0, str(Path(metadata['patchcore_repo']) / 'src'))
    import patchcore.common
    import patchcore.patchcore

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = patchcore.patchcore.PatchCore(device)
    model.load_from_path(str(MODEL_DIR), device,
                         nn_method=patchcore.common.FaissNN(False, 4))
    transform = transforms.Compose([
        transforms.Resize(96), transforms.CenterCrop(96), transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406],
                             std=[0.229, 0.224, 0.225]),
    ])
    scores, _ = model.predict(transform(crop).unsqueeze(0))
    score = float(np.asarray(scores[0]).item())
    label = 'ABOVE_THRESHOLD' if score >= THRESHOLD else 'BELOW_THRESHOLD'

    draw = PIL.ImageDraw.Draw(frame)
    color = 'red' if score >= THRESHOLD else 'lime'
    draw.rectangle(box, outline=color, width=3)
    draw.text((box[0], max(0, box[1] - 16)), f'{label} {score:.4f}', fill=color)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    frame.save(args.output)
    print(f't={args.time:.1f}s center=({x},{y}) score={score:.4f} '
          f'threshold={THRESHOLD:.4f} {label} output={args.output}')


if __name__ == '__main__':
    main()
