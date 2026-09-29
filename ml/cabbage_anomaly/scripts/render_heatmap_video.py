"""Overlay PatchCore heatmaps on cabbage crops in a /rgb MP4."""

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import PIL.Image
import PIL.ImageDraw
import torch
from matplotlib import colormaps
from torchvision import transforms


MODEL_DIR = Path(__file__).resolve().parents[1] / 'models' / 'patchcore'
SLOTS = [(143, 235), (271, 235), (405, 235),
         (118, 332), (271, 332), (426, 332)]
THRESHOLD = 0.32702351
SIZE = 640


def read_frame(stream):
    needed = SIZE * SIZE * 3
    chunks = []
    while needed:
        chunk = stream.read(needed)
        if not chunk:
            if chunks:
                raise RuntimeError('Incomplete video frame from ffmpeg')
            return None
        chunks.append(chunk)
        needed -= len(chunk)
    return np.frombuffer(b''.join(chunks), dtype=np.uint8).reshape(SIZE, SIZE, 3)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('video', type=Path)
    parser.add_argument('output', type=Path)
    roi = parser.add_mutually_exclusive_group(required=True)
    roi.add_argument('--center', type=int, nargs=2, metavar=('X', 'Y'),
                     help='one cabbage center, held fixed for this segment')
    roi.add_argument('--fixed-six-slots', action='store_true',
                     help='six trained crop positions for a stationary inspection view')
    parser.add_argument('--start', type=float, default=0.0, help='start second')
    parser.add_argument('--duration', type=float, help='seconds to process')
    parser.add_argument('--fps', type=float, default=2.0, help='sampled frames per second')
    parser.add_argument('--vmax', type=float, default=0.6, help='fixed heatmap color scale')
    args = parser.parse_args()
    if not args.video.is_file() or args.start < 0 or args.fps <= 0 or args.vmax <= 0:
        parser.error('check video path, --start, --fps, and --vmax')
    if args.duration is not None and args.duration <= 0:
        parser.error('--duration must be positive')
    if args.fixed_six_slots:
        regions = SLOTS
    else:
        x, y = args.center
        regions = [(x - 48, y - 48)]
    if any(x < 0 or y < 0 or x + 96 > SIZE or y + 96 > SIZE
           for x, y in regions):
        parser.error('96x96 crop extends outside the 640x640 frame')
    if args.output.exists():
        parser.error(f'output already exists: {args.output}')

    metadata = json.loads((MODEL_DIR / 'training_metadata.json').read_text())
    sys.path.insert(0, str(Path(metadata['patchcore_repo']) / 'src'))
    import patchcore.common
    import patchcore.patchcore

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = patchcore.patchcore.PatchCore(device)
    model.load_from_path(str(MODEL_DIR), device,
                         nn_method=patchcore.common.FaissNN(False, 4))
    transform = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406],
                             std=[0.229, 0.224, 0.225]),
    ])

    decode = ['ffmpeg', '-v', 'error', '-ss', str(args.start), '-i', str(args.video)]
    if args.duration is not None:
        decode += ['-t', str(args.duration)]
    decode += ['-vf', f'fps={args.fps}', '-f', 'rawvideo',
               '-pix_fmt', 'rgb24', 'pipe:1']
    args.output.parent.mkdir(parents=True, exist_ok=True)
    encode = ['ffmpeg', '-v', 'error', '-n', '-f', 'rawvideo',
              '-pix_fmt', 'rgb24', '-s', f'{SIZE}x{SIZE}',
              '-r', str(args.fps), '-i', 'pipe:0', '-an', '-c:v', 'libx264',
              '-pix_fmt', 'yuv420p', str(args.output)]
    cmap = colormaps['inferno']
    count = 0
    with subprocess.Popen(decode, stdout=subprocess.PIPE) as reader, \
            subprocess.Popen(encode, stdin=subprocess.PIPE) as writer:
        while (frame := read_frame(reader.stdout)) is not None:
            image = PIL.Image.fromarray(frame)
            crops = [image.crop((x, y, x + 96, y + 96)) for x, y in regions]
            scores, masks = model.predict(torch.stack([transform(crop) for crop in crops]))
            canvas = frame.copy()
            for (x, y), mask in zip(regions, masks):
                heat = np.clip(np.asarray(mask) / args.vmax, 0, 1)
                color = (cmap(heat)[..., :3] * 255).astype(np.uint8)
                region = canvas[y:y + 96, x:x + 96].astype(np.float32)
                canvas[y:y + 96, x:x + 96] = (
                    region * 0.55 + color * 0.45).astype(np.uint8)
            result = PIL.Image.fromarray(canvas)
            draw = PIL.ImageDraw.Draw(result)
            for index, ((x, y), score) in enumerate(zip(regions, scores), start=1):
                value = float(np.asarray(score).item())
                outline = 'red' if value >= THRESHOLD else 'lime'
                draw.rectangle((x, y, x + 95, y + 95), outline=outline, width=2)
                label = f'{index:02d} {value:.3f}' if args.fixed_six_slots else f'score {value:.3f}'
                draw.text((x, y - 14), label, fill=outline)
            writer.stdin.write(np.asarray(result).tobytes())
            count += 1
            if count % 20 == 0:
                print(f'Processed {count} frames', flush=True)
        writer.stdin.close()
        if reader.wait() != 0 or writer.wait() != 0:
            raise RuntimeError('ffmpeg failed')
    if count == 0:
        raise RuntimeError('No video frames decoded')
    print(f'Saved {count} heatmap frames to {args.output}')


if __name__ == '__main__':
    main()
