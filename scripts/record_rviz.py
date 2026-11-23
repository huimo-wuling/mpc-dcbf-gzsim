#!/usr/bin/env python3
"""Record an RViz2 (or any X11/XWayland) window to an MP4.

RViz has no recording plugin, so this script grabs the window with `XGetImage`
(works under a Wayland session through XWayland) at whatever rate the X server
can hand out pixels (~7 fps for a 3100x2074 window on this machine), then
encodes the frames with OpenCV.

Usage:
    bash scripts/start.sh &                       # Gazebo + RViz, keep the screen unlocked
    python3 scripts/record_rviz.py --duration 25 --output results/rviz_video.mp4

    python3 scripts/record_rviz.py --window "Gazebo Sim" --width 1000 ...

Notes:
* the window must be mapped and actually repainting; a locked screen can make
  Qt throttle painting, in which case frames repeat (the script warns if the
  captured frames are nearly identical);
* `--keep-frames` keeps the intermediate JPEGs.
"""

import argparse
import os
import shutil
import sys
import tempfile
import time

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from capture_window import grab  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=25.0)
    ap.add_argument('--output', default='results/rviz_video.mp4')
    ap.add_argument('--window', default='RViz',
                    help='substring of the window title (default: RViz)')
    ap.add_argument('--width', type=int, default=1280,
                    help='resize frames to this width (0 = keep native size)')
    ap.add_argument('--fps', type=float, default=10.0)
    ap.add_argument('--keep-frames', action='store_true')
    args = ap.parse_args()

    import cv2

    probe = grab(args.window)
    if probe is None:
        raise SystemExit('window %r not found - is RViz running (scripts/start.sh)?'
                         % args.window)

    tmp = tempfile.mkdtemp(prefix='rviz_frames_')
    os.makedirs(os.path.dirname(os.path.abspath(args.output)) or '.', exist_ok=True)

    print('[rviz] recording %.1f s from window %r ...' % (args.duration, args.window))
    t0 = time.time()
    n = 0
    skipped = 0
    last_small = None
    diffs = []
    while time.time() - t0 < args.duration:
        im = grab(args.window)
        if im is None:
            time.sleep(0.05)
            continue
        if args.width and im.width > args.width:
            im = im.resize((args.width, int(im.height * args.width / im.width)),
                           Image.LANCZOS)
        frame = np.asarray(im)
        if frame.mean() < 3.0:          # black read-back: window not repainting
            skipped += 1
            time.sleep(0.05)
            continue
        small = np.asarray(im.resize((160, int(im.height * 160 / im.width))))
        if last_small is not None:
            diffs.append(float(np.abs(small.astype(int) - last_small.astype(int)).mean()))
        last_small = small
        cv2.imwrite(os.path.join(tmp, '%05d.jpg' % n), frame[:, :, ::-1])
        n += 1

    files = sorted(os.listdir(tmp))
    if not files:
        raise SystemExit('no frames captured')
    first = cv2.imread(os.path.join(tmp, files[0]))
    h, w = first.shape[:2]
    writer = cv2.VideoWriter(args.output, cv2.VideoWriter_fourcc(*'mp4v'),
                             args.fps, (w, h))
    for f in files:
        writer.write(cv2.imread(os.path.join(tmp, f)))
    writer.release()

    fps_captured = n / args.duration
    motion = float(np.mean(diffs)) if diffs else 0.0
    print('[rviz] captured %d frames in %.1f s (%.1f fps), motion score %.2f, '
          'skipped %d black reads' % (n, args.duration, fps_captured, motion, skipped))
    if motion < 0.3:
        print('[rviz] WARNING: frames are almost identical - the scene may be static '
              '(simulation finished) or the screen is locked and painting is throttled')
    print('[rviz] wrote %s (%dx%d, %.1f fps, %.1f s)'
          % (args.output, w, h, args.fps, n / args.fps))
    if not args.keep_frames:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == '__main__':
    main()
