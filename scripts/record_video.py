#!/usr/bin/env python3
"""Record a video of the Gazebo Sim GUI (for papers / defence slides).

How it works
------------
Gazebo Sim ships a `Screenshot` GUI plugin that exposes `/gui/screenshot`
(`gz.msgs.StringMsg` -> `gz.msgs.Boolean`). Each call writes a timestamped PNG
of the 3D view into the directory you pass. This script drives that service and
re-encodes the frames into an MP4 with OpenCV:

  * camera: either a fixed pose, or a "chase" camera that follows the robot
    (`/world/<world>/dynamic_pose/info`, so no ROS python is needed);
  * frames: as fast as the GUI can render them (~10 fps on this machine);
  * encoding: `cv2.VideoWriter` with the `mp4v` codec.

Prerequisites
-------------
* the simulation must be running **with the Gazebo GUI** (i.e. do NOT pass
  `headless:=true`) and the same `GZ_PARTITION` as this script;
* the GUI window has to be visible/rendering (a minimised window still
  produces frames on X11 but Wayland may throttle it);
* `python3-opencv` (or `pip install opencv-python`).

Usage
-----
    bash scripts/start.sh rviz:=false &            # GUI, no RViz
    python3 scripts/record_video.py --duration 25 --output results/reproduction.mp4

    # fixed camera instead of chase
    python3 scripts/record_video.py --camera fixed --eye -15 -11 7.5 --target 0 0 0.3
"""

import argparse
import glob
import os
import shutil
import tempfile
import time

import gz.transport as gz_transport
from gz.msgs.boolean_pb2 import Boolean
from gz.msgs.gui_camera_pb2 import GUICamera
from gz.msgs.pose_v_pb2 import Pose_V
from gz.msgs.stringmsg_pb2 import StringMsg


def look_at(eye, target):
    """Quaternion (x, y, z, w) for a gz camera (looks along +X, up +Z)."""
    import math

    fx, fy, fz = target[0] - eye[0], target[1] - eye[1], target[2] - eye[2]
    n = math.sqrt(fx * fx + fy * fy + fz * fz) or 1.0
    fx, fy, fz = fx / n, fy / n, fz / n
    yaw = math.atan2(fy, fx)
    pitch = -math.asin(max(-1.0, min(1.0, fz)))
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    return (-sy * sp, cy * sp, sy * cp, cy * cp)


class GzVideo:
    def __init__(self, world, robot):
        self.node = gz_transport.Node()
        self.world = world
        self.robot = robot
        self.robot_xy = None
        self.node.subscribe(Pose_V, '/world/%s/dynamic_pose/info' % world,
                            self._on_pose)

    def _on_pose(self, msg):
        for pose in msg.pose:
            if pose.name == self.robot:
                self.robot_xy = (pose.position.x, pose.position.y, pose.position.z)

    def set_camera(self, eye, target, duration=0.0):
        qx, qy, qz, qw = look_at(eye, target)
        req = GUICamera()
        req.name = 'Orbit'
        req.view_controller = 'Orbit'

        req.pose.position.x, req.pose.position.y, req.pose.position.z = eye
        req.pose.orientation.x = qx
        req.pose.orientation.y = qy
        req.pose.orientation.z = qz
        req.pose.orientation.w = qw
        req.duration = float(duration)
        try:
            self.node.request('/gui/move_to/pose', req, GUICamera, Boolean, 1000)
        except Exception:  # pragma: no cover - service may not be up yet
            pass

    def close(self):
        try:
            self.node.unsubscribe('/world/%s/dynamic_pose/info' % self.world)
        except Exception:  # pragma: no cover
            pass
        self.node = None

    def screenshot(self, directory):
        req = StringMsg()
        req.data = directory
        try:
            self.node.request('/gui/screenshot', req, StringMsg, Boolean, 1000)
        except Exception:  # pragma: no cover
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=25.0)
    ap.add_argument('--output', default='results/reproduction.mp4')
    ap.add_argument('--world', default='mpc_dcbf')
    ap.add_argument('--robot', default='jackal')
    ap.add_argument('--camera', choices=['chase', 'fixed'], default='chase')
    ap.add_argument('--eye', type=float, nargs=3, default=[-7.0, -8.0, 4.5])
    ap.add_argument('--target', type=float, nargs=3, default=[0.0, 0.0, 0.3])
    ap.add_argument('--chase-eye', type=float, nargs=3, default=[-2.2, -2.6, 1.9],
                    help='camera offset from the robot for --camera chase')
    ap.add_argument('--fps', type=float, default=10.0,
                    help='encoder frame rate (capture runs as fast as the GUI)')
    ap.add_argument('--keep-frames', action='store_true')
    args = ap.parse_args()

    import cv2  # imported late so --help works without OpenCV

    gz = GzVideo(args.world, args.robot)
    frames_dir = tempfile.mkdtemp(prefix='gz_frames_')
    os.makedirs(os.path.dirname(os.path.abspath(args.output)) or '.', exist_ok=True)

    if args.camera == 'fixed':
        gz.set_camera(tuple(args.eye), tuple(args.target))

    print('[video] recording %.1f s (camera: %s) ...' % (args.duration, args.camera))
    t0 = time.time()
    next_cam = 0.0
    frames = []
    while time.time() - t0 < args.duration:
        now = time.time() - t0
        if args.camera == 'chase' and gz.robot_xy and now >= next_cam:
            rx, ry, rz = gz.robot_xy
            eye = (rx + args.chase_eye[0], ry + args.chase_eye[1],
                   rz + args.chase_eye[2])
            gz.set_camera(eye, (rx, ry, 0.3), duration=0.35)
            next_cam = now + 0.35
        before = set(glob.glob(os.path.join(frames_dir, '*.png')))
        gz.screenshot(frames_dir)
        time.sleep(0.08)
        after = set(glob.glob(os.path.join(frames_dir, '*.png')))
        frames.extend(sorted(after - before))

    frames = sorted(glob.glob(os.path.join(frames_dir, '*.png')))
    if not frames:
        raise SystemExit('no frames captured - is the Gazebo GUI running with the '
                         'same GZ_PARTITION?')
    print('[video] captured %d frames in %.1f s (%.1f fps)'
          % (len(frames), args.duration, len(frames) / args.duration))

    first = cv2.imread(frames[0])
    h, w = first.shape[:2]
    writer = cv2.VideoWriter(args.output, cv2.VideoWriter_fourcc(*'mp4v'),
                             args.fps, (w, h))
    for f in frames:
        img = cv2.imread(f)
        if img.shape[:2] != (h, w):
            img = cv2.resize(img, (w, h))
        writer.write(img)
    writer.release()
    print('[video] wrote %s (%dx%d, %.1f fps, %.1f s)'
          % (args.output, w, h, args.fps, len(frames) / args.fps))

    gz.close()

    if not args.keep_frames:
        shutil.rmtree(frames_dir, ignore_errors=True)
    else:
        print('[video] frames kept in', frames_dir)


if __name__ == '__main__':
    main()
