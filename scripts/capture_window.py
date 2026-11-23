#!/usr/bin/env python3
"""Grab an X11 / XWayland window by title and save it as PNG.

Under a Wayland session the compositor does not expose the screen to X11
clients, but XWayland *does* keep each window's pixels, so `XGetImage` on the
window itself still works (this is how the RViz / Gazebo Sim figures in
`results/` were produced, even with the session locked).

Usage:
    python3 scripts/capture_window.py out.png "RViz"
    python3 scripts/capture_window.py out.png "Gazebo Sim"

Notes:
* only works on a Linux X11/XWayland session (not native Wayland windows);
* if several windows match, the largest mapped one is used.
"""

import ctypes
import ctypes.util
import sys

import numpy as np
from PIL import Image

X11 = ctypes.CDLL(ctypes.util.find_library('X11'))
X11.XOpenDisplay.restype = ctypes.c_void_p
X11.XOpenDisplay.argtypes = [ctypes.c_char_p]
X11.XDefaultRootWindow.restype = ctypes.c_ulong
X11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
X11.XQueryTree.argtypes = [ctypes.c_void_p, ctypes.c_ulong,
                           ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_ulong),
                           ctypes.POINTER(ctypes.POINTER(ctypes.c_ulong)),
                           ctypes.POINTER(ctypes.c_uint)]
X11.XFetchName.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.POINTER(ctypes.c_char_p)]


class Attrs(ctypes.Structure):
    _fields_ = [('x', ctypes.c_int), ('y', ctypes.c_int), ('width', ctypes.c_int),
                ('height', ctypes.c_int), ('border_width', ctypes.c_int),
                ('depth', ctypes.c_int), ('visual', ctypes.c_void_p), ('root', ctypes.c_ulong),
                ('class_', ctypes.c_int), ('bit_gravity', ctypes.c_int),
                ('win_gravity', ctypes.c_int), ('backing_store', ctypes.c_int),
                ('backing_planes', ctypes.c_ulong), ('backing_pixel', ctypes.c_ulong),
                ('save_under', ctypes.c_int), ('colormap', ctypes.c_ulong),
                ('map_installed', ctypes.c_int), ('map_state', ctypes.c_int),
                ('all_event_masks', ctypes.c_long), ('your_event_mask', ctypes.c_long),
                ('do_not_propagate_mask', ctypes.c_long), ('override_redirect', ctypes.c_int),
                ('screen', ctypes.c_void_p)]


class XImg(ctypes.Structure):
    _fields_ = [('width', ctypes.c_int), ('height', ctypes.c_int), ('xoffset', ctypes.c_int),
                ('format', ctypes.c_int), ('data', ctypes.c_void_p), ('byte_order', ctypes.c_int),
                ('bitmap_unit', ctypes.c_int), ('bitmap_bit_order', ctypes.c_int),
                ('bitmap_pad', ctypes.c_int), ('depth', ctypes.c_int),
                ('bytes_per_line', ctypes.c_int), ('bits_per_pixel', ctypes.c_int)]


ERRORS = []
HANDLER = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_void_p)


@HANDLER
def _on_error(_dpy, _ev):
    ERRORS.append(1)
    return 0


X11.XSetErrorHandler.argtypes = [HANDLER]
X11.XSetErrorHandler(_on_error)
X11.XGetWindowAttributes.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.POINTER(Attrs)]
X11.XGetImage.restype = ctypes.c_void_p
X11.XGetImage.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_int,
                          ctypes.c_uint, ctypes.c_uint, ctypes.c_ulong, ctypes.c_int]
X11.XSync.argtypes = [ctypes.c_void_p, ctypes.c_int]


def walk(dpy, win, depth=0, out=None):
    out = [] if out is None else out
    r, p = ctypes.c_ulong(), ctypes.c_ulong()
    kids = ctypes.POINTER(ctypes.c_ulong)()
    n = ctypes.c_uint()
    if not X11.XQueryTree(dpy, win, ctypes.byref(r), ctypes.byref(p),
                          ctypes.byref(kids), ctypes.byref(n)):
        return out
    for i in range(n.value):
        c = kids[i]
        nm = ctypes.c_char_p()
        title = ''
        if X11.XFetchName(dpy, c, ctypes.byref(nm)) and nm.value:
            title = nm.value.decode('utf-8', 'ignore')
        out.append((c, title))
        if depth < 4:
            walk(dpy, c, depth + 1, out)
    return out


def grab(want, min_size=200):
    """Return a PIL.Image of the largest mapped window whose title contains `want`."""
    dpy = X11.XOpenDisplay(None)
    if not dpy:
        return None
    root = X11.XDefaultRootWindow(dpy)
    cands = []
    for win, title in walk(dpy, root):
        if want.lower() not in title.lower():
            continue
        a = Attrs()
        if not X11.XGetWindowAttributes(dpy, win, ctypes.byref(a)):
            continue
        if a.map_state == 2 and a.width > min_size and a.height > min_size:
            cands.append((a.width * a.height, win, a.width, a.height))
    cands.sort(reverse=True)
    for _, win, w, h in cands:
        ERRORS.clear()
        img = X11.XGetImage(dpy, win, 0, 0, w, h, ctypes.c_ulong(-1).value, 2)
        X11.XSync(dpy, 0)
        if not img or ERRORS:
            continue
        im = ctypes.cast(img, ctypes.POINTER(XImg)).contents
        buf = ctypes.string_at(im.data, im.bytes_per_line * im.height)
        arr = np.frombuffer(buf, np.uint8).reshape(im.height, im.bytes_per_line)
        arr = arr[:, :im.width * 4].reshape(im.height, im.width, 4)
        if arr[:, :, :3].mean() < 1.0:
            continue
        return Image.fromarray(arr[:, :, [2, 1, 0]])
    return None


def main():
    if len(sys.argv) < 3:
        raise SystemExit(__doc__)
    out_path, want = sys.argv[1], sys.argv[2]
    dpy = X11.XOpenDisplay(None)
    if not dpy:
        raise SystemExit('cannot open X display (not an X11/XWayland session?)')
    root = X11.XDefaultRootWindow(dpy)

    cands = []
    for win, title in walk(dpy, root):
        if want.lower() not in title.lower():
            continue
        a = Attrs()
        if not X11.XGetWindowAttributes(dpy, win, ctypes.byref(a)):
            continue
        if a.map_state == 2 and a.width > 200 and a.height > 200:
            cands.append((a.width * a.height, win, title, a.width, a.height))
    if not cands:
        raise SystemExit('no mapped window matching %r' % want)
    cands.sort(reverse=True)

    for _, win, title, w, h in cands:
        ERRORS.clear()
        img = X11.XGetImage(dpy, win, 0, 0, w, h, ctypes.c_ulong(-1).value, 2)
        X11.XSync(dpy, 0)
        if not img or ERRORS:
            continue
        im = ctypes.cast(img, ctypes.POINTER(XImg)).contents
        buf = ctypes.string_at(im.data, im.bytes_per_line * im.height)
        arr = np.frombuffer(buf, np.uint8).reshape(im.height, im.bytes_per_line)
        arr = arr[:, :im.width * 4].reshape(im.height, im.width, 4)
        if arr[:, :, :3].mean() < 1.0:      # black / not rendered
            continue
        Image.fromarray(arr[:, :, [2, 1, 0]]).save(out_path)
        print('saved %s  (%s, %dx%d)' % (out_path, title.strip(), im.width, im.height))
        return
    raise SystemExit('could not capture any matching window')


if __name__ == '__main__':
    main()
