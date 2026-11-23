"""Grey out everything in a colour frame that lies beyond the room being looked at (no ROS).

The toy sets stand 0.8-1.2 m tall, taller than the 0.53 m walls (maze_info.md), so a frame
looking into one room also shows the toy sets of neighbouring rooms over the walls, the
other arena ~1.5 m away and the hall (tables, people, signs). A vision model asked "what
room is this?" can answer for any of them. Before a frame goes to the model, every pixel
whose depth is beyond the room cell (the farthest corner of the room seen from the robot +
a margin), or that has no depth near it, is filled grey.

  limit = room_depth_limit(robot_xy, cell_centre, cell_size)
  mask = depth_mask(depth_m, depth_k, colour_k, colour_size, limit)   # True = keep
  out = apply_mask(bgr, mask)

The depth image may be unaligned to the colour image (the Lite3's Orbbec: depth fx 373 vs
colour fx 462, sensors ~14 mm apart; team_perception qr_utils.DepthAlign): depth pixels are
deprojected and projected into the colour camera (parallel axes, `t` = colour camera in the
depth optical frame). The kept region is closed and grown a little, so small depth holes on
dark or shiny toy parts stay visible. Pure numpy + OpenCV (apt cv2 4.5), so it ports into any
room_scene version (the VLM-wired one decodes frames in RoomScene.process(), before stitch()).
"""
import math

import numpy as np

try:
    import cv2
except ImportError:            # tests without OpenCV: no morphology, mask still works
    cv2 = None


def room_depth_limit(robot_xy, cell_centre, cell_size=1.02, margin=0.25):
    """m: farthest corner of the room cell from the robot (horizontal) + margin. The camera
    depth of any point in the room is at most its distance from the camera."""
    rx, ry = float(robot_xy[0]), float(robot_xy[1])
    cx, cy = float(cell_centre[0]), float(cell_centre[1])
    h = cell_size / 2 * math.sqrt(2)                   # any grid yaw: the corner circle
    return math.hypot(cx - rx, cy - ry) + h + margin


def depth_mask(depth_m, depth_k, colour_k, colour_size, max_depth, min_depth=0.15,
               t=(0.0, 0.0, 0.0), step=2, close_px=21, grow_px=9):
    """(h, w) bool mask in the colour image (colour_size = (w, h)): True where the scene is
    within max_depth m. depth_m: (H, W) float metres (0 / NaN = no return); depth_k,
    colour_k: (fx, fy, cx, cy)."""
    w, h = int(colour_size[0]), int(colour_size[1])
    d = np.asarray(depth_m, np.float32)[::step, ::step]
    vs, us = np.mgrid[0:depth_m.shape[0]:step, 0:depth_m.shape[1]:step]
    ok = np.isfinite(d) & (d > min_depth) & (d <= max_depth)
    mask = np.zeros((h, w), bool)
    if not ok.any():
        return mask
    z = d[ok].astype(np.float64)
    dfx, dfy, dcx, dcy = (float(v) for v in depth_k)
    x = (us[ok] - dcx) / dfx * z - t[0]
    y = (vs[ok] - dcy) / dfy * z - t[1]
    z = np.maximum(z - t[2], 1e-3)
    fx, fy, cx, cy = (float(v) for v in colour_k)
    u = np.round(cx + fx * x / z).astype(int)
    v = np.round(cy + fy * y / z).astype(int)
    inside = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    mask[v[inside], u[inside]] = True
    if cv2 is not None:
        m = mask.astype(np.uint8)
        k = max(3, int(round(step * fx / dfx)) * 2 + 1)        # fill the sampling gaps
        m = cv2.dilate(m, np.ones((k, k), np.uint8))
        if close_px > 1:
            m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((close_px, close_px), np.uint8))
        if grow_px > 1:
            m = cv2.dilate(m, np.ones((grow_px, grow_px), np.uint8))
        mask = m.astype(bool)
    return mask


def apply_mask(bgr, mask, fill=(128, 128, 128)):
    """Copy of bgr with the pixels outside mask filled; mask resized to bgr if it differs."""
    img = np.array(bgr, copy=True)
    if mask.shape[:2] != img.shape[:2]:
        if cv2 is None:
            raise ValueError('mask and image sizes differ and OpenCV is missing')
        mask = cv2.resize(mask.astype(np.uint8), (img.shape[1], img.shape[0]),
                          interpolation=cv2.INTER_NEAREST).astype(bool)
    img[~mask] = fill
    return img


def depth_to_metres(msg_encoding, array):
    """sensor_msgs/Image depth as metres: 16UC1 / mono16 are mm, 32FC1 metres."""
    a = np.asarray(array)
    if msg_encoding in ('16UC1', 'mono16'):
        return a.astype(np.float32) / 1000.0
    return a.astype(np.float32)
