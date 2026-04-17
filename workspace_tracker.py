#!/usr/bin/env python3
"""
Workspace Tracker for Differential Drive Robot

- 4 AprilTags at workspace corners define the 2D workspace plane
- 1 AprilTag on the robot gives pose (x, y, theta) in workspace frame
- Single-color object detected and projected onto the workspace plane

Camera: angled overhead, calibrated in MATLAB.
"""

import sys
import cv2
import numpy as np
from pupil_apriltags import Detector

# ── Camera intrinsics (MATLAB calibration) ────────────────────────────────────
K = np.array([
    [1423.3,    0.0,  964.7583],
    [   0.0, 1416.4,  577.4247],
    [   0.0,    0.0,    1.0   ]
], dtype=np.float64)

DIST = np.array([0.0236, -0.0907, 0.0, 0.0], dtype=np.float64)

# ── AprilTag IDs ───────────────────────────────────────────────────────────────
# Corner order as seen from above: top-left, top-right, bottom-right, bottom-left
CORNER_TAG_IDS = [0, 1, 2, 3]   # ← change to your actual corner tag IDs
ROBOT_TAG_ID   = 4               # ← change to your robot tag ID

# Physical size of the robot's AprilTag (side length, mm) and its height above
# the workspace plane (center of the tag face, mm).
ROBOT_TAG_SIZE_MM   = 44.0    # ← set to your robot tag's printed size
ROBOT_TAG_HEIGHT_MM = 115.0   # ← measure from ground to center of robot tag

# 3D corners of the robot tag in the tag's local frame.
# pupil-apriltags corner order: [BL, BR, TR, TL] — we match that here.
_half = ROBOT_TAG_SIZE_MM / 2.0
ROBOT_TAG_CORNERS_3D = np.array([
    [-_half, -_half, 0.0],   # BL
    [ _half, -_half, 0.0],   # BR
    [ _half,  _half, 0.0],   # TR
    [-_half,  _half, 0.0],   # TL
], dtype=np.float64)

# ── Workspace definition ───────────────────────────────────────────────────────
WORKSPACE_W_MM = 790.0   # physical width  (x-axis, mm)
WORKSPACE_H_MM = 730.0   # physical height (y-axis, mm)

# World coordinates of corner tags (matches CORNER_TAG_IDS order above)
WORKSPACE_CORNERS_WORLD = np.array([
    [0.0,            WORKSPACE_H_MM],   # top-left
    [WORKSPACE_W_MM, WORKSPACE_H_MM],   # top-right
    [WORKSPACE_W_MM, 0.0           ],   # bottom-right
    [0.0,            0.0           ],   # bottom-left
], dtype=np.float64)

# ── Color detection (HSV) ──────────────────────────────────────────────────────
# Default: red object (wraps around H=180 in OpenCV HSV)
# Tune these or swap for your object's color:
#   green  → lower=(40,50,50)  upper=(80,255,255)  no second range
#   blue   → lower=(100,50,50) upper=(130,255,255) no second range
#   yellow → lower=(20,100,100) upper=(35,255,255) no second range
COLOR_LOWER  = np.array([  0, 120,  70], dtype=np.uint8)
COLOR_UPPER  = np.array([ 10, 255, 255], dtype=np.uint8)
COLOR_LOWER2 = np.array([170, 120,  70], dtype=np.uint8)  # red wraps at 180
COLOR_UPPER2 = np.array([180, 255, 255], dtype=np.uint8)
MIN_BLOB_AREA = 500  # px²; blobs smaller than this are ignored

# ── Goal pose ──────────────────────────────────────────────────────────────────
GOAL_DISTANCE_MM = 130.0  # ← standoff distance from object centre to goal (mm)


class WorkspaceTracker:
    def __init__(self):
        self.detector = Detector(
            families='tag36h11',
            nthreads=2,
            quad_decimate=1.5,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
        )
        self.H     = None   # homography: image pixels → workspace mm (for ground-plane objects)
        self.H_inv = None   # inverse: workspace mm → image pixels (for drawing)
        self.R_cam = None   # rotation    world → camera  (3×3, from solvePnP on corner tags)
        self.t_cam = None   # translation world → camera  (3,   from solvePnP on corner tags)

    # ── Public API ────────────────────────────────────────────────────────────

    def process_frame(self, frame):
        """
        Process one BGR frame.

        Returns
        -------
        robot_pose : (x_mm, y_mm, theta_rad) in workspace frame, or None
        object_pos : (x_mm, y_mm, theta_rad) — centroid + short-axis angle, or None
        goal_pose  : (x_mm, y_mm, theta_rad) — standoff goal for the robot, or None
        vis        : annotated BGR image
        """
        undist = cv2.undistort(frame, K, DIST)
        gray   = cv2.cvtColor(undist, cv2.COLOR_BGR2GRAY)

        detections = self.detector.detect(gray)
        self._update_homography(detections)
        self._update_extrinsics(detections)

        robot_pose = self._get_robot_pose(detections)
        object_pos = self._detect_color_object(undist)
        goal_pose  = self._compute_goal_pose(object_pos, robot_pose)
        vis        = self._draw(undist, detections, robot_pose, object_pos, goal_pose)

        return robot_pose, object_pos, goal_pose, vis

    def _compute_goal_pose(self, object_pos, robot_pose):
        """
        Place the goal at GOAL_DISTANCE_MM from the object along its short axis.
        Of the two candidates (+/- short axis), pick the one closer to the robot.
        The goal heading faces toward the object centre.
        Returns (x_mm, y_mm, theta_rad) or None.
        """
        if object_pos is None:
            return None

        ox, oy, obj_theta = object_pos
        short_dir = np.array([np.cos(obj_theta), np.sin(obj_theta)])
        obj_xy    = np.array([ox, oy])

        cand_a = obj_xy + GOAL_DISTANCE_MM * short_dir
        cand_b = obj_xy - GOAL_DISTANCE_MM * short_dir

        if robot_pose is not None:
            rxy = np.array([robot_pose[0], robot_pose[1]])
            gx, gy = cand_a if np.linalg.norm(cand_a - rxy) <= np.linalg.norm(cand_b - rxy) else cand_b
        else:
            gx, gy = cand_a

        goal_heading = np.arctan2(oy - gy, ox - gx)  # robot faces the object
        return (float(gx), float(gy), float(goal_heading))

    # ── Internal methods ──────────────────────────────────────────────────────

    def _update_homography(self, detections):
        """Recompute H whenever all 4 corner tags are visible."""
        tag_map = {d.tag_id: d for d in detections}
        if not all(tid in tag_map for tid in CORNER_TAG_IDS):
            return  # keep last good homography

        img_pts = np.array(
            [tag_map[tid].center for tid in CORNER_TAG_IDS],
            dtype=np.float64
        )
        H, _ = cv2.findHomography(img_pts, WORKSPACE_CORNERS_WORLD)
        if H is not None:
            self.H     = H
            self.H_inv = np.linalg.inv(H)

    def _update_extrinsics(self, detections):
        """
        Compute camera pose (R, t) from the 4 corner tags via solvePnP.
        Uses the undistorted image, so distortion is passed as None.
        world_pts are the tag centres at z=0 on the workspace plane.
        """
        tag_map = {d.tag_id: d for d in detections}
        if not all(tid in tag_map for tid in CORNER_TAG_IDS):
            return  # keep last good extrinsics

        img_pts = np.array(
            [tag_map[tid].center for tid in CORNER_TAG_IDS],
            dtype=np.float64
        ).reshape(-1, 1, 2)

        world_pts = np.hstack(
            [WORKSPACE_CORNERS_WORLD, np.zeros((4, 1))]
        ).reshape(-1, 1, 3).astype(np.float64)

        # Image is already undistorted, so pass None for distortion coefficients
        ok, rvec, tvec = cv2.solvePnP(
            world_pts, img_pts, K, None,
            flags=cv2.SOLVEPNP_ITERATIVE
        )
        if ok:
            R, _ = cv2.Rodrigues(rvec)
            self.R_cam = R
            self.t_cam = tvec.flatten()

    def _project_to_workspace(self, px, py):
        """Map one image pixel (px, py) → workspace (x_mm, y_mm), or None."""
        if self.H is None:
            return None
        pt  = np.array([[[float(px), float(py)]]], dtype=np.float64)
        res = cv2.perspectiveTransform(pt, self.H)
        return float(res[0, 0, 0]), float(res[0, 0, 1])

    def _project_to_image(self, x_mm, y_mm):
        """Map workspace (x_mm, y_mm) → image pixel, or (None, None)."""
        if self.H_inv is None:
            return None, None
        pt  = np.array([[[float(x_mm), float(y_mm)]]], dtype=np.float64)
        res = cv2.perspectiveTransform(pt, self.H_inv)
        return float(res[0, 0, 0]), float(res[0, 0, 1])

    def _get_robot_pose(self, detections):
        """
        Returns (x_mm, y_mm, theta_rad) from the robot AprilTag.

        Uses solvePnP on the robot tag's 4 corners to recover its 3D pose in
        the camera frame, then transforms to the workspace frame using the
        camera extrinsics computed from the corner tags.  This correctly
        accounts for the tag being at height ROBOT_TAG_HEIGHT_MM above the
        workspace plane — the homography alone would give a displaced position
        because the camera is angled.

        Heading: the tag's +Y axis projected onto the workspace XY plane.
        Define which physical side of the tag faces "forward" on the robot by
        rotating ROBOT_TAG_HEIGHT_MM if needed.
        """
        if self.R_cam is None:
            return None

        tag_map = {d.tag_id: d for d in detections}
        if ROBOT_TAG_ID not in tag_map:
            return None

        tag = tag_map[ROBOT_TAG_ID]

        # ── Position: ray-plane intersection at z = ROBOT_TAG_HEIGHT_MM ─────
        # More robust than solvePnP translation, which degrades at corners.
        cx, cy = tag.center
        ray_cam = np.linalg.solve(K, np.array([cx, cy, 1.0]))  # K⁻¹·[px,py,1]
        r2 = self.R_cam.T[2, :]  # 3rd row of R_cam^T  (= R_cam[:, 2])
        lam = (ROBOT_TAG_HEIGHT_MM + r2 @ self.t_cam) / (r2 @ ray_cam)
        P_world = self.R_cam.T @ (lam * ray_cam - self.t_cam)
        x_mm = P_world[0]
        y_mm = P_world[1]

        # ── Orientation: solvePnP on tag corners (rotation is reliable) ──────
        corners_2d = tag.corners.reshape(-1, 1, 2).astype(np.float64)
        ok, rvec_tag, _ = cv2.solvePnP(
            ROBOT_TAG_CORNERS_3D, corners_2d, K, None,
            flags=cv2.SOLVEPNP_ITERATIVE
        )
        if not ok:
            return None

        R_tag, _ = cv2.Rodrigues(rvec_tag)
        tag_y_world = self.R_cam.T @ R_tag[:, 1]          # tag +Y in world frame
        theta = np.arctan2(tag_y_world[1], tag_y_world[0])

        return (x_mm, y_mm, theta)

    def _detect_color_object(self, frame_bgr):
        """
        HSV colour threshold → largest blob → workspace projection.
        Returns (x_mm, y_mm, theta_rad) where theta is the short-axis direction
        of the blob's bounding rectangle in the workspace frame, or None.
        """
        if self.H is None:
            return None

        hsv  = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, COLOR_LOWER, COLOR_UPPER)
        mask |= cv2.inRange(hsv, COLOR_LOWER2, COLOR_UPPER2)

        kernel = np.ones((5, 5), np.uint8)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < MIN_BLOB_AREA:
            return None

        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None

        cx = M['m10'] / M['m00']
        cy = M['m01'] / M['m00']

        ws = self._project_to_workspace(cx, cy)
        if ws is None:
            return None
        x_mm, y_mm = ws

        # ── Short-axis direction ───────────────────────────────────────────────
        # minAreaRect angle is the angle of the width side w.r.t. the x-axis,
        # in [-90, 0).  Short axis = the side with the smaller dimension.
        (_, _), (w, h), angle_deg = cv2.minAreaRect(largest)
        short_angle_rad = np.radians(angle_deg if w < h else angle_deg + 90.0)
        short_img = np.array([np.cos(short_angle_rad), np.sin(short_angle_rad)])

        # Map the direction to workspace via homography (two-point method)
        scale = 30.0  # px offset large enough for a stable direction estimate
        p2  = np.array([[[cx + scale * short_img[0],
                           cy + scale * short_img[1]]]], dtype=np.float64)
        ws2 = cv2.perspectiveTransform(p2, self.H)[0, 0]
        short_world = ws2 - np.array([x_mm, y_mm])
        norm = np.linalg.norm(short_world)
        theta = np.arctan2(short_world[1], short_world[0]) if norm > 1e-6 else 0.0

        return (x_mm, y_mm, theta)

    def _draw(self, frame, detections, robot_pose, object_pos, goal_pose):
        vis = frame.copy()

        # Draw all detected AprilTags
        for d in detections:
            corners = d.corners.astype(int)
            color   = (0, 200, 0) if d.tag_id in CORNER_TAG_IDS else (255, 100, 0)
            cv2.polylines(vis, [corners], True, color, 2)
            cv2.putText(vis, f'ID{d.tag_id}', tuple(corners[3]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # Draw robot pose arrow
        if robot_pose is not None and self.R_cam is not None:
            x, y, th = robot_pose
            rvec_cam, _ = cv2.Rodrigues(self.R_cam)
            projected, _ = cv2.projectPoints(
                np.array([[[x, y, ROBOT_TAG_HEIGHT_MM]]], dtype=np.float64),
                rvec_cam, self.t_cam.reshape(3, 1), K, None
            )
            ix, iy = float(projected[0, 0, 0]), float(projected[0, 0, 1])
            ex = int(ix + 40 * np.cos(th))
            ey = int(iy - 40 * np.sin(th))  # y is flipped in image coords
            cv2.arrowedLine(vis, (int(ix), int(iy)), (ex, ey),
                            (0, 0, 255), 3, tipLength=0.3)
            cv2.putText(vis,
                        f'Robot ({x:.0f}, {y:.0f}) mm  {np.degrees(th):.1f}deg',
                        (int(ix) + 8, int(iy) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

        # Draw object: cross + short-axis line
        if object_pos is not None:
            ox, oy, obj_th = object_pos
            ix, iy = self._project_to_image(ox, oy)
            if ix is not None:
                # Short-axis tick
                ex = int(ix + 30 * np.cos(obj_th))
                ey = int(iy - 30 * np.sin(obj_th))
                sx = int(ix - 30 * np.cos(obj_th))
                sy = int(iy + 30 * np.sin(obj_th))
                cv2.line(vis, (sx, sy), (ex, ey), (0, 165, 255), 2)
                cv2.drawMarker(vis, (int(ix), int(iy)),
                               (0, 165, 255), cv2.MARKER_CROSS, 24, 3)
                cv2.putText(vis, f'Obj ({ox:.0f}, {oy:.0f}) mm',
                            (int(ix) + 8, int(iy) - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 2)

        # Draw goal pose arrow
        if goal_pose is not None:
            gx, gy, gth = goal_pose
            ix, iy = self._project_to_image(gx, gy)
            if ix is not None:
                ex = int(ix + 40 * np.cos(gth))
                ey = int(iy - 40 * np.sin(gth))
                cv2.arrowedLine(vis, (int(ix), int(iy)), (ex, ey),
                                (255, 0, 255), 3, tipLength=0.3)
                cv2.putText(vis,
                            f'Goal ({gx:.0f}, {gy:.0f}) mm  {np.degrees(gth):.1f}deg',
                            (int(ix) + 8, int(iy) + 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 255), 2)

        # Status banner when workspace is not locked
        if self.H is None:
            cv2.putText(vis, 'WAITING FOR CORNER TAGS', (30, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

        return vis


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == '__main__':
    # Usage:
    #   python3 workspace_tracker.py          → webcam (device 0)
    #   python3 workspace_tracker.py 1        → webcam device 1
    #   python3 workspace_tracker.py video.mp4
    source = 0
    if len(sys.argv) > 1:
        source = int(sys.argv[1]) if sys.argv[1].isdigit() else sys.argv[1]

    cap     = cv2.VideoCapture(source)
    tracker = WorkspaceTracker()

    print('Press q to quit.')
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        robot_pose, object_pos, goal_pose, vis = tracker.process_frame(frame)

        if robot_pose:
            x, y, th = robot_pose
            print(f'Robot  x={x:7.1f} mm  y={y:7.1f} mm  '
                  f'theta={np.degrees(th):6.1f} deg')
        if object_pos:
            ox, oy, oth = object_pos
            print(f'Object x={ox:7.1f} mm  y={oy:7.1f} mm  '
                  f'short_axis={np.degrees(oth):6.1f} deg')
        if goal_pose:
            gx, gy, gth = goal_pose
            print(f'Goal   x={gx:7.1f} mm  y={gy:7.1f} mm  '
                  f'theta={np.degrees(gth):6.1f} deg')

        cv2.imshow('Workspace Tracker', vis)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
