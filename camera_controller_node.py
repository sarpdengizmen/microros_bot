#!/usr/bin/env python3
"""
ROS 2 node — camera-based closed-loop robot controller.

What it does
------------
- Runs WorkspaceTracker in a background thread (camera → robot & goal pose).
- Publishes /robot_pose     (PoseStamped) continuously when robot tag is visible.
- Publishes /object_goal    (PoseStamped) continuously when object is detected.
- Publishes /object_stable  (Bool)        True when object pose is stable over
  STABILITY_WINDOW consecutive frames within tolerance.
- On /start_motion service call: drives the robot to the goal pose using the
  polar posture regulation law directly — no trajectory needed.
- /stop_motion service cancels execution immediately.

Topics published
----------------
  /robot_pose     geometry_msgs/PoseStamped   camera-measured robot pose
  /object_goal    geometry_msgs/PoseStamped   camera-derived goal pose
  /cmd_vel        geometry_msgs/Twist         velocity commands (20 Hz when active)
  /motion_active  std_msgs/Bool               True while executing
  /object_stable  std_msgs/Bool               True when object pose is stable

Services
--------
  /start_motion  std_srvs/Trigger   latch goal & start posture regulation
  /stop_motion   std_srvs/Trigger   stop execution immediately
  /pickup        std_srvs/Trigger   run pickup servo sequence
  /drop          std_srvs/Trigger   run drop servo sequence

Usage
-----
  python3 camera_controller_node.py          # webcam device 0
  python3 camera_controller_node.py 1        # webcam device 1
  python3 camera_controller_node.py video.mp4
"""

import math
import sys
import time
import threading
from collections import deque

import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32MultiArray, Bool
from std_srvs.srv import Trigger

from workspace_tracker import WorkspaceTracker

# ── Workspace bounds [m] ───────────────────────────────────────────────────────
WORKSPACE_W_M      = 0.790
WORKSPACE_H_M      = 0.730
WORKSPACE_MARGIN_M = 0.05

# ── Object stability detection ─────────────────────────────────────────────────
STABILITY_WINDOW = 15
STABILITY_TOL_M  = 0.015
STABILITY_TOL_R  = 0.10
OBJECT_TIMEOUT_S = 1.5

# ── Control parameters ─────────────────────────────────────────────────────────
PUBLISH_HZ = 20.0   # cmd_vel rate [Hz]
V_MAX      = 0.1    # max linear speed  [m/s]

# ── Polar posture regulation gains ────────────────────────────────────────────
# All three must be > 0 for global asymptotic convergence.
K1 = 1.0   # linear speed gain   — drives ρ → 0
K2 = 2.0   # angular speed gain  — drives α → 0
K3 = 4.0   # heading correction  — drives final heading error → 0

MAX_LIN = V_MAX * 1.5   # cmd_vel clamp  [m/s]
MAX_ANG = 1.5           # cmd_vel clamp  [rad/s]

GOAL_TOLERANCE_M = 0.05  # stop when ρ < this  [m]
GOAL_TOLERANCE_R = 0.1   # stop when |θ_e| < this  [rad]

# ── Servo angles [degrees] ─────────────────────────────────────────────────────
SERVO1_DEFAULT      = 90.0
SERVO2_DEFAULT      = 90.0
PICKUP_SERVO2_GRAB  = 25.0
PICKUP_SERVO1_LIFT  = 10.0
PICKUP_SERVO2_HOLD  = 120.0
DROP_SERVO2_EXTEND  = 30.0
DROP_SERVO1_RELEASE = 120.0
SERVO_STEP_DELAY    = 0.3    # seconds between each servo step


# ── Utility ───────────────────────────────────────────────────────────────────
def _make_pose_stamped(x_m, y_m, theta, stamp):
    msg = PoseStamped()
    msg.header.stamp    = stamp
    msg.header.frame_id = 'workspace'
    msg.pose.position.x = x_m
    msg.pose.position.y = y_m
    msg.pose.orientation.z = math.sin(theta / 2.0)
    msg.pose.orientation.w = math.cos(theta / 2.0)
    return msg


# ── ROS 2 Node ────────────────────────────────────────────────────────────────
class CameraControllerNode(Node):
    def __init__(self, camera_source=0):
        super().__init__('camera_controller')

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_robot  = self.create_publisher(PoseStamped,       '/robot_pose',    10)
        self._pub_goal   = self.create_publisher(PoseStamped,       '/object_goal',   10)
        self._pub_cmd    = self.create_publisher(Twist,             '/cmd_vel',       10)
        self._pub_servo  = self.create_publisher(Float32MultiArray, '/servo/cmd',     10)
        self._pub_active = self.create_publisher(Bool,              '/motion_active', 10)
        self._pub_stable = self.create_publisher(Bool,              '/object_stable', 10)

        # ── Services ──────────────────────────────────────────────────────────
        self.create_service(Trigger, '/start_motion', self._start_cb)
        self.create_service(Trigger, '/stop_motion',  self._stop_cb)
        self.create_service(Trigger, '/pickup',       self._pickup_cb)
        self.create_service(Trigger, '/drop',         self._drop_cb)

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(PoseStamped, '/target_pose', self._target_pose_cb, 10)

        # ── Shared state (guarded by _lock) ───────────────────────────────────
        self._lock       = threading.Lock()
        self._robot_pose = None   # (x_m, y_m, theta)
        self._goal_pose  = None   # (x_m, y_m, theta)
        self._target_pose_override = False

        # Execution state
        self._active    = False
        self._exec_goal = None   # (x_m, y_m, theta) latched at /start_motion

        # Stability detection
        self._obj_history = deque(maxlen=STABILITY_WINDOW)
        self._last_obj_t  = time.monotonic()

        # ── Timers ────────────────────────────────────────────────────────────
        self.create_timer(1.0 / PUBLISH_HZ, self._control_loop)
        self.create_timer(0.5, self._stability_tick)

        # ── Camera background thread ──────────────────────────────────────────
        self._tracker    = WorkspaceTracker()
        self._cap        = cv2.VideoCapture(camera_source)
        self._cam_thread = threading.Thread(target=self._camera_loop, daemon=True)
        self._cam_thread.start()

        self.get_logger().info(
            'Camera controller ready.\n'
            '  /start_motion — drive to detected goal via posture regulation\n'
            '  /stop_motion  — halt immediately\n'
            '  /pickup       — run pickup servo sequence\n'
            '  /drop         — run drop servo sequence'
        )

    # ── Camera loop (background thread) ───────────────────────────────────────

    def _camera_loop(self):
        while rclpy.ok():
            ret, frame = self._cap.read()
            if not ret:
                continue

            robot_pose, _, goal_pose, vis = self._tracker.process_frame(frame)

            cv2.imshow('Workspace Tracker', vis)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            rp_pub = None
            gp_pub = None
            with self._lock:
                if robot_pose is not None:
                    self._robot_pose = (
                        robot_pose[0] / 1000.0,
                        robot_pose[1] / 1000.0,
                        robot_pose[2],
                    )
                    rp_pub = self._robot_pose

                if goal_pose is not None and not self._target_pose_override:
                    gx_m = goal_pose[0] / 1000.0
                    gy_m = goal_pose[1] / 1000.0
                    gth  = goal_pose[2]
                    self._goal_pose = (gx_m, gy_m, gth)
                    gp_pub = self._goal_pose

                    m = WORKSPACE_MARGIN_M
                    if (m <= gx_m <= WORKSPACE_W_M - m and
                            m <= gy_m <= WORKSPACE_H_M - m):
                        self._obj_history.append((gx_m, gy_m, gth))
                        self._last_obj_t = time.monotonic()
                    else:
                        self._obj_history.clear()

                elif self._target_pose_override and self._goal_pose is not None:
                    gp_pub = self._goal_pose

            stamp = self.get_clock().now().to_msg()
            if rp_pub is not None:
                self._pub_robot.publish(_make_pose_stamped(*rp_pub, stamp))
            if gp_pub is not None:
                self._pub_goal.publish(_make_pose_stamped(*gp_pub, stamp))

    # ── Services ──────────────────────────────────────────────────────────────

    def _start_cb(self, _, response):
        with self._lock:
            rp = self._robot_pose
            gp = self._goal_pose

        if rp is None:
            response.success = False
            response.message = 'No robot pose — check that the robot tag is visible.'
            return response
        if gp is None:
            response.success = False
            response.message = 'No goal pose — check that the object is visible.'
            return response

        with self._lock:
            self._exec_goal = gp
            self._active    = True

        self._reset_stability()
        self._publish_motion_active(True)
        self.get_logger().info(
            f'Motion started: ({rp[0]:.3f}, {rp[1]:.3f}, {math.degrees(rp[2]):.1f}°) → '
            f'({gp[0]:.3f}, {gp[1]:.3f}, {math.degrees(gp[2]):.1f}°)'
        )
        response.success = True
        response.message = 'Posture regulation started.'
        return response

    def _stop_cb(self, _, response):
        with self._lock:
            was_active                 = self._active
            self._active               = False
            self._target_pose_override = False
        self._pub_cmd.publish(Twist())
        self._publish_motion_active(False)
        response.success = True
        response.message = 'Motion stopped.' if was_active else 'Nothing was running.'
        self.get_logger().info(response.message)
        return response

    def _publish_motion_active(self, val: bool):
        msg = Bool()
        msg.data = val
        self._pub_active.publish(msg)

    def _target_pose_cb(self, msg: PoseStamped):
        x  = msg.pose.position.x
        y  = msg.pose.position.y
        th = 2.0 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)
        with self._lock:
            self._goal_pose            = (x, y, th)
            self._target_pose_override = True
        self.get_logger().info(
            f'/target_pose received: ({x:.3f}, {y:.3f}, {math.degrees(th):.1f}°)'
        )

    # ── Stability detection ────────────────────────────────────────────────────

    def _stability_tick(self):
        with self._lock:
            last_obj_t = self._last_obj_t

        if time.monotonic() - last_obj_t > OBJECT_TIMEOUT_S:
            with self._lock:
                if self._obj_history:
                    self._obj_history.clear()
                    self.get_logger().info('Object lost — stability reset.')

        msg = Bool()
        msg.data = self._is_stable()
        self._pub_stable.publish(msg)

    def _is_stable(self) -> bool:
        with self._lock:
            hist = list(self._obj_history)
        if len(hist) < STABILITY_WINDOW:
            return False
        xs  = [p[0] for p in hist]
        ys  = [p[1] for p in hist]
        ths = [p[2] for p in hist]
        return (max(xs) - min(xs) < STABILITY_TOL_M and
                max(ys) - min(ys) < STABILITY_TOL_M and
                max(ths) - min(ths) < STABILITY_TOL_R)

    def _reset_stability(self):
        with self._lock:
            self._obj_history.clear()
            self._last_obj_t = time.monotonic()

    # ── Servo helpers ─────────────────────────────────────────────────────────

    def _servo_cmd(self, s1: float, s2: float):
        msg = Float32MultiArray()
        msg.data = [float(s1), float(s2)]
        self._pub_servo.publish(msg)
        self.get_logger().info(f'Servo → S1={s1:.1f}°  S2={s2:.1f}°')

    def _pickup_cb(self, _, response):
        self._halt_motion()
        self._pickup_sequence()
        response.success = True
        response.message = 'Pickup complete.'
        return response

    def _drop_cb(self, _, response):
        self._halt_motion()
        self._drop_sequence()
        response.success = True
        response.message = 'Drop complete.'
        return response

    def _halt_motion(self):
        with self._lock:
            self._active               = False
            self._target_pose_override = False
        self._pub_cmd.publish(Twist())
        self._publish_motion_active(False)

    def _pickup_sequence(self):
        self.get_logger().info('Pickup: step 1 — default setup')
        self._servo_cmd(SERVO1_DEFAULT, SERVO2_DEFAULT)
        time.sleep(SERVO_STEP_DELAY)

        self.get_logger().info('Pickup: step 2 — servo 2 grab')
        self._servo_cmd(SERVO1_DEFAULT, PICKUP_SERVO2_GRAB)
        time.sleep(SERVO_STEP_DELAY)

        self.get_logger().info('Pickup: step 3 — servo 1 lift')
        self._servo_cmd(PICKUP_SERVO1_LIFT, PICKUP_SERVO2_GRAB)
        time.sleep(SERVO_STEP_DELAY)

        self.get_logger().info('Pickup: step 4 — servo 2 hold')
        self._servo_cmd(PICKUP_SERVO1_LIFT, PICKUP_SERVO2_HOLD)
        self.get_logger().info('Pickup sequence complete.')

    def _drop_sequence(self):
        self.get_logger().info('Drop: step 1 — servo 2 extend')
        self._servo_cmd(PICKUP_SERVO1_LIFT, DROP_SERVO2_EXTEND)
        time.sleep(SERVO_STEP_DELAY)

        self.get_logger().info('Drop: step 2 — servo 1 release')
        self._servo_cmd(DROP_SERVO1_RELEASE, DROP_SERVO2_EXTEND)
        time.sleep(SERVO_STEP_DELAY)

        self.get_logger().info('Drop: step 3 — default config')
        self._servo_cmd(SERVO1_DEFAULT, SERVO2_DEFAULT)
        self.get_logger().info('Drop sequence complete.')

    # ── Control loop (20 Hz) ──────────────────────────────────────────────────

    def _control_loop(self):
        with self._lock:
            if not self._active:
                return
            rp   = self._robot_pose
            goal = self._exec_goal

        if rp is None or goal is None:
            return

        curr_x, curr_y, curr_th = rp
        gx, gy, gth = goal

        dx  = curr_x - gx
        dy  = curr_y - gy
        x_e =  math.cos(gth) * dx + math.sin(gth) * dy
        y_e = -math.sin(gth) * dx + math.cos(gth) * dy
        th_e = math.atan2(math.sin(curr_th - gth), math.cos(curr_th - gth))

        rho = math.hypot(x_e, y_e)

        if rho < GOAL_TOLERANCE_M and abs(th_e) < GOAL_TOLERANCE_R:
            self._pub_cmd.publish(Twist())
            with self._lock:
                self._active               = False
                self._target_pose_override = False
            self._publish_motion_active(False)
            self.get_logger().info(
                f'Goal reached (ρ={rho*1000:.1f} mm, θ_e={math.degrees(th_e):.1f}°).'
            )
            return

        # Polar posture regulation control law
        alpha_raw = math.atan2(y_e, x_e) - th_e + math.pi
        alpha = math.atan2(math.sin(alpha_raw), math.cos(alpha_raw))
        delta = math.atan2(math.sin(alpha + th_e), math.cos(alpha + th_e))

        V = K1 * rho * math.cos(alpha)
        sinc_term = 1.0 if abs(alpha) < 1e-6 else math.sin(alpha) * math.cos(alpha) / alpha
        omega = K2 * alpha + K1 * sinc_term * (alpha + K3 * delta)

        cmd = Twist()
        cmd.linear.x  = float(max(-MAX_LIN, min(MAX_LIN, V)))
        cmd.angular.z = float(max(-MAX_ANG, min(MAX_ANG, omega)))
        self._pub_cmd.publish(cmd)


# ── Entry point ───────────────────────────────────────────────────────────────
def main(args=None):
    source = 0
    if len(sys.argv) > 1:
        source = int(sys.argv[1]) if sys.argv[1].isdigit() else sys.argv[1]

    rclpy.init(args=args)
    node = CameraControllerNode(camera_source=source)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
