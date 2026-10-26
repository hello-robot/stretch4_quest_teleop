"""Controller input -> robot commands (a Decision per tick)."""

import math
import time
from collections import deque

import numpy as np

from .config import Cfg


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def rotvec_from_matrix(R):
    """Axis-angle (rotation vector, radians) of a 3x3 rotation matrix."""
    cos_theta = clamp((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    theta = math.acos(cos_theta)
    if theta < 1e-7:
        return np.zeros(3)
    sin_theta = math.sin(theta)
    if abs(sin_theta) < 1e-7:
        axis = np.sqrt(np.maximum((np.diag(R) + 1.0) / 2.0, 0.0))
        return axis * theta
    axis = np.array([R[2, 1] - R[1, 2],
                     R[0, 2] - R[2, 0],
                     R[1, 0] - R[0, 1]]) / (2.0 * sin_theta)
    return axis * theta


def _skew(v):
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def rotation_between(a, b):
    """Minimal rotation matrix carrying unit vector ``a`` onto unit ``b``."""
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    s = float(np.linalg.norm(v))
    if s < 1e-9:
        if c > 0.0:
            return np.eye(3)
        perp = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 \
            else np.array([0.0, 1.0, 0.0])
        axis = np.cross(a, perp)
        axis /= np.linalg.norm(axis)
        K = _skew(axis)
        return np.eye(3) + 2.0 * (K @ K)
    K = _skew(v / s)
    theta = math.atan2(s, c)
    return np.eye(3) + math.sin(theta) * K + (1.0 - math.cos(theta)) * (K @ K)


def aim_angles(R_rel, fwd0, up_axis, right_axis):
    """Yaw and pitch (radians) of the controller's pointing direction.
    Twisting about that direction changes neither."""
    fwd1 = R_rel @ fwd0
    n = np.linalg.norm(fwd1)
    if n < 1e-9:
        return 0.0, 0.0
    swing_rv = rotvec_from_matrix(rotation_between(fwd0, fwd1 / n))
    return float(swing_rv @ up_axis), float(swing_rv @ right_axis)


def offset_to_unit(delta, deadzone, full_scale):
    """Signed offset -> [-1, 1], linear between the deadzone and full scale."""
    mag = abs(delta)
    if mag < deadzone:
        return 0.0
    span = max(full_scale - deadzone, 1e-9)
    unit = clamp((mag - deadzone) / span, 0.0, 1.0)
    return unit if delta > 0 else -unit


def js_unit(v):
    if abs(v) < Cfg.JS_DEADZONE:
        return 0.0
    span = max(1.0 - Cfg.JS_DEADZONE, 1e-9)
    unit = clamp((abs(v) - Cfg.JS_DEADZONE) / span, 0.0, 1.0)
    return unit if v > 0 else -unit


class HandInput:
    """Decoded right-controller state for one tick."""

    __slots__ = ("pose", "js_x", "js_y", "inner", "front", "a", "b", "valid")

    def __init__(self):
        self.pose = None
        self.js_x = 0.0
        self.js_y = 0.0
        self.inner = False    # side grip  -> 6D mode
        self.front = False    # index trigger -> wrist mode
        self.a = False
        self.b = False
        self.valid = False

    @staticmethod
    def decode(transforms, buttons):
        s = HandInput()
        if not transforms or "r" not in transforms:
            return s
        pose = np.asarray(transforms["r"], dtype=float)
        if pose.shape != (4, 4) or not np.all(np.isfinite(pose)):
            return s
        s.pose = pose
        b = buttons or {}

        js = b.get("rightJS", (0.0, 0.0))
        if isinstance(js, (tuple, list)) and len(js) >= 2:
            s.js_x, s.js_y = float(js[0]), float(js[1])

        # Inner = side grip.  Prefer the analog reading, fall back to the bit.
        grip = b.get("rightGrip", None)
        if isinstance(grip, (tuple, list)) and grip:
            s.inner = float(grip[0]) > 0.5
        s.inner = s.inner or bool(b.get("RG", False))

        # Front = index trigger.
        trig = b.get("rightTrig", None)
        if isinstance(trig, (tuple, list)) and trig:
            s.front = float(trig[0]) > 0.5
        s.front = s.front or bool(b.get("RTr", False))

        s.a = bool(b.get("A", False))
        s.b = bool(b.get("B", False))
        s.valid = True
        return s


class Decision:
    def __init__(self):
        self.base_x = 0.0          # forward / back
        self.base_y = 0.0          # strafe
        self.base_w = 0.0          # turn
        self.lift = 0.0
        self.arm = 0.0
        self.wrist_yaw = 0.0
        self.wrist_pitch = 0.0
        self.wrist_roll = 0.0
        self.gripper = "hold"      # "close" | "open" | "hold"
        self.mode_6d = False       # inner button held
        self.mode_wrist = False    # front button held
        self.raw_d_up = 0.0
        self.raw_d_fwd = 0.0
        self.raw_yaw_deg = 0.0
        self.raw_pitch_deg = 0.0
        self.reasons = []


class Mapping:
    """Controller input -> Decision.  Holds the clutch origin."""

    def __init__(self, js_x_mode=Cfg.JS_X_MODE):
        self.js_x_mode = js_x_mode
        self.origin_pose = None
        self.prev_mode = (False, False)
        self.events = deque(maxlen=6)

    def log_event(self, text):
        self.events.appendleft(f"{time.strftime('%H:%M:%S')}  {text}")

    def step(self, inp: HandInput) -> Decision:
        d = Decision()

        if not inp.valid:
            if self.origin_pose is not None:
                self.log_event("controller lost -> all motion stopped")
            self.origin_pose = None
            self.prev_mode = (False, False)
            d.gripper = "hold"
            d.reasons.append("no controller data -- everything held at zero")
            return d

        # ----- gripper: A/B buttons ------------------------------
        d.gripper = "open" if inp.b else ("close" if inp.a else "hold")

        d.mode_6d = inp.inner
        d.mode_wrist = inp.front

        # ----- joystick --------------------------------------------------
        # Wrist mode: X rolls the wrist and the base is off, so the robot
        # cannot wander while aiming.  Otherwise it drives the base.
        jx, jy = js_unit(inp.js_x), js_unit(inp.js_y)
        if inp.front:
            d.wrist_roll = Cfg.ROLL_SIGN * jx
        else:
            d.base_x = jy
            if self.js_x_mode == "turn":
                d.base_w = -jx
            else:
                d.base_y = -jx

        # ----- clutch ----------------------------------------------------
        mode = (inp.inner, inp.front)
        engaged = inp.inner or inp.front
        if not engaged:
            if self.origin_pose is not None:
                self.log_event("released -> 6D/wrist stopped")
            self.origin_pose = None
            self.prev_mode = mode
            d.reasons.append(
                "hold the INNER button for 6D, the FRONT button for wrist")
            return d

        if self.origin_pose is None or mode != self.prev_mode:
            self.origin_pose = inp.pose.copy()
            self.log_event(
                f"origin latched  [{'6D' if inp.inner else '--'}]"
                f"[{'WRIST' if inp.front else '-----'}]")
        self.prev_mode = mode

        # ----- offsets from the latched origin ---------------------------
        dp = inp.pose[:3, 3] - self.origin_pose[:3, 3]
        d.raw_d_up = Cfg.UP_SIGN * dp[Cfg.UP_AXIS]
        d.raw_d_fwd = Cfg.FWD_SIGN * dp[Cfg.FWD_AXIS]

        R_o = self.origin_pose[:3, :3]
        R_rel = inp.pose[:3, :3] @ R_o.T
        fwd0 = Cfg.CTRL_FWD_SIGN * R_o[:, Cfg.CTRL_FWD_AXIS]
        fwd0 = fwd0 / max(np.linalg.norm(fwd0), 1e-9)
        yaw, pitch = aim_angles(R_rel, fwd0,
                                np.asarray(Cfg.UP_VEC, dtype=float),
                                np.asarray(Cfg.RIGHT_VEC, dtype=float))
        d.raw_yaw_deg = math.degrees(Cfg.YAW_SIGN * yaw)
        d.raw_pitch_deg = math.degrees(Cfg.PITCH_SIGN * pitch)

        def rot_unit(deg):
            return offset_to_unit(deg, Cfg.ROT_DEADZONE_DEG,
                                  Cfg.ROT_FULL_SCALE_DEG)

        # ----- TRANSLATION -> lift / arm, gated on the inner button ------
        if inp.inner:
            d.lift = offset_to_unit(d.raw_d_up, Cfg.LIFT_DEADZONE_M,
                                    Cfg.LIFT_FULL_SCALE_M)
            d.arm = offset_to_unit(d.raw_d_fwd, Cfg.ARM_DEADZONE_M,
                                   Cfg.ARM_FULL_SCALE_M)

        # ----- ROTATION -> wrist if the front button is down, else base --
        # Front wins outright, so holding both gives lift/arm from
        # translation AND wrist yaw/pitch from rotation, simultaneously.
        if inp.front:
            d.wrist_yaw = rot_unit(d.raw_yaw_deg)
            d.wrist_pitch = rot_unit(d.raw_pitch_deg)
            if inp.inner:
                d.reasons.append("6D + WRIST: lift/arm and wrist together")
            else:
                d.reasons.append("WRIST: aim -> wrist yaw/pitch, joystick X -> roll")
        elif inp.inner:
            d.base_w = rot_unit(d.raw_yaw_deg)
            d.reasons.append("6D: lift / arm / base turn")

        self._scale(d)
        return d

    @staticmethod
    def _scale(d):
        g = Cfg.SPEED_SCALE
        b = g * Cfg.BASE_SPEED_SCALE
        la = g * Cfg.LIFT_ARM_SPEED_SCALE
        w = g * Cfg.WRIST_SPEED_SCALE
        d.base_x = clamp(d.base_x * b, -1.0, 1.0)
        d.base_y = clamp(d.base_y * b, -1.0, 1.0)
        d.base_w = clamp(d.base_w * b, -1.0, 1.0)
        d.lift = clamp(d.lift * la, -1.0, 1.0)
        d.arm = clamp(d.arm * la, -1.0, 1.0)
        d.wrist_yaw = clamp(d.wrist_yaw * w, -1.0, 1.0)
        d.wrist_pitch = clamp(d.wrist_pitch * w, -1.0, 1.0)
        d.wrist_roll = clamp(d.wrist_roll * w, -1.0, 1.0)
