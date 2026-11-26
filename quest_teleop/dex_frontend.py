"""Quest-only front end: controller pose -> the goal dict Dex Teleop's update_goal() expects.

Dex Teleop splits the gripper pose in two and controls the halves independently:

    position    -> simple_ik.ik_rotary_base()  -> base rotation, lift, arm
    orientation -> ZXY Euler decomposition     -> wrist yaw, pitch, roll

Rotating your hand moves the wrist and not the arm; translating your hand moves
the arm and not the wrist. Upstream feeds this from ArUco tongs seen by a
webcam (goal_from_teleop.py); this module replaces that front end with the
Quest's 6-DOF controller pose.

Reference frames
----------------
Quest:     +X right, +Y up, -Z forward (away from you). Right-handed.
IK world:  +x robot's LEFT, +y robot's BACKWARD (the arm extends along -y),
           +z up. Origin on the floor at the base's rotation axis.

oculus_reader's app publishes inv(HeadPose) * ControllerPose, and the head pose
is never transmitted, so turning your head moves the reported pose with a still
hand. That is why everything here is CLUTCHED (relative to the pose at grip)
rather than absolute: re-gripping re-zeroes it.

Joint names are upstream's (lift_joint, arm_l4_joint, wrist_yaw_joint, ...).

Why this lives here and not in the submodule
--------------------------------------------
Dex Teleop (third_party/stretch4_dex_teleop) is used as-is from its pinned
commit: everything that commands the robot -- IK, smoothing, wrist decoding,
joint limits, base rotation -- is upstream's GripperToGoal.update_goal(),
reached only through dex_bridge.py. Upstream is built around ArUco tongs and a
webcam, so it has no equivalent of anything in this file. Everything here is
Quest-specific and stays in this repo:

  - frame mapping       Quest axes -> IK world axes
  - PoseTarget          the grip clutch: hand offset since the grip -> goal
  - GoalState           packs that goal into update_goal()'s arguments,
                        clamped to the lift/arm/wrist limits first (upstream
                        would skip the tick or hold the joint instead)
  - ChordTap            trigger + A / B settings chords
  - read_live_configuration, set_speed_profile, base_speed_mps
                        start from the live pose; switch speed while running

Why the wrist maths is copied (dex_wrist.py)
---------------------------------------------
update_goal() turns the goal orientation into wrist yaw/pitch/roll inline, not
in a function we can import. This front end needs the same numbers -- to clamp
the wrist at its limits before sending (GoalState.build), for the HUD
(GoalState.wrist_angles), the startup gimbal-lock check, and the grip anchor
(so gripping starts from where the wrist really is) -- so dex_wrist.py holds a
copy of that maths. Because the clamp shapes the orientation sent, the copy
must match upstream: if upstream changes its wrist decoding, update
dex_wrist.py to match; if upstream ever exposes it as a function, import that
instead and delete the copy.
"""

import math
import time
from collections import deque

import numpy as np
from scipy.spatial.transform import Rotation

from .config import DexCfg
from .dex_bridge import clamp_to_reach
from .dex_wrist import (WRIST_NAMES, clamp_wrist, commanded_wrist,
                        nearest_rotation)


# ===========================================================================
#  Math helpers
# ===========================================================================
def matrix_to_quat(R):
    """3x3 rotation matrix -> (w, x, y, z), via scipy (stable near 180 deg)."""
    x, y, z, w = Rotation.from_matrix(R).as_quat()
    return float(w), float(x), float(y), float(z)


def rotz(theta):
    """Rotation about +z (IK world). +theta matches the IK's base angle."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# Quest axes -> IK world axes. det = +1.
R_QUEST_TO_IK = np.array([[-1.0, 0.0, 0.0],    # world x (left)     = -quest x
                          [0.0, 0.0, 1.0],     # world y (backward) = +quest z
                          [0.0, 1.0, 0.0]])    # world z (up)       = +quest y



# ===========================================================================
#  Live robot pose
# ===========================================================================
def read_live_configuration(robot):
    """-> {'lift', 'arm', 'wrist_yaw', 'wrist_pitch', 'wrist_roll'} (m / rad)
    read from the robot now. Raises RuntimeError naming any joint that is
    missing or not finite -- the caller must stop before anything moves.
    Call robot.pull_status() first (RobotClient only refreshes there)."""
    out, missing = {}, []

    def take(name, getter):
        try:
            v = float(getter())
        except Exception:
            v = float("nan")
        if math.isfinite(v):
            out[name] = v
        else:
            missing.append(name)

    take("lift", lambda: robot.lift.status["pos"])
    take("arm", lambda: robot.arm.status["pos"])
    for j in WRIST_NAMES:
        take(j, lambda j=j: robot.end_of_arm.status[j]["pos"])
    if missing:
        raise RuntimeError(f"could not read live joint position(s): "
                           f"{', '.join(missing)}")
    return out


# ===========================================================================
#  PoseTarget -- clutch-relative hand motion anchored into the IK world frame
# ===========================================================================
class PoseTarget:
    """Latches an origin when the side grip is pressed, then maps the hand's
    offset from that origin -- 1:1, recomputed every tick, no integration --
    onto a goal pose. Same shape as upstream's
    ``(marker_pos - teleop_origin) + center_wrist_position``, except:

      1. R_QUEST_TO_IK maps the Quest's axes into the IK world frame.
      2. The anchor is re-captured on every latch from where the gripper is
         being commanded, not fixed at startup. Poses are headset-relative, so
         a fixed origin would drift with your head; and releasing then
         re-gripping lets you walk the arm past one arm's length, like lifting
         a mouse.
      3. The hand's POSITION offset is rotated by the base heading read at
         latch (frozen for the grip), so "hand forward" means the way the arm
         points when you grip. A live heading would turn a constant sideways
         hand offset into a constant base spin.
    """

    def __init__(self):
        self.T_h0 = None       # 4x4 hand pose at latch
        self.R_goal_0 = None   # gripper orientation at latch, IK world
        self.p0 = None         # wrist position at latch, IK world
        self.heading0 = 0.0    # base heading at latch, rad
        self.R_align = np.eye(3)  # controller facing -> gripper facing
        self.engaged = False
        self.latched = False
        self.events = deque(maxlen=6)

    def log(self, text):
        self.events.appendleft(f"{time.strftime('%H:%M:%S')}  {text}")

    def latch(self, R_F_0, p_F_0, heading0, hand_pose):
        self.T_h0 = hand_pose.copy()
        self.R_goal_0 = np.array(R_F_0, dtype=float)
        self.p0 = np.array(p_F_0, dtype=float)
        self.heading0 = float(heading0)
        self.R_align = self._facing_alignment(self.T_h0[:3, :3], self.R_goal_0)
        self.latched = True

    def clear(self):
        self.T_h0 = self.R_goal_0 = self.p0 = None
        self.heading0 = 0.0
        self.R_align = np.eye(3)
        self.latched = False

    # Below this horizontal length the pointing direction is too close to
    # straight up/down to give a heading (~20 deg from vertical).
    _MIN_HORIZONTAL = 0.35

    def _facing_alignment(self, R_h0, R_goal_0):
        """Rotation about vertical that turns the way the controller faces at
        the grip into the way the gripper faces, so the controller's "straight
        ahead" is treated as the gripper's "straight ahead" for this grip.
        Twist -> roll and tilt -> pitch then hold whatever the wrist yaw.
        Identity (the original behaviour) when they already face the same way
        or either points too close to straight up/down to have a heading."""
        # IK world: heading 0 points along -y (straight out along the arm).
        def heading(v):
            if math.hypot(v[0], v[1]) < self._MIN_HORIZONTAL:
                return None
            return math.atan2(v[0], -v[1])
        hand_fwd = R_QUEST_TO_IK @ (-R_h0[:, 2])   # controller points along -Z
        grip_fwd = R_goal_0[:, 1]               # gripper points along its y
        h_hand, h_grip = heading(hand_fwd), heading(grip_fwd)
        if h_hand is None or h_grip is None:
            self.log("facing offset skipped (pointing near vertical)")
            return np.eye(3)
        psi = math.atan2(math.sin(h_grip - h_hand), math.cos(h_grip - h_hand))
        self.log(f"facing offset {math.degrees(psi):+.0f} deg")
        return rotz(psi)

    def offset(self, hand_pose):
        """-> (dp, R_rel): hand offset since the latch, in IK world axes."""
        R_h0, p_h0 = self.T_h0[:3, :3], self.T_h0[:3, 3]
        R_h, p_h = hand_pose[:3, :3], hand_pose[:3, 3]
        # Displacement in the TRACKING frame, not the controller's own axes,
        # so a flat slide stays flat however the controller is held.
        dp = p_h - p_h0
        R_rel = R_h0.T @ R_h
        return R_QUEST_TO_IK @ dp, R_QUEST_TO_IK @ R_rel @ R_QUEST_TO_IK.T

    def target(self, hand_pose):
        """-> (R_target, p_target) in the IK world frame."""
        dp, _ = self.offset(hand_pose)
        R_h0, R_h = self.T_h0[:3, :3], hand_pose[:3, :3]
        dR = R_QUEST_TO_IK @ (R_h @ R_h0.T) @ R_QUEST_TO_IK.T
        A = self.R_align
        return A @ dR @ A.T @ self.R_goal_0, self.p0 + rotz(self.heading0) @ dp

    def rebase_position(self, p_t, p_cmd):
        """The target ``p_t`` was clamped to ``p_cmd``: shift the anchor so
        this hand pose maps to ``p_cmd``. Overshoot past a limit is dropped,
        so moving back responds at once. POSITION ONLY -- the orientation
        mapping is never touched."""
        self.p0 = self.p0 + (np.asarray(p_cmd, dtype=float) - p_t)

    def rebase_orientation(self, R_t, R_cmd, yaw_shift):
        """The target rotation ``R_t`` was clamped to ``R_cmd`` (a wrist joint
        at its limit): shift the anchor so this hand pose maps to ``R_cmd``.
        Overshoot is dropped, so rotating back responds at once.

        Yaw overshoot is a turn about vertical, so it is taken out of the
        facing alignment as well as the anchor: the controller's facing stays
        matched to the gripper's, and twist stays roll / tilt stays pitch.
        Called only on ticks where a wrist joint is clamped; re-projected to a
        pure rotation each time so it can't drift."""
        if yaw_shift:
            Z = rotz(yaw_shift)
            # Z A dR A^T Z^T Z R_goal_0 = Z R_t exactly.
            self.R_align = Z @ self.R_align
            self.R_goal_0 = Z @ self.R_goal_0
            R_t = Z @ R_t
        # Whatever is left (pitch/roll): R_t = M R_goal_0 with M fixed by the
        # hand pose, so R_goal_0 <- M^T R_cmd = R_goal_0 R_t^T R_cmd.
        self.R_goal_0 = nearest_rotation(self.R_goal_0 @ R_t.T @ R_cmd)
        self.R_align = nearest_rotation(self.R_align)

    def step(self, inp, current_pose_fn):
        """Latch on the grip's rising edge, clear on release or lost tracking.
        ``current_pose_fn()`` -> (R, p, heading) is only called on the latch."""
        if not inp.valid:
            if self.latched:
                self.log("controller lost -- clutch cleared")
            self.engaged = False
            self.clear()
            return
        if inp.inner and not self.engaged:
            R_F_0, p_F_0, heading0 = current_pose_fn()
            self.latch(R_F_0, p_F_0, heading0, inp.pose)
            self.log("origin latched")
        elif not inp.inner and self.engaged:
            self.clear()
            self.log("released -- clutch cleared")
        self.engaged = inp.inner


# ===========================================================================
#  GoalState -- the goal dict, plus what the HUD and latch anchor need
# ===========================================================================
class GoalState:
    """Builds update_goal()'s goal dict and keeps a preview for the HUD.

    build() also keeps the goal reachable: a target past a lift/arm limit is
    clamped to the nearest reachable point (dex_bridge.clamp_to_reach) instead
    of being sent as-is, because upstream skips the WHOLE tick -- base, lift,
    arm and wrist -- on an unreachable goal. ``pinned`` says which limit, and
    ``wrist_position`` (the next latch's anchor) is always reachable.
    update_goal() solves again for real; simple_ik is closed-form and cheap.
    """

    def __init__(self, simple_ik):
        self.simple_ik = simple_ik
        self.last_cfg = None        # last successful IK solve
        self.pinned = []            # e.g. ["lift max"]; [] when in reach
        self.wrist_pinned = []      # e.g. ["wrist_yaw max"]
        self.yaw_shift = 0.0        # clamped minus requested wrist yaw, rad
        self.wrist_position = None  # last position SENT (always reachable)
        self.R_request = None       # orientation SENT (wrist within limits)

    @property
    def unreachable(self):
        """True while the position sent is clamped (the log's column name)."""
        return bool(self.pinned)

    def build(self, R_t, p_t, wrist_limits=None, wrist_prev=None):
        """-> kwargs for update_goal() (no grip_width; the gripper is A/B).
        Compare the returned position / ``R_request`` with ``p_t`` / ``R_t``
        to see the clamps. ``wrist_limits`` None (dry run without params)
        skips the wrist clamp."""
        R_cmd, self.wrist_pinned, self.yaw_shift = clamp_wrist(
            R_t, wrist_limits, wrist_prev)
        self.R_request = R_cmd
        res = clamp_to_reach(self.simple_ik, p_t)
        if res is not None:
            p_cmd, self.last_cfg, self.pinned = res
            self.wrist_position = p_cmd
        elif self.wrist_position is not None:
            # Even the unconstrained solve failed: hold the last good point.
            p_cmd, self.pinned = self.wrist_position, ["no solve -- holding"]
        else:
            p_cmd, self.pinned = np.asarray(p_t, dtype=float), ["no solve"]
        return {'wrist_position': p_cmd,
                'gripper_x_axis': R_cmd[:, 0],
                'gripper_y_axis': R_cmd[:, 1],
                'gripper_z_axis': R_cmd[:, 2]}

    def wrist_angles(self, limits=None):
        if self.R_request is None:
            return None
        return commanded_wrist(self.R_request, limits)


# ===========================================================================
#  ChordTap -- trigger + tap A cycles speed, trigger + tap B cycles strength
# ===========================================================================
class ChordTap:
    """The gamepad's settings chords. Levels and their order come from
    stretch4_body's gamepad enums: speed slow -> default -> fast (never MAX),
    strength Low -> Medium -> High. step() returns events; the caller applies
    them to the robot.

    A tap is a press shorter than DexCfg.CHORD_TAP_MAX_S released while the
    trigger is still held. While the trigger is held, A/B must not move the
    gripper.
    """

    STRENGTH_LABELS = {"high_sensitivity_manipulation": "Low",
                       "default": "Medium",
                       "strong_manipulation": "High"}

    def __init__(self, speed_name="default", clock=time.monotonic):
        from stretch4_body.core.gamepad_enums import (GuardedContactSensitivity,
                                                      MotionProfile)
        starts = [p for p in MotionProfile
                  if p is not MotionProfile.MAX and p.get_name() == speed_name]
        if not starts:
            raise ValueError(f"speed {speed_name!r} is not one of slow/default/fast")
        self.speed = starts[0]
        self.strength = GuardedContactSensitivity.MEDIUM
        self._clock = clock
        self._was_down = {"a": False, "b": False}
        self._down_at = {"a": None, "b": None}
        self._chord = {"a": False, "b": False}

    @property
    def speed_name(self):
        return self.speed.get_name()

    @property
    def strength_name(self):
        return self.strength.get_name()

    @property
    def strength_label(self):
        return self.STRENGTH_LABELS.get(self.strength_name, self.strength_name)

    def step(self, trig_pulled, a, b, enabled=True):
        """-> (events, block_gripper). enabled=False (runstopped) fires nothing
        and drops any press in progress."""
        now = self._clock()
        events = []
        for key, down in (("a", bool(a)), ("b", bool(b))):
            was = self._was_down[key]
            if down and not was:
                self._down_at[key] = now if enabled else None
                self._chord[key] = bool(trig_pulled)
            elif was and not down:
                started = self._down_at[key]
                if (enabled and trig_pulled and started is not None
                        and now - started < DexCfg.CHORD_TAP_MAX_S):
                    events.append(self._cycle(key))
                self._down_at[key] = None
                self._chord[key] = False
            if down and not enabled:
                self._down_at[key] = None
            self._was_down[key] = down
        block = bool(trig_pulled) or any(
            self._chord[k] and self._was_down[k] for k in ("a", "b"))
        return events, block

    def _cycle(self, key):
        if key == "a":
            self.speed = self.speed.cycle(is_forward=True)
            return ("speed", self.speed_name)
        self.strength = self.strength.cycle(is_forward=True)
        return ("strength", self.strength_name)


def set_speed_profile(gripper_to_goal, name):
    """Switch the IK-driven joints to motion profile ``name`` while running.

    RobotMove precomputes every profile in all_speeds and reads
    move_to_functions on each to_configuration(), so re-pointing these three
    attributes is the whole switch. KeyError for an unknown name."""
    rm = gripper_to_goal.robot_move
    entry = rm.all_speeds[name]
    rm.speed = name
    rm.move_to_functions = entry["move_to_functions"]
    rm.move_to_settings = entry["move_to_settings"]


def base_speed_mps(name):
    """-> (m/s, m/s^2) for the joystick base at profile ``name``, from the
    same RobotParams table the gamepad uses."""
    from stretch4_body.core.robot_params import RobotParams
    m = RobotParams().get_params()[1]["omnibase"]["motion"][name]
    return float(m["vel_xy_m"]), float(m["accel_xy_m"])
