"""Copies of the wrist maths inside Dex Teleop's GripperToGoal.update_goal().

WHAT THIS IS
    update_goal() (in the submodule) turns the requested gripper orientation
    into wrist yaw/pitch/roll and holds any joint that would go past its limit
    at its last value. That code is inline in update_goal(), not a function we
    can import, so the same few lines are copied here.

WHAT USES IT
    - GoalState.build(): clamp_wrist() pins an out-of-range joint at its limit
      BEFORE the goal goes to update_goal(), so upstream's hold never triggers
      and the overshoot can be dropped (PoseTarget.rebase_orientation). This
      shapes the orientation the robot is sent.
    - the HUD and --log-file: show the wrist angles the robot is being sent and
      which joint is "AT LIMIT"
    - startup: refuse to start if the live wrist is near pitch +/-90 deg, where
      the decode is ambiguous and the first tick could swing yaw/roll
    - each new grip: orientation_from_wrist() turns the robot's current wrist
      angles back into a gripper orientation, so gripping starts from where the
      wrist really is and doesn't move it

IF UPSTREAM CHANGES update_goal()'s wrist decoding, update these to match: the
wrist clamp, HUD, log and latch anchor all depend on it. Delete the copy if
upstream ever exposes the decode as a function.
"""

import numpy as np
from scipy.spatial.transform import Rotation

from .dex_bridge import angle_diff_rad

# Keep these in sync with update_goal() in the pinned submodule. They exist so
# the HUD, the log and the latch anchor show what the robot is actually told.
WRIST_NAMES = ("wrist_yaw", "wrist_pitch", "wrist_roll")
WRIST_JOINTS = ("wrist_yaw_joint", "wrist_pitch_joint", "wrist_roll_joint")


def decode_wrist(R):
    """Gripper rotation (IK world) -> (yaw, pitch, roll) before limit handling.

    Upstream: yaw = wrap(ypr[0] + pi), pitch = -ypr[1] (Stretch 4's pitch joint
    turns opposite to Stretch 3's), roll = wrap(ypr[2] + pi)."""
    ypr = Rotation.from_matrix(R).as_euler('ZXY', degrees=False)
    return (angle_diff_rad(ypr[0] + np.pi, 0.0),
            -ypr[1],
            angle_diff_rad(ypr[2] + np.pi, 0.0))


def commanded_wrist(R, limits=None):
    """decode_wrist() plus upstream's +2pi rescue for yaw AND roll: a value
    below the joint's lower limit is tried one turn up (both joints travel
    past +180 deg). ``limits`` is GripperToGoal.wrist_joint_limits; None skips
    the rescue (--dry-run without robot params)."""
    yaw, pitch, roll = decode_wrist(R)
    if limits:
        if yaw < limits['wrist_yaw_joint'][0]:
            yaw += 2.0 * np.pi
        if roll < limits['wrist_roll_joint'][0]:
            roll += 2.0 * np.pi
    return yaw, pitch, roll


def orientation_from_wrist(yaw, pitch, roll):
    """Inverse of decode_wrist(): the gripper rotation update_goal() decodes
    back to this wrist. (0, 0, 0) -> diag(1, -1, -1)."""
    return Rotation.from_euler(
        'ZXY', [yaw - np.pi, -pitch, roll - np.pi]).as_matrix()


# How far inside a wrist limit clamp_wrist() puts an out-of-range joint, so
# upstream's own decode of the result can't land a hair outside the limit.
WRIST_LIMIT_MARGIN_RAD = np.radians(0.5)


def clamp_wrist(R, limits, prev=None, margin=WRIST_LIMIT_MARGIN_RAD):
    """-> (R_cmd, pinned, yaw_shift) for the requested gripper rotation ``R``.

    Upstream holds an out-of-limit wrist joint at its LAST value, so after
    rotating past a limit you have to rotate back past it before that joint
    moves again. This clamps the joint to the limit instead (``margin``
    inside), so the caller can drop the overshoot (PoseTarget.rebase_
    orientation) and rotating back responds at once.

    Yaw and roll have a 50 deg gap between their limits; an overshoot goes to
    the limit nearer ``prev`` (the joint's current command), else nearer the
    value, so it never jumps to the far side.

    pinned:    e.g. ["wrist_yaw max"]; [] and R_cmd is R when all in range.
    yaw_shift: commanded minus requested yaw, rad (0 unless yaw is pinned).
    """
    if not limits:
        return R, [], 0.0
    vals = list(commanded_wrist(R, limits))
    req_yaw = vals[0]
    pinned = []
    for i, (name, joint) in enumerate(zip(WRIST_NAMES, WRIST_JOINTS)):
        lo, hi = limits[joint]
        v = vals[i]
        if lo <= v <= hi:
            continue
        if prev is not None and np.isfinite(prev[i]):
            to_hi = abs(prev[i] - hi) <= abs(prev[i] - lo)
        else:
            # Distance round the circle (yaw/roll gaps wrap; pitch's doesn't
            # matter -- its decode never exceeds +90 deg, so it only ever
            # goes out at the bottom).
            to_hi = abs(angle_diff_rad(v, hi)) <= abs(angle_diff_rad(v, lo))
        vals[i] = hi - margin if to_hi else lo + margin
        pinned.append(f"{name} {'max' if to_hi else 'min'}")
    if not pinned:
        return R, [], 0.0
    yaw_shift = angle_diff_rad(vals[0], req_yaw)
    return orientation_from_wrist(*vals), pinned, yaw_shift


def nearest_rotation(M):
    """The rotation matrix closest to ``M`` (SVD), so repeated re-anchoring
    can't drift away from a pure rotation."""
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R
