"""The one place that touches the stretch4_dex_teleop submodule.

Dex Teleop (third_party/stretch4_dex_teleop, pinned as a git submodule) owns
the command path: closed-form position IK, smoothing, wrist decoding, joint
limits and base rotation. This repo only supplies the Quest front end.

Upstream is a folder of scripts rather than a package, so its directory goes on
sys.path and its modules are imported by their plain names (simple_ik,
dex_teleop_parameters, gripper_to_goal, robot_move).
"""

import math
import os
import sys

import numpy as np

DEX_DIR = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir,
    "third_party", "stretch4_dex_teleop"))

if not os.path.isfile(os.path.join(DEX_DIR, "simple_ik.py")):
    raise ImportError(
        f"stretch4_dex_teleop not found at {DEX_DIR}. "
        f"Run: git submodule update --init")

if DEX_DIR not in sys.path:
    sys.path.insert(0, DEX_DIR)

# Safe without the robot library: neither imports stretch4_body at module level.
import dex_teleop_parameters as dt          # noqa: E402
import simple_ik as si                       # noqa: E402
from simple_ik import angle_diff_rad         # noqa: E402,F401  (re-export)


def _check_from_submodule(*modules):
    """Refuse to run on a Dex Teleop module that isn't the pinned submodule
    (a stray simple_ik.py, an old O-RDC copy, something on PYTHONPATH)."""
    for m in modules:
        where = os.path.dirname(os.path.abspath(getattr(m, "__file__", "") or ""))
        if where != DEX_DIR:
            raise ImportError(
                f"{m.__name__} was imported from {where}, not the "
                f"stretch4_dex_teleop submodule at {DEX_DIR}. Remove or rename "
                f"the other copy.")


_check_from_submodule(dt, si)


def make_simple_ik():
    """Upstream's SimpleIK (it finds its own URDFs, from any directory)."""
    return si.SimpleIK()


# How far inside a lift/arm limit clamp_to_reach() puts the goal, so FK -> IK
# round-off can't land it a hair outside ik_rotary_base()'s strict check.
REACH_MARGIN_M = 0.001


def safe_ik(simple_ik, p):
    """ik_rotary_base() or None. Also None where the closed form divides by
    zero, which upstream would raise."""
    try:
        return simple_ik.ik_rotary_base(np.asarray(p, dtype=float))
    except (ZeroDivisionError, FloatingPointError, ValueError):
        return None


def clamp_to_reach(simple_ik, p, margin=REACH_MARGIN_M):
    """-> (p_reachable, cfg, pinned) for wrist position ``p``, or None.

    Upstream's ik_rotary_base() returns None for any goal even 1 mm past a
    lift/arm limit, and update_goal() then skips the WHOLE tick -- base, lift,
    arm and wrist all freeze. This instead solves without the limit check,
    clips lift and arm to their limits (``margin`` inside), and runs FK back:
    the nearest reachable point along the joints that are out. Base angle and
    the free joint are kept (lift pinned at the top, the arm still extends).

    ``pinned`` names what was clipped, e.g. ["lift max"]; [] if ``p`` was
    already reachable. None if even the unconstrained solve fails."""
    p = np.asarray(p, dtype=float)
    cfg = safe_ik(simple_ik, p)
    if cfg is not None:
        return p, cfg, []
    try:
        T, L, A = si.ie.calibrated_ik_with_rotary_base(
            p, b1=simple_ik.b1, l_vector=simple_ik.l_vec,
            a_vector=simple_ik.a_vec)
    except (ZeroDivisionError, FloatingPointError, ValueError):
        return None
    if not all(math.isfinite(float(v)) for v in (T, L, A)):
        return None
    joints = {'lift_joint': ("lift", float(L) - simple_ik.l_offset),
              'arm_l4_joint': ("arm", float(A) - simple_ik.a_offset)}
    clamped, pinned = {}, []
    for jn, (label, v) in joints.items():
        lo, hi = simple_ik.all_joint_limits[jn]
        if v > hi - margin:
            pinned.append(f"{label} max")
        elif v < lo + margin:
            pinned.append(f"{label} min")
        clamped[jn] = min(max(v, lo + margin), hi - margin)
    p_c = np.array(simple_ik.fk_rotary_base({
        'mobile_base_rotation_joint': float(T), **clamped}), dtype=float)
    cfg = safe_ik(simple_ik, p_c)
    if cfg is None:
        return None
    return p_c, cfg, pinned or ["reach"]


def find_gripper(robot):
    """Upstream's gripper lookup -- the same one GripperToGoal and RobotMove use,
    so the front end and the command path can never disagree about the tool."""
    import robot_move as rm
    _check_from_submodule(rm)
    return rm.find_gripper(robot)


def make_gripper_to_goal(robot, use_robot_client, **kwargs):
    """GripperToGoal driving ``robot`` (already started; the caller stops it --
    GripperToGoal never stops a robot it was handed).

    kwargs go straight to GripperToGoal: robot_speed, starting_configuration,
    robot_allowed_to_move. The constructor moves the robot to
    starting_configuration, so pass the robot's live pose to move nothing.
    """
    import gripper_to_goal as g2g   # imports stretch4_body; lazy for --dry-run
    _check_from_submodule(g2g, g2g.si, g2g.rm, g2g.dt)

    # Read by GripperToGoal (pull_status each update_goal) and by robot_move;
    # upstream requires it to match the robot's type when one is passed in.
    dt.use_robot_client = bool(use_robot_client)
    return g2g.GripperToGoal(robot=robot, **kwargs)


def rezero_base_heading(gripper_to_goal):
    """Make the base's CURRENT heading Dex Teleop's 0 rad. -> the heading it
    replaced (rad, wrapped to [-pi, pi]), or None if odometry is unreadable.

    Why: update_goal() works in a frame fixed at the heading it was started
    with, and breaks once the base has turned more than 180 deg from there.
    simple_ik reports the goal base angle in [-pi, pi], so 181 deg arrives as
    -179 deg; the exponential smoothing then averages +179 and -179 into 0
    (the opposite direction), and the error (goal - current) is not wrapped,
    so the base spins the long way round at the rotate_by clamp. Re-zeroing
    on every grip keeps the goal near 0 rad, so this only recurs if the base
    turns more than 180 deg within ONE grip.

    The caller must rotate anything it holds in the old frame (the latch
    anchor) by -theta. Reaches into two upstream attributes:
    base_odometry (BaseOdom) and filtered_wrist_position_configuration[0]
    (the smoothed base angle). Update this if upstream renames them.
    """
    odom = gripper_to_goal.base_odometry
    theta = float(odom.get()['theta'])
    if not math.isfinite(theta):
        return None
    theta = angle_diff_rad(theta, 0.0)
    odom.reset()
    f = gripper_to_goal.filtered_wrist_position_configuration
    f[0] = angle_diff_rad(f[0] - theta, 0.0)
    return theta
