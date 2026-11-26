#!/usr/bin/env python3
"""
Teleoperate a Stretch 4 from the right Meta Quest controller using Dex Teleop's
control scheme: hold the side grip and the gripper follows your hand 1:1.

    python3 stretch_quest_dex_teleop.py --dry-run     # no robot; check the mapping
    python3 stretch_quest_dex_teleop.py --self-test   # no headset; MOVES THE ARM
    python3 stretch_quest_dex_teleop.py               # teleoperate
    python3 stretch_quest_dex_teleop.py --help-map    # print the control mapping

The command path (IK, smoothing, wrist decoding, joint limits, base rotation) is
Dex Teleop itself, from the third_party/stretch4_dex_teleop submodule. This
script is the Quest front end: it turns the controller pose into the goal dict
GripperToGoal.update_goal() expects, and drives the gripper (A/B), the
joystick base and the settings chords itself.

Unlike stretch_quest_teleop.py (hand offset sets joint SPEEDS), this is
position control: the hand's offset since the grip sets where the gripper IS.
"""

import argparse
import csv
import math
import os
import signal
import sys
import time
from collections import deque

import numpy as np

from quest_teleop.config import DexCfg
from quest_teleop.dex_bridge import angle_diff_rad, rezero_base_heading
from quest_teleop.dex_bridge import dt as dex_params
from quest_teleop.dex_hud import DEX_MAPPING_TEXT, DexHud
from quest_teleop.dex_frontend import (ChordTap, GoalState, PoseTarget,
                                       base_speed_mps, matrix_to_quat, rotz,
                                       read_live_configuration,
                                       set_speed_profile)
from quest_teleop.dex_wrist import (WRIST_NAMES, decode_wrist,
                                    orientation_from_wrist)
from quest_teleop.headset_link import HeadsetLink
from quest_teleop.mapping import HandInput, js_unit

LOG_FIELDS = ["t", "fresh", "valid", "inner", "latched",
              "hand_x", "hand_y", "hand_z",
              "hand_qw", "hand_qx", "hand_qy", "hand_qz",
              "dp_x", "dp_y", "dp_z", "dist_m",
              "goal_x", "goal_y", "goal_z",
              "goal_qw", "goal_qx", "goal_qy", "goal_qz",
              "unreachable",
              "base_rot", "lift", "arm", "yaw", "pitch", "roll",
              "js_x", "js_y", "cmd_vx", "cmd_vy", "cmd_w", "odom_theta"]


def parse_args():
    ap = argparse.ArgumentParser(
        description="Teleoperate Stretch 4 from the right Quest controller "
                    "using Dex Teleop's control scheme.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=DEX_MAPPING_TEXT)
    ap.add_argument("--dry-run", action="store_true",
                    help="run the goal + IK path every tick but never touch "
                         "the robot")
    ap.add_argument("--self-test", action="store_true",
                    help="MOVES THE ARM: start the robot, go to Dex Teleop's "
                         "starting configuration, stop. No headset needed.")
    ap.add_argument("-d", "--direct", action="store_true",
                    help="use the direct stretch4_body API, not the server")
    ap.add_argument("--no-hud", action="store_true",
                    help="disable the terminal dashboard")
    ap.add_argument("--rate", type=float, default=DexCfg.RATE_HZ,
                    help=f"control loop rate in Hz (default {DexCfg.RATE_HZ:g})")
    ap.add_argument("--speed", default="default",
                    choices=["slow", "default", "fast"],
                    help="starting motion profile for the IK-driven joints and "
                         "the joystick base. Cycle it live with trigger + A.")
    ap.add_argument("--js-x", dest="js_x_mode", default=DexCfg.JS_X_MODE,
                    choices=["strafe", "turn"],
                    help="what joystick left/right does while the grip is "
                         "released")
    ap.add_argument("--ground", action="store_true",
                    help="--self-test / --dry-run only: use the low (ground "
                         "manipulation) starting lift height")
    ap.add_argument("--no-heading-frame", action="store_true",
                    help="orient hand motion to the startup heading instead "
                         "of the base heading at each grip")
    ap.add_argument("--orient-debug", action="store_true",
                    help="print the controller quaternion and decoded wrist "
                         "angles every tick")
    ap.add_argument("--ip", default=None, help="Quest IP for wireless ADB")
    ap.add_argument("--no-restart-app", dest="restart_app",
                    action="store_false", default=True,
                    help="reuse the headset app instance left by a previous "
                         "run instead of force-stopping and relaunching it")
    ap.add_argument("--link-timeout", type=float, default=5.0,
                    help="seconds to wait for the first pose before printing "
                         "what's wrong; it keeps waiting after that")
    ap.add_argument("--stop-app-on-exit", action="store_true",
                    help="force-stop the headset app when quitting")
    ap.add_argument("--log-file", default=None,
                    help="write a CSV row per tick")
    ap.add_argument("--help-map", action="store_true",
                    help="print the control mapping and exit")
    return ap.parse_args()


def build_robot(direct):
    """Start the robot; None (and a printed reason) if it can't be driven."""
    if direct:
        from stretch4_body.robot.robot import Robot
        robot = Robot()
    else:
        from stretch4_body.robot.robot_client import RobotClient
        robot = RobotClient(client_id="stretch_quest_dex_teleop")
    print(f"Starting robot ({'direct' if direct else 'server'}) ...")
    if not robot.startup():
        print("ERROR: robot.startup() failed.", file=sys.stderr)
        return None
    if robot.is_runstopped():
        print("\nNOTE: the robot is RUNSTOPPED -- nothing will move until you "
              "twist the runstop button on the head to release it.",
              file=sys.stderr)
    if not robot.is_homed():
        print("\nERROR: the robot is not homed. Run `stretch_robot_home` first.",
              file=sys.stderr)
        robot.stop()
        return None
    return robot


def make_gripper_cmd(robot):
    """-> (gripper name, gamepad gripper command). The name comes from Dex
    Teleop's own find_gripper(), so the front end and GripperToGoal always agree
    on which tool is fitted. CommandToolPosition handles SG4, SG4 v2 and PG4."""
    from stretch4_body.core import gamepad_joints
    from quest_teleop.dex_bridge import find_gripper
    name = find_gripper(robot).name
    cmd = gamepad_joints.CommandToolPosition(
        name=name, motion_profile=DexCfg.GRIPPER_MOTION_PROFILE)
    return name, cmd


def apply_gripper(cmd, robot, want):
    """"close" / "open" / "hold". Only queues; the caller pushes."""
    if want == "close":
        cmd.close_gripper(robot)
    elif want == "open":
        cmd.open_gripper(robot)
    else:
        cmd.stop_gripper(robot)


def lift_middle_for(ground):
    """Dex Teleop's parked lift height. Not ground mode reads the lift maximum
    from RobotParams; fall back to ground mode (no robot library needed) so
    --dry-run still runs on a laptop."""
    try:
        return dex_params.get_lift_middle(ground)
    except ImportError as e:
        print(f"NOTE: robot params unavailable ({e}); dry-run anchors at the "
              f"ground-mode lift height.", file=sys.stderr)
        return dex_params.get_lift_middle(True)


def read_wrist_limits_from_params():
    """Wrist limits the way GripperToGoal reads them, for --dry-run. None if
    the robot library isn't installed."""
    try:
        from stretch4_body.core.robot_params import RobotParams
        p = RobotParams().get_params()[1]
        return {f"{j}_joint": tuple(math.radians(v) for v in p[j]['range_deg'])
                for j in WRIST_NAMES}
    except Exception:
        return None


def self_test(args):
    """Exercise the submodule's command path on the robot. No headset."""
    from quest_teleop.dex_bridge import make_gripper_to_goal
    lift_middle = dex_params.get_lift_middle(args.ground)
    starting = dex_params.get_starting_configuration(lift_middle)
    center = dex_params.get_center_configuration(lift_middle)
    print("\n*** --self-test MOVES THE ARM to Dex Teleop's starting "
          f"configuration (lift {lift_middle:.2f} m). Make sure it is clear. ***")
    print("Ctrl-C now to abort; starting in 5s ...", flush=True)
    try:
        time.sleep(5.0)
    except KeyboardInterrupt:
        print("aborted.")
        return 130
    robot = build_robot(args.direct)
    if robot is None:
        return 1
    try:
        g2g = make_gripper_to_goal(robot, use_robot_client=not args.direct,
                                   robot_speed=args.speed,
                                   starting_configuration=starting,
                                   robot_allowed_to_move=True)
        print("\nwrist limits (RobotParams):")
        for k, v in g2g.wrist_joint_limits.items():
            print(f"   {k:20s} [{v[0]:+.3f}, {v[1]:+.3f}] rad")
        p = g2g.simple_ik.fk_rotary_base(center)
        print(f"\ncenter_wrist_position = {np.round(p, 4)}")
        print(f"gripper = {g2g.gripper_name}")
        print("\nself-test OK -- the submodule drives every joint this script uses.")
    finally:
        robot.stop()
    return 0


def start_headset(args, on_link_event):
    """-> a live HeadsetLink, or None. Like stretch_quest_teleop.py, it keeps
    waiting (and says what's wrong) rather than giving up."""
    try:
        import oculus_reader.reader  # noqa: F401  (import check only)
    except ImportError as e:
        print(f"ERROR: cannot import oculus_reader ({e}).\n"
              f"       Install it by following the Install steps in README.md",
              file=sys.stderr)
        return None
    print("Connecting to the Quest over ADB ...", flush=True)
    try:
        link = HeadsetLink(ip_address=args.ip, on_event=on_link_event)
    except (Exception, SystemExit) as e:
        # oculus_reader calls exit(1) from inside get_usb_device().
        print(f"\nERROR: could not reach the headset over ADB ({e}).\n"
              f"       Check `adb devices` shows it and that you accepted "
              f"the USB-debugging prompt inside the headset.", file=sys.stderr)
        return None
    if args.restart_app:
        print("Restarting the headset app for a clean start ...", flush=True)
    print("Waiting for the first controller pose ...", flush=True)
    if not link.start(restart_app=args.restart_app, wait_s=args.link_timeout):
        print(f"\nNo controller data after {args.link_timeout:.0f}s.  Do what's "
              f"below -- it carries on by itself once it works "
              f"(Ctrl-C to quit).", file=sys.stderr, flush=True)
        t0, last_report, last_note = time.time(), None, time.time()
        try:
            while not link.wait_for_pose(5.0):
                link.bring_app_to_front()
                report = link.diagnose()
                if report != last_report:
                    print("", file=sys.stderr)
                    for line in report or ["- No controller data yet.",
                                           "    -> Press any button on both "
                                           "controllers."]:
                        print(f"  {line}", file=sys.stderr)
                    last_report, last_note = report, time.time()
                elif time.time() - last_note > 30:
                    print(f"  ... still waiting "
                          f"({time.time() - t0 + args.link_timeout:.0f}s)",
                          file=sys.stderr, flush=True)
                    last_note = time.time()
        except KeyboardInterrupt:
            link.stop()
            return None
    print("Headset link up -- poses flowing.", flush=True)
    return link


def main():
    args = parse_args()
    if args.help_map:
        print(DEX_MAPPING_TEXT)
        return 0
    DexCfg.JS_X_MODE = args.js_x_mode

    if args.self_test:
        return self_test(args)

    # ---- headset ------------------------------------------------------
    link_events = deque(maxlen=6)
    link_log = []
    hud = None

    def on_link_event(msg):
        line = f"{time.strftime('%H:%M:%S')}  LINK: {msg}"
        link_events.appendleft(line)
        link_log.append(line)
        if hud is None or not hud.enabled:
            print(line, file=sys.stderr, flush=True)

    link = start_headset(args, on_link_event)
    if link is None:
        return 1

    # ---- robot --------------------------------------------------------
    robot = None
    gripper_to_goal = None
    gripper_name = gripper_cmd = None
    live = None

    def fail(msg):
        print(f"\nERROR: {msg}", file=sys.stderr)
        if robot is not None:
            robot.stop()
        link.stop()
        return 1

    if args.dry_run:
        print("DRY RUN -- the robot will not be touched.")
        from quest_teleop.dex_bridge import make_simple_ik
        simple_ik = make_simple_ik()
        wrist_limits = read_wrist_limits_from_params()
    else:
        from quest_teleop.dex_bridge import make_gripper_to_goal
        print("\nThe robot stays where it is; it moves only when you grip "
              "and move the hand.\n", flush=True)
        robot = build_robot(args.direct)
        if robot is None:
            link.stop()
            return 1
        try:
            # Before GripperToGoal, so a failure leaves the arm unmoved.
            gripper_name, gripper_cmd = make_gripper_cmd(robot)
        except Exception as e:
            return fail(f"could not set up the gripper command: {e}")
        # GripperToGoal moves to starting_configuration in its constructor and
        # seeds its position filter from it, so hand it the LIVE pose: a move
        # to where the arm already is. Every failure here leaves it unmoved.
        try:
            robot.pull_status()
            live = read_live_configuration(robot)
            # Near gimbal lock (pitch ~ +/-90 deg) the decode is ambiguous and
            # the first tick would command a different yaw/roll.
            back = decode_wrist(orientation_from_wrist(
                live["wrist_yaw"], live["wrist_pitch"], live["wrist_roll"]))
            err = max(abs(angle_diff_rad(back[i], live[n]))
                      for i, n in enumerate(WRIST_NAMES))
            if err > 0.05:
                raise RuntimeError(
                    f"the wrist pose (yaw {live['wrist_yaw']:+.3f}, pitch "
                    f"{live['wrist_pitch']:+.3f}, roll {live['wrist_roll']:+.3f}) "
                    f"does not survive the orientation round trip (off by "
                    f"{err:.3f} rad, likely near gimbal lock). Move the wrist "
                    f"away from pitch +/-90 deg and retry.")
        except Exception as e:
            return fail(f"could not start from the robot's current pose ({e}). "
                        f"Nothing was moved.")
        live_configuration = {
            'joint_mobile_base_rotate_by': 0.0,
            'lift_joint': live["lift"],
            'arm_l4_joint': live["arm"],
            'wrist_yaw_joint': live["wrist_yaw"],
            'wrist_pitch_joint': live["wrist_pitch"],
            'wrist_roll_joint': live["wrist_roll"],
        }
        try:
            gripper_to_goal = make_gripper_to_goal(
                robot, use_robot_client=not args.direct,
                robot_speed=args.speed,
                starting_configuration=live_configuration,
                robot_allowed_to_move=True)
        except Exception as e:
            return fail(f"could not start the Dex Teleop command path: {e}")
        if gripper_to_goal.gripper_name != gripper_name:
            return fail(f"gripper mismatch: front end {gripper_name!r}, "
                        f"Dex Teleop {gripper_to_goal.gripper_name!r}")
        # Seed the wrist filter from the real wrist, not zeros, or the first
        # tracked tick would blend the wrist halfway toward (0, 0, 0).
        gripper_to_goal.filtered_wrist_orientation = np.array(
            [live["wrist_yaw"], live["wrist_pitch"], live["wrist_roll"]])
        simple_ik = gripper_to_goal.simple_ik
        wrist_limits = gripper_to_goal.wrist_joint_limits
        print(f"Robot ready.  Gripper: {gripper_name}")

    # Where the first grip measures from: the live pose on the robot (base
    # angle 0 -- GripperToGoal just reset odometry), the center config in a
    # dry run.
    if live is not None:
        start_p = np.array(simple_ik.fk_rotary_base({
            'mobile_base_rotation_joint': 0.0, 'lift_joint': live["lift"],
            'arm_l4_joint': live["arm"], 'wrist_yaw_joint': live["wrist_yaw"],
            'wrist_pitch_joint': live["wrist_pitch"],
            'wrist_roll_joint': live["wrist_roll"]}), dtype=float)
        if simple_ik.ik_rotary_base(start_p) is None:
            print("NOTE: the arm's current pose is outside the IK's reach "
                  "(arm past 0.52 m?); the first grip pulls it to the "
                  "nearest reachable pose (a small move).", file=sys.stderr)
    else:
        center = dex_params.get_center_configuration(lift_middle_for(args.ground))
        start_p = np.array(simple_ik.fk_rotary_base(center), dtype=float)
    start_R = np.array(DexCfg.NEUTRAL_GRIPPER_R)
    print(f"start wrist position = {np.round(start_p, 4)} (IK world frame, "
          f"{'live pose' if live is not None else 'center config'})", flush=True)

    pose_target = PoseTarget()
    goal = GoalState(simple_ik)

    # Settings chords need stretch4_body's gamepad enums.
    try:
        chord = ChordTap(speed_name=args.speed)
    except Exception as e:
        chord = None
        print(f"NOTE: trigger + A / B settings chords unavailable ({e}).",
              file=sys.stderr)
    # Strength = guarded-contact sensitivity. The server outlives this script,
    # so it is put back to default on exit (strength_touched).
    strength_touched = False
    strength_shown = chord.strength_label if chord is not None else None
    if chord is not None and robot is not None:
        strength_touched = True
        try:
            robot.set_guarded_contact_sensitivity(chord.strength_name)
        except Exception as e:
            strength_shown = f"{chord.strength_label} (APPLY FAILED)"
            print(f"WARNING: could not set strength {chord.strength_name!r} "
                  f"({e}).", file=sys.stderr)
    js_max_v, js_max_a = DexCfg.JS_MAX_V_MPS, None
    try:
        js_max_v, js_max_a = base_speed_mps(args.speed)
    except Exception as e:
        print(f"NOTE: could not read the base speed for {args.speed!r} ({e}); "
              f"joystick base stays at {js_max_v:.2f} m/s.", file=sys.stderr)

    turned_since_start = [0.0]   # base heading re-zeroed away so far, rad

    def current_pose_for_latch():
        """-> (R, p, heading) anchoring a new grip, so gripping never moves
        the robot.

        Orientation: on the robot, the wrist Dex Teleop is actually commanding
        (its filtered output, after it held any out-of-limit joint), so the
        anchor is always a reachable pose. In a dry run, the last request.
        Position: the last REACHABLE requested goal, else the start pose.
        Heading: re-zeroed to the base's current heading (see
        rezero_base_heading), so 0 unless --no-heading-frame."""
        if gripper_to_goal is not None:
            R = orientation_from_wrist(*gripper_to_goal.filtered_wrist_orientation)
        else:
            R = goal.R_request if goal.R_request is not None else start_R
        p = goal.wrist_position if goal.wrist_position is not None else start_p
        p = np.array(p, dtype=float)
        heading0 = 0.0
        if gripper_to_goal is not None:
            # The base's heading NOW becomes Dex Teleop's 0 rad, so the goal
            # base angle starts near 0 every grip and never crosses +/-180
            # deg (where upstream spins the long way round). Hand forward is
            # then the arm's direction with heading0 = 0.
            theta = rezero_base_heading(gripper_to_goal)
            if theta is None:
                pose_target.log("base heading unavailable -- frame not re-zeroed")
            else:
                p = rotz(-theta) @ p
                goal.wrist_position = p      # the anchor, in the new frame
                turned_since_start[0] += theta
                if args.no_heading_frame:
                    # Keep hand forward = the heading at startup.
                    heading0 = -turned_since_start[0]
        return np.array(R, dtype=float), p, heading0

    log_f = open(args.log_file, "w", newline="") if args.log_file else None
    log_w = csv.writer(log_f) if log_f else None
    if log_w:
        log_w.writerow(LOG_FIELDS)

    hud = DexHud(enabled=not args.no_hud)
    print(DEX_MAPPING_TEXT)
    time.sleep(1.0)

    stop = {"now": False, "hits": 0}

    def on_sigint(_sig, _frm):
        stop["hits"] += 1
        stop["now"] = True
        if stop["hits"] >= 2:
            hud.close()
            print("\nsecond Ctrl-C -- exiting immediately.", file=sys.stderr)
            os._exit(130)

    signal.signal(signal.SIGINT, on_sigint)

    period = 1.0 / max(args.rate, 1.0)
    t0 = time.time()
    fps_hist = deque(maxlen=30)
    prev_tick = t0
    js_active = False       # a non-zero base velocity setpoint is outstanding
    was_runstopped = False

    try:
        while not stop["now"]:
            tick = time.time()
            # poll() owns freshness: the reader hands back its last sample
            # forever, so a dead headset would look like a still hand.
            fresh = link.poll()
            transforms, buttons = link.read()
            inp = HandInput.decode(transforms, buttons) if fresh else HandInput()

            runstopped = False
            if robot is not None:
                # Before anything reads status: RobotClient only refreshes
                # here, and stale odometry spins the base (update_goal also
                # pulls, but the runstop check and latch heading need it now).
                robot.pull_status()
                runstopped = robot.is_runstopped()
                if runstopped and not was_runstopped:
                    pose_target.log("RUNSTOPPED -> gripper/base stopped, clutch cleared")
                    try:
                        gripper_cmd.stop_gripper(robot)
                    except Exception as e:
                        print(f"  warning: gripper stop failed: {e}", file=sys.stderr)
                    try:
                        robot.base.set_velocity(0.0, 0.0, 0.0)
                        robot.push_command()
                    except Exception as e:
                        print(f"  warning: runstop stop failed: {e}", file=sys.stderr)
                    # Releasing the runstop with the grip held re-latches at
                    # the arm's real pose.
                    pose_target.clear()
                    pose_target.engaged = False
                    js_active = False
                elif was_runstopped and not runstopped:
                    pose_target.log("runstop released")
                was_runstopped = runstopped

            # Settings chords (trigger = HandInput.front, same 0.5 threshold).
            chord_events, block_gripper = (
                chord.step(inp.front, inp.a, inp.b, enabled=not runstopped)
                if chord is not None else ([], False))
            for kind, name in chord_events:
                if kind == "speed":
                    try:
                        new_v, new_a = base_speed_mps(name)
                        if gripper_to_goal is not None:
                            set_speed_profile(gripper_to_goal, name)
                        js_max_v, js_max_a = new_v, new_a
                        pose_target.log(f"speed -> {name} (base {js_max_v:.2f} m/s)")
                    except Exception as e:
                        pose_target.log(f"speed change to {name} FAILED: {e}")
                else:
                    try:
                        if robot is not None:
                            strength_touched = True
                            robot.set_guarded_contact_sensitivity(name)
                        strength_shown = chord.strength_label
                        pose_target.log(f"strength -> {chord.strength_label}")
                    except Exception as e:
                        strength_shown = f"{chord.strength_label} (APPLY FAILED)"
                        pose_target.log(f"strength change FAILED: {e}")

            # Gripper: hold A close / B open, release stops. Live in both
            # clutch states; blocked while the trigger is held (chords).
            want = ("hold" if block_gripper
                    else "open" if inp.b else "close" if inp.a else "hold")
            if gripper_cmd is not None and not runstopped:
                will_queue = want != "hold" or gripper_cmd.stop_reqd
                apply_gripper(gripper_cmd, robot, want)
                if will_queue:
                    robot.push_command()

            base_cmd = (0.0, 0.0, 0.0)
            if not runstopped:
                pose_target.step(inp, current_pose_for_latch)

            if inp.inner and pose_target.latched:
                # Grip HELD: Dex Teleop owns the arm, wrist and base ROTATION
                # (rotate_by inside update_goal). The joystick is dead.
                R_t, p_t = pose_target.target(inp.pose)
                goal_kwargs = goal.build(
                    R_t, p_t, wrist_limits,
                    wrist_prev=(gripper_to_goal.filtered_wrist_orientation
                                if gripper_to_goal is not None else None))
                # At a limit the goal is clamped there; also drop the hand's
                # overshoot so moving back responds at once (the hand-to-
                # gripper offset shifts, like a mouse at the screen edge).
                if goal.pinned:
                    pose_target.rebase_position(
                        p_t, goal_kwargs['wrist_position'])
                if goal.wrist_pinned:
                    pose_target.rebase_orientation(
                        R_t, goal.R_request, goal.yaw_shift)
                if gripper_to_goal is not None:
                    # grip_width=None: the gripper is driven by A/B above.
                    gripper_to_goal.update_goal(grip_width=None, **goal_kwargs)
                js_active = False
            else:
                # Grip RELEASED: arm and wrist hold; the joystick drives the
                # base (velocity setpoint, so it is zeroed once on recentre).
                jx, jy = js_unit(inp.js_x), js_unit(inp.js_y)
                vx = jy * js_max_v
                if DexCfg.JS_X_MODE == "turn":
                    vy, w = 0.0, -jx * DexCfg.JS_MAX_W_RADPS
                else:
                    vy, w = -jx * js_max_v, 0.0
                base_cmd = (0.0, 0.0, 0.0) if runstopped else (vx, vy, w)
                if robot is not None and (any(base_cmd) or js_active):
                    if js_max_a is None:
                        robot.base.set_velocity(*base_cmd)
                    else:
                        robot.base.set_velocity(*base_cmd, a_m=js_max_a)
                    robot.push_command()
                js_active = any(base_cmd)

            if args.orient_debug and inp.valid:
                ang = goal.wrist_angles(wrist_limits)
                qw, qx, qy, qz = matrix_to_quat(inp.pose[:3, :3])
                line = (f"[orient] controller quat w={qw:+.3f} x={qx:+.3f} "
                        f"y={qy:+.3f} z={qz:+.3f}")
                if ang is not None:
                    line += (f"   ->  wrist yaw={ang[0]:+.3f} pitch={ang[1]:+.3f} "
                             f"roll={ang[2]:+.3f} rad")
                print(line, file=sys.stderr, flush=True)

            if log_w:
                hand = inp.pose[:3, 3] if inp.valid else (None,) * 3
                hand_q = (matrix_to_quat(inp.pose[:3, :3]) if inp.valid
                          else (None,) * 4)
                if pose_target.latched and inp.valid:
                    dp, _ = pose_target.offset(inp.pose)
                    dist = float(np.linalg.norm(dp))
                else:
                    dp, dist = (None,) * 3, None
                p_goal = (goal.wrist_position if goal.wrist_position is not None
                          else (None,) * 3)
                goal_q = (matrix_to_quat(goal.R_request)
                          if goal.R_request is not None else (None,) * 4)
                c = goal.last_cfg
                solved = ([c['mobile_base_rotation_joint'], c['lift_joint'],
                           c['arm_l4_joint']] if c is not None else [None] * 3)
                ang = goal.wrist_angles(wrist_limits) or (None,) * 3
                log_w.writerow([f"{tick - t0:.3f}", fresh, inp.valid, inp.inner,
                                pose_target.latched, *hand, *hand_q,
                                *dp, dist, *p_goal, *goal_q, goal.unreachable,
                                *solved, *ang,
                                f"{inp.js_x:.3f}", f"{inp.js_y:.3f}", *base_cmd,
                                (gripper_to_goal.base_odometry.get()['theta']
                                 if gripper_to_goal is not None else None)])
                log_f.flush()

            fps_hist.append(1.0 / max(tick - prev_tick, 1e-6))
            prev_tick = tick
            hud.render(inp, pose_target, goal, {
                "fps": sum(fps_hist) / len(fps_hist),
                "fresh": fresh,
                "uptime": tick - t0,
                "dry_run": args.dry_run,
                "link_state": link.state,
                "reconnects": link.reconnects,
                "link_problem": link.problem,
                "runstopped": runstopped,
                "wrist_limits": wrist_limits,
                "gripper_name": gripper_name,
                "gripper_want": want,
                "speed_name": chord.speed_name if chord is not None else args.speed,
                "base_speed_mps": js_max_v,
                "strength_label": strength_shown,
                "chords_ok": chord is not None,
                "js_x_mode": DexCfg.JS_X_MODE,
            }, list(pose_target.events)[:2] + list(link_events)[:2])

            time.sleep(max(0.0, period - (time.time() - tick)))
    finally:
        hud.close()
        if hud.enabled and link_log:
            print("Headset link events this session:")
            for line in link_log[-20:]:
                print(f"  {line}")
        print("\nStopping ...")
        if log_f:
            log_f.close()
            print(f"  wrote log to {args.log_file}")
        # Each in its own try, so one failure can't skip the base zeroing and
        # robot.stop() that follow.
        if gripper_cmd is not None:
            try:
                gripper_cmd.stop_gripper(robot)
            except Exception as e:
                print(f"  warning: gripper stop failed: {e}", file=sys.stderr)
        if robot is not None and strength_touched:
            try:
                robot.set_guarded_contact_sensitivity("default")
            except Exception as e:
                print(f"  warning: could not restore guarded-contact "
                      f"sensitivity: {e}", file=sys.stderr)
        try:
            if robot is not None:
                # set_velocity persists until replaced; rotate_by is one-shot.
                robot.base.set_velocity(0.0, 0.0, 0.0)
                robot.push_command()
                robot.stop()
        except Exception as e:
            print(f"  warning: robot.stop failed: {e}", file=sys.stderr)
        try:
            if args.stop_app_on_exit:
                link.force_stop_app()
            link.stop()
        except Exception as e:
            print(f"  warning: link.stop failed: {e}", file=sys.stderr)
        if link.reconnects:
            print(f"  ({link.reconnects} headset reconnect attempt(s))")
        print("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
