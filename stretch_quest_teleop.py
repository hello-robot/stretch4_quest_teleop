#!/usr/bin/env python3
"""
Teleoperate a Stretch 4 from the right Meta Quest controller.

    python3 stretch_quest_teleop.py --dry-run     # no robot, learn the mapping
    python3 stretch_quest_teleop.py               # drive the robot
    python3 stretch_quest_teleop.py --help-map    # print the control mapping
"""

import argparse
import os
import signal
import sys
import time
from collections import deque

from quest_teleop.config import Cfg
from quest_teleop.headset_link import HeadsetLink
from quest_teleop.hud import Hud, MAPPING_TEXT
from quest_teleop.mapping import HandInput, Mapping
from quest_teleop.robot_commander import RobotCommander


def build_robot(use_server):
    if use_server:
        from stretch4_body.robot.robot_client import RobotClient
        return RobotClient(client_id="stretch_quest_teleop"), True
    from stretch4_body.robot.robot import Robot
    return Robot(), False


def main():
    ap = argparse.ArgumentParser(
        description="Teleoperate Stretch 4 from an Oculus/Quest right controller.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=MAPPING_TEXT)
    ap.add_argument("--dry-run", action="store_true",
                    help="read the controller and show the HUD, never touch "
                         "the robot")
    ap.add_argument("-d", "--direct", action="store_true",
                    help="use the direct stretch4_body API, not the server")
    ap.add_argument("--no-hud", action="store_true",
                    help="disable the terminal dashboard")
    ap.add_argument("--js-x", choices=["strafe", "turn"], default=Cfg.JS_X_MODE,
                    help="what joystick left/right does (strafe or turn); "
                         "in wrist mode the joystick rolls the wrist instead")
    ap.add_argument("--rate", type=float, default=Cfg.RATE_HZ,
                    help="control loop rate in Hz")
    ap.add_argument("--speed", type=float, default=None,
                    help="global speed scale, e.g. 0.5 for half speed")
    ap.add_argument("--ip", default=None, help="Quest IP for wireless ADB")
    ap.add_argument("--no-restart-app", dest="restart_app",
                    action="store_false", default=True,
                    help="reuse the headset app instance left by a previous "
                         "run instead of force-stopping and relaunching it "
                         "(default is to restart, which is more reliable)")
    ap.add_argument("--link-timeout", type=float, default=5.0,
                    help="seconds to wait for the first pose before "
                         "printing what's wrong; it keeps waiting after that "
                         "(default 5)")
    ap.add_argument("--stop-app-on-exit", action="store_true",
                    help="force-stop the headset app when quitting")
    ap.add_argument("--log", default=None, metavar="FILE",
                    help="append a line per tick to FILE while a button or "
                         "the joystick is active: modes, hand offsets, "
                         "commands")
    ap.add_argument("--help-map", action="store_true",
                    help="print the control mapping and exit")
    args = ap.parse_args()

    if args.help_map:
        print(MAPPING_TEXT)
        return 0
    if args.speed is not None:
        Cfg.SPEED_SCALE = float(args.speed)

    # ---- headset ----------------------------------------------------
    try:
        import oculus_reader.reader  # noqa: F401  (import check only)
    except ImportError as e:
        print(f"ERROR: cannot import oculus_reader ({e}).\n"
              f"       Install it into this interpreter (clone it outside this\n"
              f"       repo) by following the Install steps in README.md",
              file=sys.stderr)
        return 1

    print("Connecting to the Quest over ADB ...", flush=True)
    link_events = deque(maxlen=6)
    link_log = []           # every link event, printed again on exit
    hud = None

    def on_link_event(msg):
        line = f"{time.strftime('%H:%M:%S')}  LINK: {msg}"
        link_events.appendleft(line)
        link_log.append(line)
        # Printing while the dashboard is up would scribble over it.
        if hud is None or not hud.enabled:
            print(line, file=sys.stderr, flush=True)

    try:
        link = HeadsetLink(ip_address=args.ip, on_event=on_link_event)
    except (Exception, SystemExit) as e:
        # oculus_reader calls exit(1) from inside get_usb_device(), which is
        # a SystemExit and therefore not an Exception
        print(f"\nERROR: could not reach the headset over ADB ({e}).\n"
              f"       Check `adb devices` shows it and that you accepted "
              f"the USB-debugging prompt inside the headset.", file=sys.stderr)
        return 1

    if args.restart_app:
        print("Restarting the headset app for a clean start ...", flush=True)
    print("Waiting for the first controller pose ...", flush=True)
    if not link.start(restart_app=args.restart_app, wait_s=args.link_timeout):
        # Don't give up: everything that stops poses is fixable on the
        # headset, so say what's wrong and carry on once it's fixed.
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
            return 1
    print("Headset link up -- poses flowing.", flush=True)

    # ---- robot ------------------------------------------------------
    robot = None
    commander = None
    if not args.dry_run:
        robot, use_server = build_robot(use_server=not args.direct)
        print(f"Starting robot ({'server' if use_server else 'direct'}) ...")
        if not robot.startup():
            print("ERROR: robot.startup() failed.", file=sys.stderr)
            return 1
        if robot.is_runstopped():
            # Not fatal: the runstop can be released while running.
            print("\nNOTE: the robot is RUNSTOPPED -- nothing will move until "
                  "you twist the runstop button on the head to release it.",
                  file=sys.stderr)
        if not robot.is_homed():
            print("\nERROR: the robot is not homed.  Run "
                  "`stretch_robot_home.py` first.", file=sys.stderr)
            robot.stop()
            return 1
        try:
            commander = RobotCommander(robot, use_server)
        except NotImplementedError as e:
            print(f"ERROR: {e}", file=sys.stderr)
            robot.stop()
            link.stop()
            return 1
        print(f"Robot ready.  Gripper: {commander.gripper_name}"
              f"   dex wrist: {commander.has_dex_wrist}")
    else:
        print("DRY RUN -- the robot will not be touched.")

    mapping = Mapping(js_x_mode=args.js_x)
    hud = Hud(enabled=not args.no_hud)
    print(MAPPING_TEXT)
    time.sleep(1.0)

    stop = {"now": False, "hits": 0}

    def on_sigint(_sig, _frm):
        stop["hits"] += 1
        stop["now"] = True
        if stop["hits"] >= 2:
            # Escape hatch: never leave the user stuck holding Ctrl-C because
            # shutdown wedged on the robot or the headset.
            hud.close()
            print("\nsecond Ctrl-C -- exiting immediately.", file=sys.stderr)
            os._exit(130)

    signal.signal(signal.SIGINT, on_sigint)

    log = open(args.log, "a", buffering=1) if args.log else None
    period = 1.0 / max(args.rate, 1.0)
    t0 = time.time()
    fps_hist = deque(maxlen=30)
    prev_tick = t0
    was_runstopped = False

    try:
        while not stop["now"]:
            tick = time.time()
            # poll() owns freshness and reconnection: the reader hands back its
            # last sample forever, so a dead headset otherwise looks like a
            # live one holding perfectly still.
            fresh = link.poll()
            transforms, buttons = link.read()
            inp = HandInput.decode(transforms, buttons) if fresh else HandInput()

            d = mapping.step(inp)

            runstopped = False
            if commander is not None:
                if hasattr(robot, "pull_status"):
                    robot.pull_status()
                runstopped = robot.is_runstopped()
                if runstopped:
                    d.reasons.append("RUNSTOPPED -- robot cannot move")
                    if not was_runstopped:
                        mapping.log_event("RUNSTOPPED -> all motion stopped")
                        try:
                            commander.stop_all()
                        except Exception as e:
                            print(f"  warning: stop_all failed: {e}",
                                  file=sys.stderr)
                elif was_runstopped:
                    mapping.log_event("runstop released")
                was_runstopped = runstopped

            fps_hist.append(1.0 / max(tick - prev_tick, 1e-6))
            prev_tick = tick
            hud.render(d, inp, mapping, {
                "fps": sum(fps_hist) / len(fps_hist),
                "fresh": fresh, "dry_run": args.dry_run,
                "uptime": tick - t0,
                "link_state": link.state,
                "reconnects": link.reconnects,
                "runstopped": runstopped,
                "link_events": link_events,
                "link_problem": link.problem})

            if commander is not None and not runstopped:
                commander.apply(d)

            if log and (d.mode_6d or d.mode_wrist or d.gripper != "hold"
                        or d.base_x or d.base_y or d.base_w):
                log.write(
                    f"{time.strftime('%H:%M:%S')}.{int(tick * 1000) % 1000:03d}"
                    f" {'6D' if d.mode_6d else '--'}"
                    f" {'WR' if d.mode_wrist else '--'}"
                    f" | hand up {d.raw_d_up*100:+6.1f}cm"
                    f" fwd {d.raw_d_fwd*100:+6.1f}cm"
                    f" yaw {d.raw_yaw_deg:+6.1f} pitch {d.raw_pitch_deg:+6.1f}"
                    f" | base x {d.base_x:+.2f} y {d.base_y:+.2f}"
                    f" w {d.base_w:+.2f} lift {d.lift:+.2f} arm {d.arm:+.2f}"
                    f" wyaw {d.wrist_yaw:+.2f} wpitch {d.wrist_pitch:+.2f}"
                    f" wroll {d.wrist_roll:+.2f} grip {d.gripper}"
                    f"{' RUNSTOPPED' if runstopped else ''}\n")

            time.sleep(max(0.0, period - (time.time() - tick)))
    finally:
        if log:
            log.close()
        hud.close()
        if hud.enabled and link_log:
            print("Headset link events this session:")
            for line in link_log[-20:]:
                print(f"  {line}")
        print("\nStopping ...")
        try:
            if commander is not None:
                commander.stop_all()
        except Exception as e:
            print(f"  warning: stop_all failed: {e}", file=sys.stderr)
        try:
            if robot is not None:
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
            print(f"  ({link.reconnects} headset reconnect attempt(s) "
                  f"during this session)")
        print("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
