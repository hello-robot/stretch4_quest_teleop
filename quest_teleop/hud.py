"""Terminal dashboard and the control-map text."""

import sys
import textwrap

from .mapping import clamp


CSI = "\033["
BOLD, DIM, RESET = CSI + "1m", CSI + "2m", CSI + "0m"
RED, GRN, YEL, MAG = (CSI + "31m", CSI + "32m", CSI + "33m", CSI + "35m")


def bar(v, width=17):
    half = width // 2
    v = clamp(v, -1.0, 1.0)
    n = int(round(abs(v) * half))
    if v >= 0:
        return " " * half + "|" + "#" * n + " " * (half - n)
    return " " * (half - n) + "#" * n + "|" + " " * half


class Hud:
    def __init__(self, enabled=True):
        self.enabled = enabled and sys.stdout.isatty()
        self.plain = enabled and not sys.stdout.isatty()
        self._open = False

    def close(self):
        """Leave the dashboard screen and give the terminal back as it was."""
        if self._open:
            sys.stdout.write(CSI + "?25h" + CSI + "?1049l")
            sys.stdout.flush()
            self._open = False

    def _c(self, s, col):
        return f"{col}{s}{RESET}" if self.enabled else s

    def render(self, d, inp, mapping, stats):
        if not (self.enabled or self.plain):
            return
        L = []
        A = L.append

        st = stats.get("link_state", "live")
        if stats["fresh"]:
            link = self._c(f"{stats['fps']:5.1f} Hz", GRN)
        elif st == "stale":
            link = self._c("  STALE  ", YEL)
        else:
            link = self._c("  DOWN   ", RED)
        mode = self._c("DRY-RUN", YEL) if stats["dry_run"] else self._c("LIVE", GRN)
        rc = stats.get("reconnects", 0)
        rctxt = self._c(f"  reconn {rc}", YEL) if rc else ""
        A(f" stretch_quest_teleop   link {link}   {mode}   "
          f"{stats['uptime']:6.1f}s{rctxt}")
        A("-" * 68)

        m6 = self._c("[ 6D ]", GRN) if d.mode_6d else self._c("[ -- ]", DIM)
        mw = self._c("[ WRIST ]", MAG) if d.mode_wrist else self._c("[ ----- ]", DIM)
        A(f" inner {m6}   front {mw}   "
          f"gripper {self._c(d.gripper.upper(), YEL)}")
        A(self._c(" *** ROBOT RUNSTOPPED -- CANNOT MOVE ***", RED)
          if stats.get("runstopped") else "")

        problem = stats.get("link_problem") or []
        if problem:
            A(self._c(" HEADSET LINK DOWN -- what to check:", RED))
            for line in problem:
                indent = " " * (len(line) - len(line.lstrip()) + 3)
                for w in textwrap.wrap(line.strip(), 68,
                                       initial_indent=indent,
                                       subsequent_indent=indent + "  "):
                    A(self._c(w, YEL))
            A("")

        A(self._c(" INPUT", BOLD))
        A(f"   joystick  x {inp.js_x:+.2f}  y {inp.js_y:+.2f}"
          f"    inner {'ON ' if inp.inner else 'off'}"
          f"  front {'ON ' if inp.front else 'off'}"
          f"  A {'ON ' if inp.a else 'off'}  B {'ON' if inp.b else 'off'}")
        if d.mode_6d or d.mode_wrist:
            A(f"   hand dUP {d.raw_d_up*100:+7.1f} cm"
              f"  dFWD {d.raw_d_fwd*100:+7.1f} cm"
              f"  aimYAW {d.raw_yaw_deg:+7.1f}"
              f"  aimPITCH {d.raw_pitch_deg:+7.1f} deg")
        else:
            A(self._c("   no clutch held -- hand offsets not tracked", DIM))
        A("")

        A(self._c(" COMMANDING", BOLD))
        for name, val, pos, neg in (
                ("BASE fwd/back", d.base_x, "forward", "backward"),
                ("BASE strafe  ", d.base_y, "left", "right"),
                ("BASE turn    ", d.base_w, "ccw", "cw"),
                ("LIFT         ", d.lift, "UP", "DOWN"),
                ("ARM          ", d.arm, "EXTEND", "RETRACT"),
                ("WRIST yaw    ", d.wrist_yaw, "ccw", "cw"),
                ("WRIST pitch  ", d.wrist_pitch, "down", "up"),
                ("WRIST roll   ", d.wrist_roll, "cw", "ccw")):
            verb = self._c("idle", DIM) if val == 0.0 \
                else self._c(pos if val > 0 else neg, YEL)
            A(f"   {name} {val:+.2f} [{bar(val)}]  {verb}")
        A("")

        A(self._c(" EVENTS", BOLD))
        evs = (list(stats.get("link_events", []))[:2]
               + list(mapping.events))[:4]
        for e in evs:
            A(f"   {e}")
        for _ in range(4 - len(evs)):
            A("")
        for r in d.reasons[:1]:
            A(self._c(f"   note: {r}", DIM))
        for _ in range(1 - len(d.reasons[:1])):
            A("")
        A("-" * 68)
        A(self._c(" Ctrl-C to stop.  Releasing both buttons stops 6D/wrist.",
                  DIM))

        if self.plain:
            print(" | ".join(x.strip() for x in L if x.strip()), flush=True)
            return
        out = "\n".join(x + CSI + "K" for x in L)
        if not self._open:
            # Alternate screen: the dashboard never mixes with scrollback.
            sys.stdout.write(CSI + "?1049h" + CSI + "?25l" + CSI + "2J")
            self._open = True
        sys.stdout.write(CSI + "H" + out + "\n" + CSI + "J")
        sys.stdout.flush()


MAPPING_TEXT = """
RIGHT Quest controller -> Stretch 4
===================================

ALWAYS LIVE
  A ......................... gripper CLOSE
  B ......................... gripper OPEN
  HOLD to move, release to stop. Both pressed = open.
  Everything stops if the robot is runstopped.

JOYSTICK  (drives the base -- unless the FRONT button is held)
  Joystick  forward/back .... base drives forward / backward
  Joystick  left/right ...... base strafes left / right
                              (--js-x turn to turn in place instead)

HOLD THE INNER BUTTON (side grip)  ->  6D MODE
  Raise / lower hand ........ LIFT up / down
  Push / pull hand .......... ARM extend / retract
  Rotate hand left / right .. BASE turns

HOLD THE FRONT BUTTON (index trigger)  ->  WRIST MODE
  Rotate hand left / right .. WRIST YAW
  Rotate hand up / down ..... WRIST PITCH
  Joystick left / right ..... WRIST ROLL
  Joystick forward/back ..... nothing.  Base driving is DISABLED in wrist
                              mode so the robot cannot wander off while you
                              are aiming the gripper.

  The front button WINS the rotation channel: while it is held, hand
  rotation drives the wrist whether or not the inner button is also down.

  Hold BOTH and the modes stack -- lift/arm from hand translation and
  wrist yaw/pitch from hand rotation, at the same time.

  Releasing both stops all 6D/wrist motion and clears the origin.
"""
