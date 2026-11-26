"""Terminal dashboard and control-map text for stretch_quest_dex_teleop.py."""

import sys
import textwrap

import numpy as np

from .dex_frontend import matrix_to_quat
from .mapping import rotvec_from_matrix

CSI = "\033["
BOLD, DIM, RESET = CSI + "1m", CSI + "2m", CSI + "0m"
RED, GRN, YEL = CSI + "31m", CSI + "32m", CSI + "33m"
WIDTH = 74


class DexHud:
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

    def render(self, inp, clutch, goal, stats, events):
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
        A(f" stretch_quest_dex_teleop   link {link}   {mode}   "
          f"{stats['uptime']:6.1f}s{rctxt}")
        A("-" * WIDTH)
        A(self._c(" *** ROBOT RUNSTOPPED -- CANNOT MOVE ***", RED)
          if stats.get("runstopped") else "")

        problem = stats.get("link_problem") or []
        if problem:
            A(self._c(" HEADSET LINK DOWN -- what to check:", RED))
            for line in problem:
                indent = " " * (len(line) - len(line.lstrip()) + 3)
                for w in textwrap.wrap(line.strip(), WIDTH,
                                       initial_indent=indent,
                                       subsequent_indent=indent + "  "):
                    A(self._c(w, YEL))
            A("")
        elif not inp.valid:
            A(self._c(" POSE  --  NO POSE (controller not tracked)", RED))
            A("")

        A(self._c(" CLUTCH", BOLD))
        if clutch.latched:
            A(f"   state  {self._c('LATCHED', GRN)}")
            if inp.valid:
                dp, R_rel = clutch.offset(inp.pose)
                rot = np.degrees(np.linalg.norm(rotvec_from_matrix(R_rel)))
                A(f"   hand since latch    dx={dp[0]:+.3f} dy={dp[1]:+.3f} "
                  f"dz={dp[2]:+.3f} m   rot={rot:5.1f} deg")
        else:
            A(f"   state  {self._c('not latched', DIM)}")
            A(self._c("   (hold the side grip to track the hand)", DIM))
        A("")

        A(self._c(" GOAL  (IK world frame: +x left, +y back, +z up)", BOLD))
        if goal.R_request is None:
            A(self._c("   (nothing sent yet)", DIM))
            A("")
        else:
            p = goal.wrist_position
            A(f"   wrist_position  x={p[0]:+.3f}  y={p[1]:+.3f}  z={p[2]:+.3f} m"
              if p is not None else self._c("   wrist_position  (none reachable yet)", DIM))
            qw, qx, qy, qz = matrix_to_quat(goal.R_request)
            A(f"   orientation     w={qw:+.3f} x={qx:+.3f} y={qy:+.3f} z={qz:+.3f}")
        A("")

        A(self._c(" POSITION IK  -> base rotation, lift, arm", BOLD))
        if goal.pinned:
            A(f"   status  {self._c('AT REACH LIMIT: ' + ', '.join(goal.pinned) + ' -- clamped', YEL)}")
        elif goal.last_cfg is None:
            A(f"   status  {self._c('no solve yet', DIM)}")
        else:
            A(f"   status  {self._c('ok', GRN)}")
        if goal.last_cfg is not None:
            c = goal.last_cfg
            A(f"   solved  base_rot={c['mobile_base_rotation_joint']:+.3f}  "
              f"lift={c['lift_joint']:.3f}  arm={c['arm_l4_joint']:.3f}")
        else:
            A("")
        A("")

        A(self._c(" WRIST  -> commanded directly, no IK", BOLD))
        lim = stats.get("wrist_limits")
        ang = goal.wrist_angles(lim)
        if ang is None:
            A(self._c("   (nothing sent yet)", DIM))
            A("")
        else:
            A(f"   yaw={ang[0]:+.3f}  pitch={ang[1]:+.3f}  roll={ang[2]:+.3f} rad")
            bad = goal.wrist_pinned
            A(f"   {self._c('AT LIMIT: ' + ', '.join(bad) + ' -- clamped', YEL)}"
              if bad else "")
        A("")

        A(self._c(" GRIPPER / MODE", BOLD))
        A(f"   {stats.get('gripper_name') or '(no gripper)'}   "
          f"hold A close / B open:  "
          f"{self._c(stats.get('gripper_want', 'hold').upper(), YEL)}")
        bs = stats.get("base_speed_mps")
        A(f"   speed {self._c(str(stats.get('speed_name') or '--'), YEL)}"
          f"{f' (base {bs:.2f} m/s)' if bs is not None else ''}"
          f"   strength {self._c(str(stats.get('strength_label') or '--'), YEL)}"
          f"   {self._c('(trigger+A speed, trigger+B strength)' if stats.get('chords_ok', True) else '(settings chords unavailable)', DIM)}")
        A("")

        A(self._c(" INPUT", BOLD))
        A(f"   grip {'HELD' if inp.inner else 'off '}   trigger "
          f"{'ON ' if inp.front else 'off'}   A {'ON ' if inp.a else 'off'}"
          f"   B {'ON ' if inp.b else 'off'}")
        js = self._c("dead (grip held)", DIM) if inp.inner else self._c(
            f"live -> base {stats.get('js_x_mode', 'strafe')}", GRN)
        A(f"   joystick ({inp.js_x:+.2f}, {inp.js_y:+.2f})   {js}")
        A("")

        A(self._c(" EVENTS", BOLD))
        evs = list(events)[:4]
        for e in evs:
            A(f"   {e}")
        for _ in range(4 - len(evs)):
            A("")
        A("-" * WIDTH)
        A(self._c(" hold grip to track  |  A close, B open  |  Ctrl-C to stop",
                  DIM))

        if self.plain:
            print(" | ".join(x.strip() for x in L if x.strip()), flush=True)
            return
        out = "\n".join(x + CSI + "K" for x in L)
        if not self._open:
            sys.stdout.write(CSI + "?1049h" + CSI + "?25l" + CSI + "2J")
            self._open = True
        sys.stdout.write(CSI + "H" + out + "\n" + CSI + "J")
        sys.stdout.flush()


DEX_MAPPING_TEXT = """
RIGHT Quest controller -> Stretch 4, Dex Teleop control scheme
==============================================================

HOLD THE SIDE GRIP  ->  TRACK THE HAND (clutch)
  Move hand ................. gripper follows 1:1
                                up/down -> lift, forward/back -> arm,
                                sideways -> the base TURNS to reach
  Rotate hand ............... wrist yaw / pitch / roll follow 1:1
  Release ................... arm and wrist hold; re-grip continues from
                              there (re-grip to reach further, like a mouse)
  Joystick .................. dead while gripping

RELEASE THE GRIP
  Joystick forward/back ..... base drives forward / backward
  Joystick left/right ....... base strafes (--js-x turn: turns in place)

ALWAYS LIVE
  A ......................... gripper CLOSE (hold)
  B ......................... gripper OPEN  (hold; wins if both)
  Trigger + tap A ........... speed:    slow -> default -> fast
  Trigger + tap B ........... strength: Medium -> High -> Low
                              (collision sensitivity of lift/arm/base)
  While the trigger is held, A/B never move the gripper.
"""
