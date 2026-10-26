"""Connection to the Quest over adb/oculus_reader, with diagnosis."""

import os
import socket
import sys
import threading
import time


class HeadsetLink:
    """Owns the logcat pump, reports link state, and always shuts down. """

    LIVE, STALE, DOWN = "live", "stale", "down"
    APP = "com.rail.oculus.teleop"
    FORK_URL = "https://github.com/jborbik/oculus_reader"

    def __init__(self, ip_address=None, stale_after=1.0, reconnect_after=3.0,
                 on_event=None):
        from oculus_reader.reader import OculusReader
        self._OculusReader = OculusReader
        # run=False: we start the pump ourselves, as a daemon.
        self._reader = OculusReader(ip_address=ip_address, run=False)
        self.device = self._reader.device
        self.stale_after = float(stale_after)
        self.reconnect_after = float(reconnect_after)
        self.on_event = on_event or (lambda msg: None)

        self._lock = threading.Lock()
        self._transforms, self._buttons = {}, {}
        self._conn = None
        self._running = False
        self._thread = None
        self._last_data = 0.0
        self.state = self.DOWN
        self.reconnects = 0
        self.problem = []       # diagnose() lines while the link is down
        self._last_retry = 0.0
        self._last_diag = 0.0

    # -- lifecycle -------------------------------------------------------
    def start(self, restart_app=True, wait_s=5.0):
        """Bring the link up and WAIT for the first real pose.

        Returns True once poses are arriving.

        ``restart_app`` force-stops the app first, so each run starts clean.
        """
        self._running = True
        self._last_data = time.time()
        if restart_app:
            self.force_stop_app()
        self.launch_app()
        self._thread = threading.Thread(target=self._pump, daemon=True)
        self._thread.start()
        return self.wait_for_pose(wait_s)

    def wait_for_pose(self, wait_s):
        """True as soon as a right-controller pose arrives, False after
        wait_s seconds without one."""
        deadline = time.time() + wait_s
        while time.time() < deadline:
            with self._lock:
                got = "r" in (self._transforms or {})
            if got:
                self.state = self.LIVE
                self._last_data = time.time()
                return True
            time.sleep(0.2)
        return False

    def launch_app(self):
        try:
            self.device.shell(
                'am start -n "com.rail.oculus.teleop/'
                'com.rail.oculus.teleop.MainActivity" '
                '-a android.intent.action.MAIN -c android.intent.category.LAUNCHER')
            return True
        except Exception as e:
            self.on_event(f"could not launch the headset app: {e}")
            return False

    def bring_app_to_front(self):
        """Launch the app if it's closed or behind something.  Never relaunches
        an app that's already in front (that makes it crash), and does nothing
        while the headset is asleep.  Returns True if it launched."""
        try:
            wake = self.device.shell("dumpsys power | grep -m1 mWakefulness=")
            if "Awake" not in (wake or ""):
                return False
            act = self.device.shell(
                "dumpsys activity activities | grep -m1 mFocusedApp=") or ""
        except Exception:
            return False
        if self.APP in act:
            return False
        self.reconnects += 1
        return self.launch_app()

    def force_stop_app(self):
        try:
            self.device.shell("am force-stop com.rail.oculus.teleop")
            time.sleep(0.4)
            return True
        except Exception as e:
            self.on_event(f"could not force-stop the headset app: {e}")
            return False

    # -- diagnosis -------------------------------------------------------
    def diagnose(self):
        """Why no pose is arriving, as printable lines: one "problem" line per
        failed check, each followed by "->" lines saying what to do.  Empty list means every
        check passed.  Side effect: relaunches the app if it isn't running."""
        out = []

        def problem(msg, *fixes):
            out.append(f"- {msg}")
            out.extend(f"    -> {f}" for f in fixes)

        try:
            wake = (self.device.shell(
                "dumpsys power | grep -m1 mWakefulness=") or "").strip()
        except Exception as e:
            problem(f"Can't reach the headset over USB ({e}).",
                    "Replug the USB cable.",
                    "Put the headset on and accept the USB debugging prompt.")
            return out      # every other check needs adb
        if "Asleep" in wake or "Dozing" in wake:
            problem("The headset is asleep.",
                    "Press the power button once to wake it.",
                    "To stop it sleeping when not worn, tape over the sensor "
                    "between the lenses.")

        out.extend(self._check_apk())

        try:
            pid = (self.device.shell(f"pidof {self.APP}") or "").strip()
        except Exception:
            pid = "?"
        if not pid:
            self.launch_app()
            problem("The headset app had closed -- reopened it.",
                    "If this keeps happening, put the headset on and look "
                    "for an error message.")
        else:
            try:
                act = self.device.shell(
                    "dumpsys activity activities | grep -m1 mFocusedApp=") or ""
            except Exception:
                act = self.APP
            if self.APP not in act:
                problem("The headset app is behind another screen -- "
                        "bringing it back.",
                        "If it stays hidden, put the headset on and close any "
                        "menu or pop-up until you see the green-and-blue "
                        "tiles.",
                        "Avoid the Meta button while driving.")

        with self._lock:
            keys = sorted((self._transforms or {}).keys())
        if keys and "r" not in keys:
            problem("The headset only sees the LEFT controller.",
                    "Pick up the RIGHT controller, hold it where the headset "
                    "can see it, and press any button on it.")
        elif not keys and not out:
            problem("The headset can't see either controller.",
                    "Pick up BOTH controllers, hold them in front of the "
                    "headset, and press any button on each.",
                    "Check the controller batteries.",
                    "Turn on more lights if the room is dark.",
                    "Still nothing? Reboot the headset: `adb reboot`.")
        return out

    def _check_apk(self):
        """Problems with the headset app itself, as diagnose() lines.  The
        local checks can't change while we run, so they're done once."""
        out = []
        if not hasattr(self, "_local_apk_problems"):
            self._local_apk_problems = []
            pkg = os.path.dirname(os.path.realpath(
                sys.modules[type(self._reader).__module__].__file__))
            apk = os.path.join(pkg, "APK", "teleop-debug.apk")
            self._local_apk_size = (os.path.getsize(apk)
                                    if os.path.exists(apk) else None)
            if self._local_apk_size is not None and self._local_apk_size < 1e6:
                self._local_apk_problems += [
                    f"- {apk} is a Git LFS placeholder "
                    f"({self._local_apk_size} bytes), not the real app.",
                    f"    -> `git lfs install && git lfs pull` in "
                    f"{os.path.dirname(pkg)}"]
                self._local_apk_size = None
            git_cfg = os.path.join(os.path.dirname(pkg), ".git", "config")
            if (os.path.exists(git_cfg)
                    and "jborbik" not in open(git_cfg).read()):
                self._local_apk_problems += [
                    f"- oculus_reader at {os.path.dirname(pkg)} isn't the "
                    f"jborbik fork, whose app has Quest 3 support.",
                    f"    -> Reinstall from {self.FORK_URL} (see README "
                    f"Install), then replace the headset app."]
        out.extend(self._local_apk_problems)

        try:
            path = (self.device.shell(f"pm path {self.APP}") or "").strip()
        except Exception:
            return out
        if not path:
            out += ["- the headset app isn't installed.",
                    "    -> `python3 -m oculus_reader.install`"]
            return out
        try:
            size = int(self.device.shell(
                f"stat -c %s {path.split(':', 1)[1].splitlines()[0]}").strip())
        except Exception:
            return out
        if self._local_apk_size and size != self._local_apk_size:
            out += [f"- the headset app ({size} bytes) isn't the one in your "
                    f"oculus_reader ({self._local_apk_size} bytes); it's "
                    f"probably an older version.",
                    f"    -> `adb uninstall {self.APP} && "
                    f"python3 -m oculus_reader.install`"]
        return out

    def stop(self):
        """Always returns.  Closes the socket to break the blocked read."""
        self._running = False
        conn = self._conn
        self._conn = None
        if conn is not None:
            for closer in (lambda: conn.socket.shutdown(socket.SHUT_RDWR),
                           lambda: conn.socket.close(),
                           lambda: conn.close()):
                try:
                    closer()
                except Exception:
                    pass
        if self._thread is not None:
            # Bounded: the thread is a daemon, so even a wedged one cannot
            # keep the interpreter alive past this point.
            self._thread.join(timeout=1.5)

    # -- pump ------------------------------------------------------------
    def _pump(self):
        while self._running:
            try:
                self.device.shell("logcat -T 0", self._handler)
            except Exception as e:
                if self._running:
                    self.on_event(f"logcat connection dropped: {e}")
            if not self._running:
                break
            time.sleep(0.5)     # connection died; the supervisor loop retries

    def _handler(self, connection):
        self._conn = connection
        try:
            f = connection.socket.makefile()
        except Exception:
            return
        while self._running:
            try:
                line = f.readline()
            except (OSError, ValueError, UnicodeDecodeError):
                break
            if not line:
                break           # EOF: adb closed the stream
            try:
                data = self._reader.extract_data(line.strip())
                if not data:
                    continue
                transforms, buttons = self._OculusReader.process_data(data)
            except Exception:
                continue
            if transforms is None:
                continue
            with self._lock:
                self._transforms, self._buttons = transforms, buttons
                if "r" in transforms:
                    self._last_data = time.time()
        try:
            f.close()
        except Exception:
            pass
        try:
            connection.close()
        except Exception:
            pass

    # -- polling ---------------------------------------------------------
    def read(self):
        """-> (transforms, buttons).  Never blocks."""
        with self._lock:
            return self._transforms, self._buttons

    def poll(self):
        """Update link state and reconnect if the feed has gone quiet.

        Returns True while data is flowing.  Called once per control tick.
        """
        age = time.time() - self._last_data
        prev = self.state

        if age < self.stale_after:
            self.state = self.LIVE
        elif age < self.reconnect_after:
            self.state = self.STALE
        else:
            self.state = self.DOWN

        if self.state != prev:
            if self.state == self.LIVE:
                self.problem = []
                self.on_event("headset link RECOVERED -- poses flowing again"
                              if prev != self.DOWN or self.reconnects
                              else "headset link up")
            elif self.state == self.STALE:
                self.on_event(f"headset link STALE -- no pose for {age:.1f}s "
                              f"(robot held at zero)")
            else:
                self._last_diag = 0.0
                self.on_event(f"headset link DOWN after {age:.1f}s with no "
                              "pose (robot held at zero). Retrying...")

        # While down, keep trying to bring the app back to the foreground,
        # and keep the diagnosis current.
        if self.state == self.DOWN:
            now = time.time()
            if now - self._last_diag > 10.0:
                self._last_diag = now
                self.problem = self.diagnose() or [
                    "- No controller data yet.",
                    "    -> Press any button on both controllers."]
            if now - self._last_retry > 5.0:
                self._last_retry = now
                self.bring_app_to_front()
        return self.state == self.LIVE
