"""Sends a Decision to the robot through stretch4_body's gamepad commands."""

import sys

from .config import Cfg
from .mapping import Decision


class RobotCommander:
    """Applies a Decision to the robot."""

    def __init__(self, robot, use_server):
        from stretch4_body.core import gamepad_joints
        from stretch4_body.utils.stretch_pose_models import RobotJoints

        self.robot = robot
        self.use_server = use_server

        mp = Cfg.MOTION_PROFILE
        self.base = gamepad_joints.CommandBase(
            motion_profile=mp, motion_profile_angular=Cfg.MOTION_PROFILE_ANGULAR)
        self.lift = gamepad_joints.CommandLift(motion_profile=mp)
        self.arm = gamepad_joints.CommandArm(motion_profile=mp)

        wmp, wdx = Cfg.WRIST_MOTION_PROFILE, Cfg.WRIST_DX_DEG
        self.wrist_yaw = gamepad_joints.CommandWristYaw(
            dx_deg=wdx, motion_profile=wmp)
        joints = {j.value for j in RobotJoints}
        self.has_dex_wrist = {"wrist_pitch", "wrist_roll"} <= joints
        if self.has_dex_wrist:
            self.wrist_pitch = gamepad_joints.CommandWristPitch(
                dx_deg=wdx, motion_profile=wmp)
            self.wrist_roll = gamepad_joints.CommandWristRoll(
                dx_deg=wdx, motion_profile=wmp)
        else:
            self.wrist_pitch = self.wrist_roll = None

        self.gripper_name = RobotJoints.gripper.value
        self.gripper_cmd = self._make_gripper_cmd()

    # -- gripper ---------------------------------------------------------
    def _make_gripper_cmd(self):
        """Gripper command for the installed tool."""
        from stretch4_body.core import gamepad_joints
        if self.gripper_name is None:
            raise NotImplementedError("No gripper configured on this robot.")
        return gamepad_joints.CommandToolPosition(
            name=self.gripper_name,
            motion_profile=Cfg.GRIPPER_MOTION_PROFILE)

    def _apply_gripper(self, want):
        """want is "close", "open" or "hold"."""
        if want == "close":
            self.gripper_cmd.close_gripper(self.robot)
        elif want == "open":
            self.gripper_cmd.open_gripper(self.robot)
        else:
            self.gripper_cmd.stop_gripper(self.robot)

    # -- main ------------------------------------------------------------
    def apply(self, d: Decision):
        r = self.robot
        self.base.command_stick_to_motion(d.base_x, d.base_y, d.base_w, r)

        for cmd, val in ((self.lift, d.lift), (self.arm, d.arm),
                         (self.wrist_yaw, d.wrist_yaw),
                         (self.wrist_pitch, d.wrist_pitch),
                         (self.wrist_roll, d.wrist_roll)):
            if cmd is None:
                continue
            if val != 0.0:
                cmd.command_stick_to_motion(val, r)
            else:
                cmd.stop_motion(r)

        self._apply_gripper(d.gripper)
        self.push()

    def stop_all(self):
        r = self.robot
        self.base.stop_motion(r)
        for cmd in (self.lift, self.arm, self.wrist_yaw,
                    self.wrist_pitch, self.wrist_roll):
            if cmd is not None:
                cmd.stop_motion(r)
        # A held close leaves a goal active in the servo. Guarded so a
        # failure here can't skip the push() that sends the other stops.
        try:
            self.gripper_cmd.stop_gripper(r)
        except Exception as e:
            print(f"  warning: gripper stop failed: {e}", file=sys.stderr)
        self.push()

    def push(self):
        if self.use_server:
            self.robot.push_command(ignore_control_lock=True, priority=1)
        else:
            self.robot.push_command()
