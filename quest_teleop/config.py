"""Tuning knobs: speeds, motion profiles, deadzones."""


class Cfg:
    # --- loop ----------------------------------------------------------
    RATE_HZ = 15.0 # Raising it speeds the wrist up.

    # --- hand translation axes (Quest frame: +Y up, -Z away from you) ---
    UP_AXIS = 1
    UP_SIGN = +1.0
    FWD_AXIS = 2
    FWD_SIGN = -1.0

    # --- hand rotation --------------------------------------------------
    YAW_SIGN = +1.0          # aim left/right -> BASE turn / WRIST yaw
    PITCH_SIGN = -1.0        # aim up/down    -> WRIST pitch
    ROLL_SIGN = +1.0         # joystick X     -> WRIST roll

    # Pointing direction = CTRL_FWD_SIGN * controller axis CTRL_FWD_AXIS.
    CTRL_FWD_AXIS = 2
    CTRL_FWD_SIGN = -1.0
    UP_VEC = (0.0, 1.0, 0.0)
    RIGHT_VEC = (1.0, 0.0, 0.0)

    # --- offset -> velocity ---------------------------------------------
    # Offset from the latched clutch pose sets joint SPEED.
    # Inside *_DEADZONE is ignored; at *_FULL_SCALE the command saturates.
    LIFT_DEADZONE_M = 0.030
    LIFT_FULL_SCALE_M = 0.250
    ARM_DEADZONE_M = 0.030
    ARM_FULL_SCALE_M = 0.250
    ROT_DEADZONE_DEG = 8.0
    ROT_FULL_SCALE_DEG = 50.0

    JS_DEADZONE = 0.15
    JS_X_MODE = "strafe"       # "strafe" | "turn" 

    # --- speed scaling ---------------------------------------------------
    # Multiply the final [-1, 1] commands.  1.0 is full speed. SPEED_SCALE hits 
    # everything.
    SPEED_SCALE = 1.0
    BASE_SPEED_SCALE = 1.0
    LIFT_ARM_SPEED_SCALE = 1.0
    WRIST_SPEED_SCALE = 1.0

    # --- wrist servo travel ----------------------------------------------
    # Degrees requested per tick at full command.
    WRIST_DX_DEG = 5.0
    WRIST_MOTION_PROFILE = "default"   # slow | default | fast | max

    # --- gripper ----------------------------------------------------------
    # HOLD A to close, HOLD B to open, release to stop.  Supports PG4, SG4 and SG4 v2.
    # slow | default | fast | max
    GRIPPER_MOTION_PROFILE = "default"

    # --- motion profiles for base/lift/arm --------------------------------
    MOTION_PROFILE = "default"          # slow | default | fast | max
    MOTION_PROFILE_ANGULAR = "slow"     # base turning


class DexCfg:
    """Tuning knobs for stretch_quest_dex_teleop.py (Dex Teleop control scheme).

    Dex Teleop's own tuning (smoothing, wrist-change guard, base rotation gain
    and clamp, gripper margins) lives upstream in
    third_party/stretch4_dex_teleop/dex_teleop_parameters.py, not here.
    """

    # --- loop -------------------------------------------------------------
    RATE_HZ = 15.0

    # --- gripper orientation before anything has been commanded ---------------
    # diag(1, -1, -1) decodes to wrist (0, 0, 0): straight out and level. Only
    # used by --dry-run; on the robot the first latch anchors on the live wrist.
    NEUTRAL_GRIPPER_R = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))

    # --- joystick: base TRANSLATION, only while the grip is released ---------
    # While gripping, the IK owns base ROTATION (rotate_by); while released the
    # joystick owns translation (set_velocity). They never share a tick:
    # omnibase.move_by() refuses to blend translation with rotation.
    # Full-deflection speed follows the speed profile (slow/default/fast);
    # JS_MAX_V_MPS is only the fallback if RobotParams can't be read. The
    # deadzone is Cfg.JS_DEADZONE (shared with stretch_quest_teleop.py).
    JS_MAX_V_MPS = 0.10
    JS_X_MODE = "strafe"                 # "strafe" | "turn"
    JS_MAX_W_RADPS = 0.35                # ~20 deg/s, only for JS_X_MODE "turn"

    # --- gripper -----------------------------------------------------------
    # HOLD A to close, HOLD B to open, release to stop (same as
    # stretch_quest_teleop.py). slow | default | fast | max
    GRIPPER_MOTION_PROFILE = "default"

    # --- settings chords -----------------------------------------------------
    # Trigger + tap A cycles speed, trigger + tap B cycles strength (collision
    # sensitivity), as on the gamepad (RT + A / RT + B). A tap is a press
    # shorter than this, released while the trigger is still held.
    CHORD_TAP_MAX_S = 1.0
