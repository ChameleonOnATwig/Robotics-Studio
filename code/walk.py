"""
walk.py - makes the two-legged robot walk (2 legs x 3 LX-16A servos), with optional
IMU balance and data recording.

HOW TO RUN (in Terminal, inside the project folder):
    python walk.py                                 # walk with both legs
    python walk.py left                            # only the left leg (e.g. before the right is built)
    python walk.py --seconds 20                    # stop automatically after 20 seconds
    python walk.py left --lean-ankle 8             # override the lean for one run (see LEAN below)
    python walk.py --double-support 0.5            # keep both feet down longer (see DOUBLE_SUPPORT)
  IMU (MPU6050, see imu.py):
    python walk.py --balance                       # use the IMU to adjust the ankles for balance
    python walk.py left --balance-test             # stand still and only balance: tilt it by hand
    python walk.py left --simulate-imu             # pretend IMU, for testing on a Mac
    python walk.py --no-imu                        # walk without the IMU
  Every run with the IMU saves its data to logs/walk_<date>_<time>.csv.
  Stop any time with Control+C: the robot returns to standing, then goes limp.
  The robot's state (walking, fault, stopped, tilt) is written to robot_state.json
  a few times a second, so eyes.py can show matching expressions on the screen.

WHERE SETTINGS LIVE
  leg.py  : PORT, the LEFT leg's calibrated JOINTS, and the safety limits
  walk.py : the RIGHT leg's JOINTS, gait timing, lean, double support, IMU balance
  imu.py  : how the IMU is mounted and calibrated
Keep leg.py and imu.py in the same folder as this file.

HOW THE WALK WORKS
  Each step is a repeating cycle. A "phase" goes from 0 to 2*pi once per step, like the
  hand of a clock. Every joint's angle is worked out from the phase each loop:
      angle = standing angle (+ lean) + swing * sin(phase) + lift * (foot-lift amount)
  The two legs run the same cycle half a step apart, so one foot is down while the
  other steps.

EXCEPTION HANDLING is organised in layers, from outside to inside:
  Layer 0  settings typos                     -> caught before anything moves
  Layer 1  USB serial port, then the IMU      -> clear message saying what to check
  Layer 2  finding the servos                 -> names the joint that isn't answering
  Layer 3  the walking loop                   -> any problem triggers a SAFE SHUTDOWN
  Plus: a dropped SSH connection or a `kill` is treated like Control+C, so the
  safe shutdown still runs.
"""
import argparse               # reads options like --seconds and --balance from the command line
import csv                    # writes the IMU data file
import json                   # writes robot_state.json for the eyes on the screen
import os                     # file paths (to save logs next to this file)
import signal                 # catches "connection dropped" / "kill" signals
import sys                    # sys.exit() ends the program with a message
import time                   # time.sleep() and time.monotonic() (a clock that never jumps)
from math import sin, cos, pi # used to make smooth, repeating step motions

import imu  # our MPU6050 tilt sensor code (imu.py)
import leg  # our settings: PORT, left-leg JOINTS, MAX_TEMP_C, MIN_VIN_MV, MAX_RETRIES, STEP_DT

# The servo library. Each exception type means a different kind of problem,
# so the code can react to each one differently.
from pylx16a.lx16a import (
    LX16A,               # one object per servo; LX16A(4) talks to the servo with ID 4
    ServoError,          # base class: catching this catches all four below
    ServoTimeoutError,   # servo didn't answer (unplugged, no power, wrong ID)
    ServoChecksumError,  # servo's reply was garbled (electrical noise)
    ServoArgumentError,  # we asked for an impossible value, e.g. angle outside limits
    ServoLogicalError,   # wrong state, e.g. move() while torque (motor power) is off
)

# =====================================================================================
# SETTINGS
# =====================================================================================

# ---------------- Right leg: placeholders until it's built and calibrated ----------------
# Same fields as JOINTS in leg.py. IDs must be different from the left leg's (4, 5, 6).
# The hips move side to side, so the hip's swing stays 0 for forward walking.
# If the right leg is mounted as a mirror image of the left, its joints may turn the
# opposite way: flip the sign of swing or lift for any joint that moves backwards.
RIGHT_JOINTS = {
    "hip":   {"id": 4, "limits": (60, 180), "neutral": 120, "swing": 0,  "lift": 0},
    "knee":  {"id": 5, "limits": (80, 200), "neutral": 140, "swing": 0,  "lift": 20},
    "ankle": {"id": 6, "limits": (60, 180), "neutral": 120, "swing": 0,  "lift": 0},
}

# Both legs' settings in one place: the left leg's come from leg.py.
LEGS = {"left": leg.JOINTS, "right": RIGHT_JOINTS}

# The legs run the same step cycle, half a cycle (pi) apart: while one foot lifts and
# swings forward, the other stays on the ground and pushes back, so one foot is always down.
PHASE_OFFSET = {"left": 0.0, "right": pi}

# ---------------- Gait settings ----------------
STEP_TIME = 3.0          # seconds for one full step cycle (bigger = slower walking)
RAMP_STEPS = 2           # ease in over the first 2 cycles instead of starting at full stride
STEP_DT = leg.STEP_DT    # seconds per loop (from leg.py): a new position 20 times a second
HEALTH_EVERY = 20        # check temperature/voltage every N loops (~once per second)

# Lean: degrees added to each joint's standing angle, on both legs, for the whole walk.
# Use it to shift the weight forward if the robot tips backward (or back if it tips forward).
# The ankle shifts the weight forward/back; the hips move side to side, so a hip lean
# tilts the robot sideways. Which sign is "forward" depends on how the servo is mounted:
# try +5, and if the robot tips back MORE, use -5.
# Can be overridden for one run with --lean-hip / --lean-knee / --lean-ankle.
# The robot is back-heavy, so the ankle leans forward by default (+5 worked in testing).
LEAN = {"hip": 0, "knee": 0, "ankle": 5}

# Double support: how much of each step BOTH feet stay on the ground, from 0 to 0.9.
#   0   = a foot is always lifting (fastest, least stable)
#   0.3 = feet lift only in the middle of each swing; both are down around the change-over
# Higher is steadier but leaves less time for the foot to clear the ground: if the foot
# starts scuffing, lower this or raise the knee's lift. Override with --double-support.
DOUBLE_SUPPORT = 0.3

# ---------------- IMU: data recording, fall protection, balance ----------------
USE_IMU = True       # look for the IMU at startup (if it's missing, walk without it unless --balance)
FALL_ANGLE = 30      # stop safely if the body tilts more than this many degrees (any direction)
LOG_DIR = "logs"     # where the IMU data files are saved

# Balance (only active with --balance): when the body tilts, lean the joints to push it back.
#   correction = -(gain * tilt + damping * tilting speed), limited to max_correction
# "gain" reacts to HOW FAR it's tilted; "damping" reacts to HOW FAST it's tilting,
# which stops the correction overshooting and wobbling.
BALANCE = {
    "target_pitch": 0.0,    # body pitch to hold; use the value --balance-test shows when it stands well
    "pitch_gain": 0.5,      # ankle degrees per degree of forward/back tilt
    "pitch_damping": 0.02,  # ankle degrees per degree/second of tilting (calms overshoot)
    "roll_gain": 0.0,       # hip degrees per degree of sideways tilt (0 = off until you use the hips)
    "roll_damping": 0.0,
    "max_correction": 8,    # never correct a joint by more than this many degrees
}
# Which way each joint pushes. +1 means a POSITIVE angle change moves the body FORWARD
# (ankle) or to the RIGHT (hip); -1 means the opposite. Your left ankle's +lean leans the
# body forward, so it's +1. A mirrored right leg may need -1: check with --balance-test.
BALANCE_DIRECTION = {"left": {"ankle": 1, "hip": 1}, "right": {"ankle": 1, "hip": 1}}

# ---------------- Screen: share the robot's state with eyes.py ----------------
SHARE_STATE = True               # write robot_state.json for the eyes (False = don't)
STATE_FILE = "robot_state.json"  # saved next to this file; eyes.py reads the same file


# =====================================================================================
# SAFETY HELPERS
# =====================================================================================

def _stop_on_signal(signum, frame):
    """A closed SSH session (SIGHUP) or a `kill` (SIGTERM) would normally end the program
    instantly, skipping the safe shutdown. Treat them like Control+C instead.
    After a lost connection there's no screen to print to, so messages go to a log file
    (walk_last_run.log) that you can read when you reconnect."""
    if signum == signal.SIGHUP:
        log = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "walk_last_run.log"), "w")
        sys.stdout = sys.stderr = log         # send all further messages to the file
        print("Connection to the terminal was lost; stopping safely.")
    raise KeyboardInterrupt                   # the walking loop handles this like Control+C


class RobotFault(Exception):
    """Our own exception for 'the robot cannot continue safely'
    (overheating, low voltage, a servo or the IMU stopped working, tipped too far)."""


def joint_name(servo_id):
    """'servo 5 (right knee)' instead of just 'servo 5', so you know which cable to check."""
    for leg_name, joints in LEGS.items():
        for joint, cfg in joints.items():
            if cfg["id"] == servo_id:
                return f"servo {servo_id} ({leg_name} {joint})"
    return f"servo {servo_id}"                # an ID that isn't in either leg's settings


def check_config():
    """Layer 0: catch typos in either leg's settings before anything moves.
    Raises ValueError with a message saying exactly what is wrong."""
    # Every servo on BOTH legs needs its own ID, or two servos would answer one command.
    ids = [cfg["id"] for joints in LEGS.values() for cfg in joints.values()]
    duplicates = sorted({i for i in ids if ids.count(i) > 1})
    if duplicates:
        raise ValueError(f"servo ID(s) {duplicates} used more than once. "
                         "Every joint on both legs needs its own ID.")
    # Limits must be a real range the servo can reach, with the standing angle inside it.
    for leg_name, joints in LEGS.items():
        for joint, cfg in joints.items():
            lo, hi = cfg["limits"]
            if not 0 <= lo < hi <= 240:
                raise ValueError(f"{leg_name} {joint}: limits must satisfy 0 <= low < high <= 240 "
                                 f"(got {lo}, {hi})")
            if not lo <= cfg["neutral"] <= hi:
                raise ValueError(f"{leg_name} {joint}: neutral {cfg['neutral']} is outside "
                                 f"its limits {lo}-{hi}")
    # Gait and IMU settings must be sensible numbers.
    if STEP_TIME <= 0:
        raise ValueError("STEP_TIME must be greater than 0")
    if not 0 <= DOUBLE_SUPPORT <= 0.9:
        raise ValueError(f"DOUBLE_SUPPORT must be between 0 and 0.9 (got {DOUBLE_SUPPORT})")
    if not 5 <= FALL_ANGLE <= 80:
        raise ValueError(f"FALL_ANGLE should be between 5 and 80 degrees (got {FALL_ANGLE})")
    if not 0 <= BALANCE["max_correction"] <= 30:
        raise ValueError("BALANCE max_correction should be between 0 and 30 degrees")
    for leg_name, dirs in BALANCE_DIRECTION.items():
        for joint, d in dirs.items():
            if d not in (1, -1):
                raise ValueError(f"BALANCE_DIRECTION {leg_name} {joint} must be 1 or -1 (got {d})")
    # The standing angle PLUS the lean must still be inside each joint's limits.
    for leg_name, joints in LEGS.items():
        for joint, cfg in joints.items():
            stance = cfg["neutral"] + LEAN.get(joint, 0)
            lo, hi = cfg["limits"]
            if not lo <= stance <= hi:
                raise ValueError(f"{leg_name} {joint}: standing angle {cfg['neutral']} + lean "
                                 f"{LEAN.get(joint, 0)} = {stance} is outside its limits {lo}-{hi}. "
                                 "Use a smaller lean.")


# =====================================================================================
# THE ROBOT
# =====================================================================================

class Robot:
    """Everything the program can do with the legs: set up the servos, move a joint
    safely, check servo health, put the legs at a point in the step, shut down safely."""

    def __init__(self, leg_names):
        """Layer 2: find and configure every servo on the chosen legs. Can raise
        ServoTimeoutError (a servo didn't answer) - main() turns that into a clear message."""
        # Connect to every servo LIMP (torque off) first. If one is missing, the ones
        # already found aren't left stiff and holding position when the program exits.
        self.joints = {}  # "left hip" -> (servo, cfg, phase offset, standing angle incl. lean)
        for leg_name in leg_names:
            for joint, cfg in LEGS[leg_name].items():
                servo = LX16A(cfg["id"], disable_torque=True)
                stance = cfg["neutral"] + LEAN.get(joint, 0)   # standing angle + lean
                self.joints[f"{leg_name} {joint}"] = (servo, cfg, PHASE_OFFSET[leg_name], stance)

        for servo, cfg, _, _ in self.joints.values():
            servo.servo_mode()                    # in case a servo was left in motor mode
            servo.set_angle_limits(*cfg["limits"])        # the servo itself also enforces the limits
            servo.set_temp_limit(leg.MAX_TEMP_C)          # the servo protects itself above this
            servo.set_led_error_triggers(True, True, True)  # LED flashes on temp/voltage/stall

        # The servo stores limits in its own units (34 deg is kept as 34.08 deg),
        # so clamp to the limits it actually holds.
        self.limits = {name: s.get_angle_limits() for name, (s, _, _, _) in self.joints.items()}
        self.warned = set()  # joints that already printed a clamp warning

        # Every servo answered and is configured: now it's safe to power them up.
        for servo, _, _, _ in self.joints.values():
            servo.enable_torque()

    # ---------- Exception-handling helpers ----------
    def _with_retry(self, func):
        """Call a servo reading (like get_temp). A single failed reading is usually a
        glitch, so try again; only MAX_RETRIES failures in a row count as a real fault.
        This is how an unplugged servo is detected while walking."""
        for attempt in range(1, leg.MAX_RETRIES + 1):
            try:
                return func()                     # success: hand back the reading
            except (ServoTimeoutError, ServoChecksumError) as e:
                if attempt == leg.MAX_RETRIES:
                    # "from e" keeps the original error attached for debugging.
                    raise RobotFault(f"{joint_name(e.id_)} failed {leg.MAX_RETRIES} reads "
                                     f"in a row ({type(e).__name__})") from e
                time.sleep(0.01)                  # short pause before trying again

    def move_joint(self, name, angle, time_ms=0):
        """Move one joint (e.g. 'left knee') to an angle over time_ms milliseconds,
        never outside its limits."""
        servo, _, _, _ = self.joints[name]
        lo, hi = self.limits[name]
        clamped = max(lo, min(hi, angle))   # the library raises on out-of-range angles
        if abs(clamped - angle) > 0.5 and name not in self.warned:
            self.warned.add(name)           # warn once per joint, not every loop
            print(f"[warn] {name}: target {angle:.1f} is past its limit, clamped to {clamped:.1f} "
                  "(this joint is hitting its limit; further warnings hidden)")
        try:
            servo.move(clamped, time=int(time_ms))
        except ServoLogicalError:
            # Most common cause: torque is disabled. Recover once; otherwise re-raise.
            if servo.is_torque_enabled():
                raise
            print(f"[warn] {name}: torque was off, re-enabling")
            servo.enable_torque()
            servo.move(clamped, time=int(time_ms))

    def check_health(self):
        """Read every servo's temperature and voltage; raise RobotFault if one is unsafe
        or has stopped answering."""
        for name, (servo, _, _, _) in self.joints.items():
            temp = self._with_retry(servo.get_temp)   # degrees C
            vin = self._with_retry(servo.get_vin)     # millivolts
            if temp >= leg.MAX_TEMP_C:
                raise RobotFault(f"{name} servo overheating: {temp} C")
            if vin < leg.MIN_VIN_MV:
                raise RobotFault(f"{name} servo low voltage: {vin} mV")

    # ---------- Motion ----------
    def go_neutral(self, time_ms=1000):
        """Move every joint to its standing angle (including lean), slowly, then wait."""
        for name, (_, _, _, stance) in self.joints.items():
            self.move_joint(name, stance, time_ms)
        time.sleep(time_ms / 1000)                # let the move finish

    def pose(self, phase, amount, time_ms, corrections=None):
        """Put every joint at its point in the step cycle.
        swing * sin  : forward/back motion over the whole cycle
        lift  * cos+ : extra bend only during the middle of the forward swing (lifts the foot)
        amount       : 0..1, scales the motion so walking eases in smoothly.
        The lift starts only once cos passes DOUBLE_SUPPORT, so around each change-over
        (when the legs are at the ends of their swing) both feet are on the ground.
        corrections  : extra degrees per joint from the IMU balance (not scaled by amount)."""
        corrections = corrections or {}           # no balance corrections -> empty
        for name, (_, cfg, offset, stance) in self.joints.items():
            p = phase + offset                    # the right leg is half a cycle behind
            # 0 for most of the cycle; rises to 1 only in the middle of the forward swing
            lift_amount = max(0.0, (cos(p) - DOUBLE_SUPPORT) / (1 - DOUBLE_SUPPORT))
            motion = cfg["swing"] * sin(p) + cfg["lift"] * lift_amount
            self.move_joint(name, stance + amount * motion + corrections.get(name, 0.0), time_ms)

    def shutdown(self):
        """Safe shutdown: return to standing, then turn torque off so the legs go limp.
        ALWAYS runs (called from a 'finally' block). Each step has its own try/except,
        so one failure can't stop the steps after it."""
        print("Shutting down...")
        try:
            self.go_neutral(800)
        except KeyboardInterrupt:
            print("[shutdown] skipping neutral pose")  # a 2nd Ctrl+C must not skip torque-off
        except (ServoError, OSError) as e:
            print(f"[shutdown] couldn't reach neutral: {e}")
        all_off = True                            # becomes False if any servo can't be reached
        for name, (servo, _, _, _) in self.joints.items():
            try:
                if servo.is_torque_enabled():
                    servo.disable_torque()
            except (ServoError, OSError) as e:
                all_off = False
                print(f"[shutdown] couldn't disable {name} torque: {e}")
        if all_off:
            print("All joints limp. Done.")
        else:
            # The program couldn't reach some servo, so it can't promise the robot is safe.
            print("WARNING: torque may still be ON. Switch off the servo power now!")


# =====================================================================================
# IMU BALANCE AND DATA RECORDING
# =====================================================================================

def balance_corrections(sensor, joint_names):
    """How far to lean each ankle (forward/back tilt) and hip (sideways tilt) to push
    the body back toward upright. Returns ({joint name: degrees}, ankle, hip)."""
    b = BALANCE
    lim = b["max_correction"]
    # Tilted forward (pitch > target) -> negative correction, leaning the ankles back; and
    # the faster it's tipping (pitch_rate), the harder it pushes against it.
    ankle = -(b["pitch_gain"] * (sensor.pitch - b["target_pitch"]) + b["pitch_damping"] * sensor.pitch_rate)
    hip = -(b["roll_gain"] * sensor.roll + b["roll_damping"] * sensor.roll_rate)
    ankle, hip = max(-lim, min(lim, ankle)), max(-lim, min(lim, hip))   # cap both corrections
    corrections = {}
    for name in joint_names:
        leg_name, joint = name.split()            # "left ankle" -> "left", "ankle"
        if joint == "ankle":
            corrections[name] = ankle * BALANCE_DIRECTION[leg_name]["ankle"]
        elif joint == "hip":
            corrections[name] = hip * BALANCE_DIRECTION[leg_name]["hip"]
    return corrections, ankle, hip


class DataLog:
    """Saves one row of IMU data per loop to a CSV file (opens in Excel / Google Sheets).
    A problem writing the file (disk full, folder missing) must never stop the robot
    mid-step, so write errors switch logging off with a warning instead."""

    # The column headings in the file, in order.
    COLUMNS = ["time_s", "phase_deg", "step_amount", "pitch_deg", "roll_deg",
               "pitch_rate_dps", "roll_rate_dps", "accel_x_g", "accel_y_g", "accel_z_g",
               "gyro_x_dps", "gyro_y_dps", "gyro_z_dps", "ankle_correction_deg", "hip_correction_deg"]

    def __init__(self):
        """Create logs/walk_<date>_<time>.csv next to this file and write the headings."""
        folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), LOG_DIR)
        os.makedirs(folder, exist_ok=True)        # make the logs folder if it isn't there yet
        self.path = os.path.join(folder, time.strftime("walk_%Y-%m-%d_%H-%M-%S.csv"))
        self.file = open(self.path, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(self.COLUMNS)
        self.rows = 0                             # how many data rows have been written

    def write(self, t, phase, amount, sensor, ankle, hip):
        """Add one row: the time, where we are in the step, and everything the IMU measured."""
        if self.file is None:                     # logging was switched off after an error
            return
        try:
            self.writer.writerow([f"{t:.3f}", f"{phase * 180 / pi:.1f}", f"{amount:.2f}",
                                  f"{sensor.pitch:.2f}", f"{sensor.roll:.2f}",
                                  f"{sensor.pitch_rate:.2f}", f"{sensor.roll_rate:.2f}",
                                  *(f"{v:.3f}" for v in sensor.accel), *(f"{v:.2f}" for v in sensor.gyro),
                                  f"{ankle:.2f}", f"{hip:.2f}"])
            self.rows += 1
            if self.rows % 20 == 0:
                self.file.flush()   # save regularly so a crash or power cut keeps the data
        except OSError as e:
            print(f"[warn] couldn't write the data log ({e}); logging switched off, still walking.")
            self.close()

    def close(self):
        """Close the file (safe to call more than once)."""
        if self.file is not None:
            try:
                self.file.close()
            except OSError:
                pass
            self.file = None


class StateFile:
    """Writes robot_state.json a few times a second so eyes.py (the face on the screen)
    can show what the robot is doing. Like the data log, a problem writing it must never
    stop the robot, so errors just switch it off with one warning."""

    def __init__(self):
        self.path = os.path.join(os.path.dirname(os.path.abspath(__file__)), STATE_FILE)
        self.on = SHARE_STATE
        self.mode = None            # last mode written
        self.since = time.time()    # when the current mode started
        self.last = 0.0             # when the file was last written

    def write(self, mode, pitch=None, roll=None, message="", force=False):
        """Save the state. Writes at most 5 times a second unless force=True (used for
        important changes like a fault, so the eyes react straight away)."""
        if not self.on:
            return
        now = time.time()
        if mode != self.mode:
            self.mode, self.since, force = mode, now, True      # a new mode is always written
        if not force and now - self.last < 0.2:
            return
        self.last = now
        data = {"mode": mode, "pitch": pitch, "roll": roll, "message": message,
                "time": now, "since": self.since}
        temp = self.path + ".tmp"
        try:
            with open(temp, "w") as f:
                json.dump(data, f)
            os.replace(temp, self.path)   # swap in the whole file at once, so eyes.py never reads half
        except OSError as e:
            print(f"[warn] couldn't write {STATE_FILE} ({e}); the eyes won't follow the robot this run.")
            self.on = False


# =====================================================================================
# MAIN PROGRAM
# =====================================================================================

def main():
    """Read the command-line options, set everything up layer by layer, then walk
    until stopped. Whatever happens, the safe shutdown runs at the end."""

    # ---------- Command-line options ----------
    parser = argparse.ArgumentParser(description="Walk the two-legged robot.")
    parser.add_argument("legs", nargs="?", default="both", choices=["both", "left", "right"],
                        help="which legs to use (default: both)")
    parser.add_argument("--seconds", type=float, default=None,
                        help="stop automatically after this many seconds")
    parser.add_argument("--double-support", type=float, default=None, metavar="FRACTION",
                        help="how much of each step both feet stay down, 0 to 0.9 "
                             "(overrides DOUBLE_SUPPORT for this run)")
    parser.add_argument("--balance", action="store_true",
                        help="use the IMU to adjust the ankles (and hips, if roll_gain is set) for balance")
    parser.add_argument("--balance-test", action="store_true",
                        help="don't step: hold the standing pose and only balance, to check directions")
    imu_choice = parser.add_mutually_exclusive_group()   # can't use both of these at once
    imu_choice.add_argument("--no-imu", action="store_true", help="walk without the IMU")
    imu_choice.add_argument("--simulate-imu", action="store_true",
                            help="use a pretend IMU (for testing on a Mac)")
    for joint in ("hip", "knee", "ankle"):
        parser.add_argument(f"--lean-{joint}", type=float, default=None, metavar="DEG",
                            help=f"lean for the {joint}, in degrees (overrides LEAN for this run)")
    args = parser.parse_args()

    # Options typed on the command line override the settings at the top of the file.
    for joint in ("hip", "knee", "ankle"):
        value = getattr(args, f"lean_{joint}")
        if value is not None:
            LEAN[joint] = value
    global DOUBLE_SUPPORT                         # we're changing the setting, not a local copy
    if args.double_support is not None:
        DOUBLE_SUPPORT = args.double_support
    leg_names = ["left", "right"] if args.legs == "both" else [args.legs]
    if args.balance_test:
        args.balance = True                       # the balance test needs balance switched on

    # Share the robot's state with the eyes on the screen.
    face = StateFile()

    # Stop safely even if the SSH connection drops or the program is killed.
    signal.signal(signal.SIGTERM, _stop_on_signal)
    if hasattr(signal, "SIGHUP"):                 # SIGHUP doesn't exist on Windows
        signal.signal(signal.SIGHUP, _stop_on_signal)

    # Layer 0: settings typos. Caught before anything moves.
    try:
        check_config()
    except ValueError as e:
        face.write("fault", message=f"Config error: {e}")
        sys.exit(f"Config error: {e}")

    # Layer 1: the serial port. pyserial raises SerialException (an OSError) if it's wrong/busy.
    try:
        LX16A.initialize(leg.PORT)
    except OSError as e:
        face.write("fault", message=f"Could not open {leg.PORT}")
        sys.exit(f"Could not open {leg.PORT}: {e}\n(Run `ls /dev/cu.*` and check PORT in leg.py)")

    # Layer 1b: the IMU. Opened BEFORE the servos power up, so the robot is still
    # while the gyro calibrates. Balancing needs it; plain walking can go on without it.
    sensor = None                                 # stays None if there's no IMU
    if args.simulate_imu or (USE_IMU and not args.no_imu):
        try:
            sensor = imu.open_imu(simulate=args.simulate_imu)
            print("IMU ready" + (" (simulated)" if args.simulate_imu else "")
                  + f": pitch {sensor.pitch:+.1f}, roll {sensor.roll:+.1f} deg.")
        except imu.IMUError as e:
            if args.balance:
                face.write("fault", message=f"IMU problem: {e}")
                sys.exit(f"IMU problem: {e}\nBalancing needs the IMU. Fix it, or run without --balance.")
            print(f"[warn] IMU not available ({e}). Walking without IMU data.")
    elif args.balance:
        sys.exit("--balance needs the IMU. Remove --no-imu, or set USE_IMU = True.")

    # Start the data file (only if there's an IMU to record from).
    log = None
    if sensor:
        try:
            log = DataLog()
            print(f"Recording IMU data to {os.path.relpath(log.path)}")
        except OSError as e:
            print(f"[warn] couldn't create the data log ({e}). Walking without recording.")

    # Layer 2: finding and configuring the servos. Close the IMU and data file first
    # if this fails, then exit with a message naming the problem.
    try:
        robot = Robot(leg_names)
    except (ServoError, OSError) as e:
        if log:
            log.close()
        if sensor:
            sensor.dev.close()
        face.write("fault", message=f"Setup failed: {e}")
        if isinstance(e, ServoTimeoutError):
            sys.exit(f"{joint_name(e.id_)} is not responding. Check its cable, power, and ID.")
        if isinstance(e, ServoError):
            sys.exit(f"Setup failed on {joint_name(e.id_)}: {type(e).__name__}: {e}")
        sys.exit(f"Lost connection to the servo bus during setup: {e}")

    # Layer 3: the walking loop. Whatever goes wrong inside 'try', the matching 'except'
    # prints why, and 'finally' ALWAYS runs the safe shutdown afterwards.
    end_mode, end_message = "stopped", ""        # what the eyes show after the loop ends
    try:
        face.write("starting")
        robot.go_neutral()
        # Print a summary line of the settings for this run.
        leaning = ", ".join(f"{j} {v:+g}" for j, v in LEAN.items() if v)
        mode = "Balance test (standing still)" if args.balance_test else "Walking"
        print(f"{mode} with: {', '.join(leg_names)} leg(s)"
              + (f", lean: {leaning}" if leaning else "")
              + ("" if args.balance_test else f", double support: {DOUBLE_SUPPORT:g}")
              + (", balance ON" if args.balance else "")
              + ". Press Control+C to stop.")
        if args.balance_test:
            print("Tilt the body gently forward: the ankles should lean to push it BACK.")
        start, cycle = time.monotonic(), 0
        while args.seconds is None or time.monotonic() - start < args.seconds:
            t = time.monotonic() - start          # seconds since walking started
            phase = (2 * pi * t / STEP_TIME) % (2 * pi)     # based on the clock, so speed stays steady
            # How much of the full stride to use: 0 in the balance test (stand still),
            # otherwise rising from 0 to 1 over the first RAMP_STEPS steps (smooth start).
            amount = 0.0 if args.balance_test else min(1.0, t / (RAMP_STEPS * STEP_TIME))

            # --- IMU: read the tilt, check for a fall, work out balance, record data ---
            corrections, ankle_corr, hip_corr = {}, 0.0, 0.0
            if sensor:
                try:
                    pitch, roll = sensor.update()
                except imu.IMUError as e:
                    if args.balance:
                        # Balancing without a working IMU isn't safe: stop.
                        raise RobotFault(f"IMU stopped working: {e}")
                    # Not balancing: we only lose the data, so carry on walking.
                    print(f"[warn] IMU stopped working ({e}). Walking on without IMU data.")
                    sensor = None
                else:                             # runs only if the reading succeeded
                    if abs(pitch) > FALL_ANGLE or abs(roll) > FALL_ANGLE:
                        raise RobotFault(f"body tilted too far (pitch {pitch:+.0f}, roll {roll:+.0f} deg, "
                                         f"limit {FALL_ANGLE}). Stopped to avoid a fall.")
                    if args.balance:
                        corrections, ankle_corr, hip_corr = balance_corrections(sensor, robot.joints)
                    if log:
                        log.write(t, phase, amount, sensor, ankle_corr, hip_corr)
                    if args.balance_test and cycle % 10 == 0:
                        # Twice a second, show the tilt and the correction on one updating line.
                        print(f"\r  pitch {sensor.pitch:+6.1f}  roll {sensor.roll:+6.1f}  ->  "
                              f"ankle correction {ankle_corr:+5.1f}  hip correction {hip_corr:+5.1f}   ",
                              end="", flush=True)

            # --- Tell the eyes what's happening (at most 5 times a second) ---
            face.write("balance_test" if args.balance_test else "walking",
                       sensor.pitch if sensor else None, sensor.roll if sensor else None)

            # --- Move the legs to this loop's point in the step ---
            robot.pose(phase, amount, STEP_DT * 1000, corrections)
            if cycle % HEALTH_EVERY == 0:
                robot.check_health()              # ~once a second: temperature, voltage, unplugged servo
            time.sleep(STEP_DT)
            cycle += 1
        print("Time's up.")
    except KeyboardInterrupt:
        print("\nStopped by user (Control+C).")
    except RobotFault as e:
        print(f"[FAULT] {e}")
        end_mode, end_message = "fault", str(e)
    except ServoArgumentError as e:
        print(f"[BUG] invalid value sent to {joint_name(e.id_)}: {e}")  # clamping should prevent this
        end_mode, end_message = "fault", str(e)
    except ServoError as e:
        print(f"[SERVO ERROR] {joint_name(e.id_)}: {type(e).__name__}: {e}")
        end_mode, end_message = "fault", str(e)
    except OSError as e:
        # pyserial errors are NOT ServoErrors: USB unplugged, write timeout, etc.
        print(f"[SERIAL ERROR] lost connection to the servo bus: {e}")
        end_mode, end_message = "fault", str(e)
    finally:
        robot.shutdown()
        face.write(end_mode, message=end_message)
        if log:
            log.close()
            print(f"IMU data saved: {os.path.relpath(log.path)} ({log.rows} rows)")
        if sensor:
            sensor.dev.close()


# Only run main() when this file is started directly (python walk.py), not when
# another file imports it.
if __name__ == "__main__":
    main()
