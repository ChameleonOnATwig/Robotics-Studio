"""
leg.py - controls ONE three-joint robot leg (hip, knee, ankle) built from
HiWonder LX-16A servos, using the PyLX-16A library.

This file has two jobs:
  1. It holds the SETTINGS for the leg (port, servo IDs, calibrated angles,
     safety limits). walk.py and disconnect_test.py read their settings from here.
  2. Run on its own, it makes the leg step in the air, which is handy for testing.

    Install:  pip install pylx16a
    Run:      python leg.py
    Stop:     Control+C   (the leg returns to standing, then goes limp)

EXCEPTION HANDLING is organised in layers, from outside to inside:
    Layer 0  settings typos           -> caught before anything moves
    Layer 1  the USB serial port      -> clear message if the port is wrong
    Layer 2  finding the servos       -> names the joint that isn't answering
    Layer 3  the walking loop         -> any problem triggers a SAFE SHUTDOWN
"""
import sys                    # sys.exit() ends the program with a message
import time                   # time.sleep() pauses between loop steps
from math import sin, cos, pi # used to make smooth, repeating step motions

# The servo library. Each exception type means a different kind of problem,
# so the code can react to each one differently.
from pylx16a.lx16a import (
    LX16A,               # one object per servo; LX16A(3) talks to the servo with ID 3
    ServoError,          # base class: catching this catches all four below
    ServoTimeoutError,   # servo didn't answer (unplugged, no power, wrong ID)
    ServoChecksumError,  # servo's reply was garbled (electrical noise)
    ServoArgumentError,  # we asked for an impossible value, e.g. angle outside limits
    ServoLogicalError,   # wrong state, e.g. move() while torque (motor power) is off
)

# =====================================================================================
# SETTINGS - measure these on YOUR leg (calibration saves limits/neutral automatically)
# =====================================================================================

# The USB port the BusLinker board shows up as.
#   Mac:          run `ls /dev/cu.*`  -> e.g. "/dev/cu.usbserial-10"
#   Raspberry Pi: run `ls /dev/ttyUSB*` -> usually "/dev/ttyUSB0"
PORT = "/dev/cu.usbserial-10"

# One entry per joint, listed from the body down to the foot.
#   id      : the servo's ID number (every servo on the bus needs a different one)
#   limits  : safe range in servo degrees (0-240); the code never moves the joint outside it
#   neutral : the joint's angle in the standing pose (must be inside limits)
#   swing   : degrees of back-and-forth motion during each step (follows a sine wave)
#   lift    : extra degrees of bend while the foot swings forward (lifts the foot)
# If a joint moves the wrong way, flip the sign of its swing or lift (e.g. 20 -> -20).
JOINTS = {
    "hip":   {"id": 1, "limits": (60, 180), "neutral": 120, "swing": 15, "lift": 0},
    "knee":  {"id": 2, "limits": (80, 200), "neutral": 140, "swing": 0,  "lift": 20},
    "ankle": {"id": 3, "limits": (60, 180), "neutral": 120, "swing": 0,  "lift": 0},
}

MAX_TEMP_C = 70        # stop if any servo gets this hot (degrees C); the library accepts 50-100
MIN_VIN_MV = 6000      # stop if the battery drops below this (millivolts); 10500 suits a 3S LiPo
MAX_RETRIES = 3        # how many times to retry a failed servo reading before giving up
STEP_DT = 0.05         # seconds per loop: the leg gets a new position 20 times a second
PHASE_STEP = 0.1       # how far through the step cycle each loop moves (bigger = faster steps)


class LegFault(Exception):
    """Our own exception for 'the leg cannot continue safely'
    (overheating, low voltage, or a servo that stopped answering)."""


def check_config():
    """Layer 0: catch typos in JOINTS before anything moves.
    Raises ValueError with a message saying exactly what is wrong."""
    # Every servo needs its own ID, or two servos would answer the same command.
    ids = [cfg["id"] for cfg in JOINTS.values()]
    if len(set(ids)) != len(ids):          # a set drops duplicates, so lengths differ if any repeat
        raise ValueError(f"Two joints share a servo ID: {ids}. Each joint needs its own ID.")
    for name, cfg in JOINTS.items():
        lo, hi = cfg["limits"]
        # The servo can only reach 0-240 degrees, and low must be below high.
        if not 0 <= lo < hi <= 240:
            raise ValueError(f"{name}: limits must satisfy 0 <= low < high <= 240 (got {lo}, {hi})")
        # The standing angle has to be reachable, i.e. inside the limits.
        if not lo <= cfg["neutral"] <= hi:
            raise ValueError(f"{name}: neutral {cfg['neutral']} is outside its limits {lo}-{hi}")


def joint_name(servo_id):
    """Turn a servo ID into a readable label like 'servo 2 (knee)' for error messages,
    so you know which cable to check."""
    for name, cfg in JOINTS.items():
        if cfg["id"] == servo_id:
            return f"servo {servo_id} ({name})"
    return f"servo {servo_id}"    # an ID that isn't in JOINTS


class Leg:
    """Everything the program can do with the leg: set it up, move a joint safely,
    check servo health, take steps, and shut down safely."""

    def __init__(self):
        """Layer 2: find and configure every servo. Can raise ServoTimeoutError
        (a servo didn't answer) - main() turns that into a clear message."""
        # Connect to every servo LIMP (torque off) first. If one is missing, the ones
        # already found aren't left stiff and holding position when the program exits.
        self.joints = {}               # name -> (servo object, its settings)
        for name, cfg in JOINTS.items():
            self.joints[name] = (LX16A(cfg["id"], disable_torque=True), cfg)

        for servo, cfg in self.joints.values():
            # The servo remembers "motor mode" (continuous spinning) across power cycles,
            # and move() raises ServoLogicalError in motor mode, so force position mode.
            servo.servo_mode()
            servo.set_angle_limits(*cfg["limits"])   # the servo itself also enforces the limits
            servo.set_temp_limit(MAX_TEMP_C)         # the servo protects itself above this
            servo.set_led_error_triggers(True, True, True)  # LED flashes on temp/voltage/stall

        # The servo stores limits in its own units, so 34 degrees is kept as 34.08 degrees.
        # Read back the limits it ACTUALLY holds and clamp to those, or angles right at
        # the edge would be rejected with ServoArgumentError.
        self.limits = {name: servo.get_angle_limits() for name, (servo, _) in self.joints.items()}

        # Every servo answered and is configured: now it's safe to power them up.
        for servo, _ in self.joints.values():
            servo.enable_torque()

    # ---------- Exception-handling helpers ----------
    def _with_retry(self, func, *args, **kwargs):
        """Call a servo reading (like get_temp). A single failed reading is usually a
        glitch (electrical noise, a dropped byte), so try again a few times. Only if it
        fails MAX_RETRIES times in a row do we decide the servo is really gone."""
        for attempt in range(1, MAX_RETRIES + 1):
            try:
                return func(*args, **kwargs)          # success: hand back the reading
            except (ServoTimeoutError, ServoChecksumError) as e:
                if attempt == MAX_RETRIES:
                    # Out of retries: report it as a leg fault naming the joint.
                    # "from e" keeps the original error attached for debugging.
                    raise LegFault(
                        f"{joint_name(e.id_)} failed {MAX_RETRIES} reads in a row ({type(e).__name__})"
                    ) from e
                time.sleep(0.01)                      # short pause before trying again

    def move_joint(self, name, angle, time_ms=0):
        """Move one joint to an angle over time_ms milliseconds, never outside its limits."""
        servo, _ = self.joints[name]
        lo, hi = self.limits[name]

        # The library RAISES an error on out-of-range angles instead of clamping,
        # so clamp here (keep the angle between lo and hi) to prevent that.
        clamped = max(lo, min(hi, angle))
        if abs(clamped - angle) > 0.5:  # ignore tiny rounding differences
            print(f"[warn] {name}: target {angle:.1f} clamped to {clamped:.1f}")

        try:
            servo.move(clamped, time=int(time_ms))
        except ServoLogicalError:
            # The most common cause is that torque got switched off. Fix that once and
            # retry; if torque was already on, it's some other problem, so pass it up.
            if servo.is_torque_enabled():
                raise  # some other state problem (e.g. motor mode) -- don't guess
            print(f"[warn] {name}: torque was off, re-enabling")
            servo.enable_torque()
            servo.move(clamped, time=int(time_ms))

    def check_health(self):
        """Read every servo's temperature and voltage; raise LegFault if one is unsafe.
        Readings use _with_retry, so a servo that stops answering is also caught here -
        this is how an unplugged servo gets detected while walking."""
        for name, (servo, _) in self.joints.items():
            temp = self._with_retry(servo.get_temp)   # degrees C
            vin = self._with_retry(servo.get_vin)     # millivolts
            if temp >= MAX_TEMP_C:
                raise LegFault(f"{name} servo overheating: {temp} C")
            if vin < MIN_VIN_MV:
                raise LegFault(f"{name} servo low voltage: {vin} mV")

    # ---------- Motion ----------
    def go_neutral(self, time_ms=1000):
        """Move every joint to its standing angle, slowly (over time_ms), then wait."""
        for name, (_, cfg) in self.joints.items():
            self.move_joint(name, cfg["neutral"], time_ms)
        time.sleep(time_ms / 1000)                    # let the move finish

    def step(self, phase, time_ms):
        """Put every joint at one point of the step cycle. 'phase' goes around 0 -> 2*pi
        once per step, like the hand of a clock.
          swing * sin(phase) : smooth back-and-forth motion
          lift * cos(phase)  : positive only while the swing is moving forward, so the
                               extra bend (foot lift) happens on the forward swing only"""
        lift_amount = max(0.0, cos(phase))            # 0 when cos is negative, so no lift
        for name, (_, cfg) in self.joints.items():
            target = cfg["neutral"] + cfg["swing"] * sin(phase) + cfg["lift"] * lift_amount
            self.move_joint(name, target, time_ms)

    def shutdown(self):
        """Safe shutdown: return to standing, then turn torque off so the leg goes limp.
        ALWAYS runs (it's called from a 'finally' block). Each step has its own
        try/except, so one failure can't stop the steps after it."""
        print("Shutting down leg...")
        try:
            self.go_neutral(800)
        except KeyboardInterrupt:
            print("[shutdown] skipping neutral pose")  # a 2nd Ctrl+C must not skip torque-off
        except (ServoError, OSError) as e:
            print(f"[shutdown] couldn't reach neutral: {e}")
        all_off = True                                # becomes False if any servo can't be reached
        for name, (servo, _) in self.joints.items():
            try:
                if servo.is_torque_enabled():
                    servo.disable_torque()
            except (ServoError, OSError) as e:
                all_off = False
                print(f"[shutdown] couldn't disable {name} torque: {e}")
        if all_off:
            print("Leg is limp. Done.")
        else:
            # The program couldn't reach some servo, so it can't promise the leg is safe.
            print("WARNING: torque may still be ON. Switch off the servo power now!")


def main():
    """Set everything up layer by layer, then step until stopped."""
    # Layer 0: settings typos. Caught before anything moves.
    try:
        check_config()
    except ValueError as e:
        sys.exit(f"Config error in leg.py: {e}")

    # Layer 1: the serial port. pyserial raises SerialException (an OSError) if the port is wrong/busy.
    try:
        LX16A.initialize(PORT)
    except OSError as e:
        sys.exit(f"Could not open {PORT}: {e}\n(Run `ls /dev/cu.*` and check PORT)")

    # Layer 2: finding and configuring the servos. Each failure gets a specific message.
    try:
        leg = Leg()
    except ServoTimeoutError as e:
        sys.exit(f"{joint_name(e.id_)} is not responding. Check its cable, power, and ID.")
    except ServoError as e:
        sys.exit(f"Setup failed on {joint_name(e.id_)}: {type(e).__name__}: {e}")
    except OSError as e:
        sys.exit(f"Lost connection to the servo bus during setup: {e}")

    # Layer 3: the control loop. Whatever goes wrong inside 'try', the matching 'except'
    # prints why, and 'finally' ALWAYS runs the safe shutdown afterwards.
    try:
        leg.go_neutral()
        print("Walking... press Control+C to stop.")
        phase, cycle = 0.0, 0
        while True:
            leg.step(phase, STEP_DT * 1000)       # move to this loop's point in the step
            if cycle % 20 == 0:          # hardware reads are slow; check ~once per second
                leg.check_health()
            time.sleep(STEP_DT)
            phase = (phase + PHASE_STEP) % (2 * pi)   # advance around the cycle, wrapping at 2*pi
            cycle += 1
    except KeyboardInterrupt:
        print("\nStopped by user (Control+C).")
    except LegFault as e:
        print(f"[FAULT] {e}")
    except ServoArgumentError as e:
        # Clamping should prevent this, so reaching here means a bug in the code.
        print(f"[BUG] invalid value sent to {joint_name(e.id_)}: {e}")
    except ServoError as e:
        print(f"[SERVO ERROR] {joint_name(e.id_)}: {type(e).__name__}: {e}")
    except OSError as e:
        # pyserial errors are NOT ServoErrors: USB unplugged, write timeout, etc.
        print(f"[SERIAL ERROR] lost connection to the servo bus: {e}")
    finally:
        leg.shutdown()


# Only run main() when this file is started directly (python leg.py), not when
# another file imports it to read its settings (import leg).
if __name__ == "__main__":
    main()
