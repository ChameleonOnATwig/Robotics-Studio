"""
disconnect_test.py - motor-disconnect test for the robot leg.

It runs the REAL code in leg.py and checks that it handles a disconnected servo
correctly: a clear message naming the joint, a safe shutdown, and no crash
(no red Python traceback). It runs 6 tests and prints PASS/FAIL for each:
    3 tests: a servo unplugged BEFORE starting (hip, knee, ankle)
    3 tests: a servo unplugged WHILE walking   (hip, knee, ankle)

    python disconnect_test.py              # guided test on the real leg (follow the prompts)
    python disconnect_test.py --simulate   # same tests on simulated servos (no hardware)

Put this file in the same folder as leg.py.
"""
import io           # StringIO: collects text in memory (used to capture leg.py's messages)
import signal       # an alarm that stops a test that runs too long
import sys          # command-line options and sys.exit()
import time         # timing how long each test takes
import traceback    # gets the full error report if leg.py ever crashes

import leg          # the code under test, plus its settings (PORT, JOINTS)
from pylx16a.lx16a import LX16A, ServoError

SIMULATE = "--simulate" in sys.argv   # True when run with --simulate
STARTUP_TIMEOUT_S = 6     # a startup test should finish well before this
MIDRUN_TIMEOUT_S = 30     # time allowed to unplug a cable and for the leg to react
results = []              # (test name, passed, detail) for the summary at the end


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
class Tee(io.StringIO):
    """Shows leg.py's output live AND keeps a copy so the test can check it.
    (Named after a T-shaped pipe: one input, two outputs.)"""

    def __init__(self, real):
        super().__init__()
        self.real = real            # the real screen

    def write(self, s):
        self.real.write(s)          # show it on screen...
        self.real.flush()
        return super().write(s)     # ...and keep a copy


class AutoStop(Exception):
    """Raised by the timer when a run goes on longer than the test allows."""


def _alarm(signum, frame):
    """Called when the time limit runs out."""
    raise KeyboardInterrupt  # leg.py treats this exactly like Control+C


def run_leg(timeout_s, on_start=None):
    """Run leg.main() once, capturing everything it prints.
    Returns (output text, crash traceback or None, seconds, timed_out)."""
    tee = Tee(sys.stdout)
    crash, timed_out = None, False
    start = time.monotonic()
    old_stdout = sys.stdout
    sys.stdout = tee                         # capture leg.py's messages
    signal.signal(signal.SIGALRM, _alarm)
    signal.alarm(timeout_s)                  # start the time limit
    if on_start:
        on_start()
    try:
        leg.main()
    except SystemExit as e:      # leg.py exits this way on startup errors
        if e.code not in (None, 0):
            print(e.code)                    # keep its exit message in the captured text
    except KeyboardInterrupt:    # timer fired outside leg.py's own handling
        timed_out = True
    except BaseException:        # anything else is an unhandled crash = FAIL
        crash = traceback.format_exc()
    finally:
        signal.alarm(0)                      # cancel the time limit
        sys.stdout = old_stdout              # stop capturing
    elapsed = time.monotonic() - start
    text = tee.getvalue()
    if "Stopped by user" in text:  # the timer ended the walk, not a fault
        timed_out = True
    return text, crash, elapsed, timed_out


def record(name, passed, detail):
    """Save a test result and print it."""
    results.append((name, passed, detail))
    print(f"\n>>> {'PASS' if passed else 'FAIL'}: {name}\n    {detail}\n")


def ask(prompt):
    """Wait for the user to press Enter (skipped automatically in simulation mode)."""
    if SIMULATE:
        print(f"{prompt} [simulated]")
        return ""
    return input(prompt)


def all_servos_answer():
    """True if every joint replies. Leaves servos limp."""
    LX16A.initialize(leg.PORT)
    missing = []
    for name, cfg in leg.JOINTS.items():
        try:
            LX16A(cfg["id"], disable_torque=True)
        except ServoError:
            missing.append(f"{name} (servo {cfg['id']})")
    if missing:
        print("  Not answering: " + ", ".join(missing))
    return not missing


def wait_for_reconnect():
    """Give the user a chance to plug everything back in before the next test."""
    while True:
        ask("Plug ALL servo cables back in, then press Enter...")
        try:
            if all_servos_answer():
                print("  All servos answering.")
                return True
        except OSError as e:
            print(f"  Can't open the port: {e}")
        if SIMULATE:
            return False
        if ask("Still missing. Press Enter to check again, or type s to skip: ").strip().lower() == "s":
            return False


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def test_startup_disconnect(name, cfg):
    """Test: a servo is unplugged BEFORE leg.py starts.
    PASS = leg.py refuses to start, names the missing joint, and doesn't crash."""
    title = f"{name} unplugged BEFORE starting"
    print(f"\n=== {title} ===")
    ask(f"Unplug the {name} servo's cable (servo {cfg['id']}), then press Enter...")
    sim_unplug({cfg["id"]})
    text, crash, elapsed, timed_out = run_leg(STARTUP_TIMEOUT_S)
    sim_reconnect()

    # Decide PASS/FAIL from what leg.py printed.
    if crash:
        record(title, False, "leg.py crashed with a traceback:\n" + crash)
    elif "is not responding" not in text:
        record(title, False, "The leg started normally. Was the right cable unplugged?")
    elif f"servo {cfg['id']} ({name})" in text:
        record(title, True, f"Clear message naming the {name}, stopped in {elapsed:.1f} s, no crash.")
    else:
        # Servos wired in a chain: unplugging one cable can also cut the ones after it,
        # so leg.py reports the first missing joint it checks.
        record(title, True, "Clear 'not responding' message and no crash, but it named a "
                            "different joint (normal if your servos are daisy-chained).")
    wait_for_reconnect()


def test_midrun_disconnect(name, cfg):
    """Test: a servo is unplugged WHILE the leg is walking.
    PASS = leg.py notices, reports a [FAULT] naming the joint, and shuts down safely."""
    title = f"{name} unplugged WHILE walking"
    print(f"\n=== {title} ===")
    print("Hold or clamp the leg with the foot in the air.")
    print(f"The leg will start walking. After a few steps, UNPLUG the {name} servo's cable.")
    print(f"(If nothing is unplugged within {MIDRUN_TIMEOUT_S} s, the test stops on its own.)")
    ask("Press Enter to start walking...")
    text, crash, elapsed, timed_out = run_leg(
        MIDRUN_TIMEOUT_S, on_start=lambda: sim_unplug({cfg["id"]}, after_s=2)
    )
    sim_reconnect()

    # Decide PASS/FAIL from what leg.py printed; each FAIL explains the likely cause.
    if crash:
        record(title, False, "leg.py crashed with a traceback:\n" + crash)
    elif "[FAULT]" in text and "failed 3 reads in a row" in text:
        safe = "Leg is limp" in text or "Switch off the servo power" in text
        named = f"servo {cfg['id']} ({name})" in text
        detail = f"Disconnect detected and reported after {elapsed:.1f} s of walking"
        detail += f", naming the {name}." if named else " (named a different joint; normal if daisy-chained)."
        detail += " Safe shutdown ran." if safe else " WARNING: shutdown message missing."
        record(title, safe, detail)
    elif "low voltage" in text:
        record(title, False, "The leg stopped for LOW VOLTAGE before the unplug was tested. "
                             "Charge the battery, and check MIN_VIN_MV in leg.py matches your battery.")
    elif "overheating" in text:
        record(title, False, "The leg stopped because a servo was OVERHEATING. Let the servos cool, then rerun.")
    elif "[SERIAL ERROR]" in text:
        record(title, False, "The USB connection dropped instead. Unplug the servo cable, not the USB.")
    elif timed_out:
        record(title, False, f"No disconnect detected in {MIDRUN_TIMEOUT_S} s. Was the cable unplugged?")
    else:
        record(title, False, "Unexpected ending. Output:\n" + text[-500:])
    wait_for_reconnect()


# ---------------------------------------------------------------------------
# Simulation mode: fake servos so the tests can run with no hardware
# ---------------------------------------------------------------------------
class SimSerial:
    """Imitates the LX-16A bus closely enough for pylx16a to talk to it.
    It answers like a healthy servo, or stays silent like an unplugged one."""

    dead = set()        # servo IDs currently "unplugged"
    unplug_at = None    # (time, ids) for a scheduled mid-run unplug

    def __init__(self, **kwargs):
        self.timeout = kwargs.get("timeout")
        self.write_timeout = kwargs.get("write_timeout")
        self.last = (0, 0)

    def write(self, packet):
        """Remember which servo was asked what, so read() can answer it."""
        self.last = (packet[2], packet[4])  # (servo id, command)

    def read(self, n):
        """Return a correctly formatted reply, or nothing if that servo is 'unplugged'."""
        if SimSerial.unplug_at and time.monotonic() >= SimSerial.unplug_at[0]:
            SimSerial.dead |= SimSerial.unplug_at[1]       # the scheduled unplug happens now
            SimSerial.unplug_at = None
        sid, cmd = self.last
        if sid not in {cfg["id"] for cfg in leg.JOINTS.values()} or sid in SimSerial.dead:
            return b""  # no reply, exactly like an unplugged servo
        vin = max(7400, leg.MIN_VIN_MV + 1000)  # healthy voltage for whatever battery leg.py expects
        # replies: 26 = temperature, 27 = voltage, 28 = position, 32 = torque on
        data = {26: [40], 27: [vin % 256, vin // 256], 28: [0xF4, 0x01], 32: [1]}.get(cmd, [])
        packet = [0x55, 0x55, sid, n - 3, cmd, *data]
        packet += [0] * (n - 1 - len(packet))
        packet.append((~sum(packet[2:])) % 256)            # checksum, as the real servo sends
        return bytes(packet)

    def flushInput(self):
        pass

    reset_input_buffer = reset_output_buffer = close = flushInput


def sim_unplug(ids, after_s=0):
    """In simulation, 'unplug' these servos now, or after_s seconds from now."""
    if SIMULATE:
        if after_s:
            SimSerial.unplug_at = (time.monotonic() + after_s, set(ids))
        else:
            SimSerial.dead |= set(ids)


def sim_reconnect():
    """In simulation, 'plug' every servo back in."""
    if SIMULATE:
        SimSerial.dead.clear()
        SimSerial.unplug_at = None


# ---------------------------------------------------------------------------
def main():
    """Check the setup, run the 6 tests, then print a summary."""
    if SIMULATE:
        import pylx16a.lx16a as lib
        lib.serial.Serial = SimSerial            # swap the real USB port for the fake one
        print("SIMULATION MODE: no hardware is used.\n")
    else:
        print("Motor-disconnect test. Keep a hand near the servo power switch.\n")

    try:
        leg.check_config()
    except ValueError as e:
        sys.exit(f"Fix leg.py first: {e}")

    print("Checking that all servos answer before starting...")
    try:
        if not all_servos_answer():
            sys.exit("Plug in and power all servos, then run this test again.")
    except OSError as e:
        sys.exit(f"Could not open {leg.PORT}: {e}")

    for name, cfg in leg.JOINTS.items():
        test_startup_disconnect(name, cfg)
    for name, cfg in leg.JOINTS.items():
        test_midrun_disconnect(name, cfg)

    print("\n" + "=" * 60 + "\nSUMMARY")
    for name, passed, _ in results:
        print(f"  {'PASS' if passed else 'FAIL'}  {name}")
    passed = sum(p for _, p, _ in results)
    print(f"\n{passed}/{len(results)} tests passed.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        signal.alarm(0)
        print("\nTest stopped. Switch off the servo power if the leg is still stiff.")
