"""
imu.py - MPU6050 IMU (tilt sensor) support for the walking robot, on a Raspberry Pi.

An IMU ("inertial measurement unit") measures two things:
  - acceleration (accelerometer): mostly gravity, so it shows which way is DOWN
  - rotation speed (gyroscope):   how fast the robot is turning
This file turns those into PITCH (leaning forward/back) and ROLL (leaning left/right),
which walk.py uses for balance, fall protection and data recording.

HOW TO RUN (on the Pi, inside the project folder):
    python imu.py              # live tilt readout: check the wiring and the axis directions
    python imu.py --zero       # one-time calibration, IMU flat and still: saves imu_offsets.json
    python imu.py --simulate   # the same with a simulated IMU (works on a Mac, no hardware)

BASED ON the ElectronicCats MPU6050 Arduino library (github.com/ElectronicCats/mpu6050,
MIT license, itself based on jrowberg/i2cdevlib), translated to Python for the Pi:
    initialize()                    -> MPU6050.__init__   (clock source, ranges, wake up)
    testConnection()/getDeviceID()  -> MPU6050.__init__   (device ID check)
    getMotion6()                    -> MPU6050.read()
    CalibrateGyro()/CalibrateAccel()/PID() -> MPU6050.calibrate_gyro()/calibrate_accel()/_pid()
    GetActiveOffsets()              -> MPU6050.active_offsets()
The library's DMP (on-chip motion processor) is not used; tilt is computed by TiltSensor.

WIRING (MPU6050 -> Raspberry Pi header pins), with the Pi switched OFF:
    VCC -> pin 1 (3.3V)       GND -> pin 6 (GND)
    SDA -> pin 3 (GPIO 2)     SCL -> pin 5 (GPIO 3)

ONE-TIME SETUP on the Pi:
    sudo raspi-config nonint do_i2c 0     # turn on I2C, then: sudo reboot
    pip install smbus2                     # inside the project's .venv
    sudo apt install -y i2c-tools && i2cdetect -y 1    # should show 68

Mount the IMU on the BODY (not a leg), flat, chip side up, near the middle.

SIGN CONVENTIONS used everywhere in the robot code:
    pitch > 0  : leaning FORWARD (nose down)        pitch < 0 : leaning back
    roll  > 0  : leaning to the RIGHT               roll  < 0 : leaning left

EXCEPTION HANDLING: every IMU problem (missing, unplugged, frozen, can't calibrate)
raises IMUError with a plain-English reason. walk.py decides what to do with it:
stop safely if it's balancing, or carry on without data if it isn't.
"""
import json      # saves/loads the calibration offsets file
import math      # trigonometry for turning gravity into tilt angles
import os        # file paths
import random    # noise for the simulated IMU
import sys       # command-line options and sys.exit()
import time      # pauses and timing

# =====================================================================================
# SETTINGS
# =====================================================================================

ADDRESS = 0x68   # the chip's I2C address; 0x69 if the board's AD0 pin is connected to 3.3V
I2C_BUS = 1      # the Pi's header I2C bus (pins 3 and 5)

# How the IMU is mounted: which of its axes (printed on the board) points FORWARD on the
# robot, and which points to the robot's LEFT. Use "x", "-x", "y", "-y".
# If `python imu.py` shows pitch going NEGATIVE when you tip the robot forward, or the
# pitch and roll swapped, change these two lines.
FORWARD_AXIS = "x"
LEFT_AXIS = "y"

CALIBRATION_LOOPS = 6    # rounds of on-chip calibration, as in the library's DMP6 example
# Accelerometer calibration makes the CURRENT position read as perfectly level (it assumes
# the chip is flat, chip side up, while it runs). Leave False to measure true tilt; set True
# if the IMU is mounted slightly crooked and the robot is level at startup.
CALIBRATE_ACCEL = False
CALIBRATION_MAX_TRIES = 2000   # safety limit: the library's loop can run forever if the robot moves
# Saved by `python imu.py --zero` and loaded at every startup, like the library's workflow of
# running IMU_Zero once and then setting those offsets with setXAccelOffset() etc.
OFFSETS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "imu_offsets.json")

FILTER_ALPHA = 0.98   # complementary filter: how much to trust the gyro vs the accelerometer
READ_RETRIES = 3      # retries for a failed I2C read before giving up
FROZEN_LIMIT = 40     # identical readings in a row = the IMU has stopped updating

# Each axis name as a direction (x, y, z). "-x" points the opposite way to "x".
_AXES = {"x": (1, 0, 0), "-x": (-1, 0, 0), "y": (0, 1, 0), "-y": (0, -1, 0)}


class IMUError(Exception):
    """Our own exception: the IMU is missing, disconnected, or giving readings that
    can't be trusted. The message always says what to check."""


def _dot(a, b):
    """Dot product: how much of vector a points along direction b."""
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    """Cross product: the direction at right angles to both a and b.
    Used to work out which way is UP from FORWARD and LEFT."""
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


# =====================================================================================
# THE REAL CHIP
# =====================================================================================

class MPU6050:
    """Talks to the real chip over I2C (two wires: SDA data, SCL clock).
    read() returns (accel in g, gyro in deg/s), each as (x, y, z)."""

    # Register addresses: numbered memory slots inside the chip (same names as MPU6050.h).
    RA_XA_OFFS_H = 0x06      # accelerometer offsets (MPU6050)
    RA_XG_OFFS_USRH = 0x13   # gyro offsets
    RA_ACCEL_XOUT_H = 0x3B   # first byte of the accelerometer readings
    RA_GYRO_XOUT_H = 0x43    # first byte of the gyro readings
    RA_PWR_MGMT_1 = 0x6B     # power and clock settings
    RA_WHO_AM_I = 0x75       # holds the chip's ID number
    CLOCK_PLL_XGYRO = 0x01   # "use the X gyro as the timing source" (more stable)

    def __init__(self, bus=I2C_BUS, address=ADDRESS):
        """Connect to the chip, check it's there, and set it up (like the library's
        initialize() + testConnection()). Raises IMUError if anything is missing."""
        try:
            from smbus2 import SMBus                   # the Pi's I2C library
        except ImportError:
            raise IMUError("the smbus2 library isn't installed. On the Pi run: pip install smbus2")
        try:
            self.bus = SMBus(bus)
        except (FileNotFoundError, PermissionError, OSError) as e:
            raise IMUError(f"I2C bus {bus} isn't available ({e}). On the Pi, turn on I2C with "
                           "`sudo raspi-config nonint do_i2c 0` and reboot. (A Mac has no I2C bus.)")
        self.address = address
        try:
            # testConnection(): getDeviceID() is bits 6..1 of WHO_AM_I; a genuine MPU6050 gives 0x34
            self.device_id = (self.bus.read_byte_data(address, self.RA_WHO_AM_I) >> 1) & 0x3F
            # initialize(): gyro clock as timing source + wake up, gyro +/-250 deg/s, accel +/-2 g
            self.bus.write_byte_data(address, self.RA_PWR_MGMT_1, self.CLOCK_PLL_XGYRO)
            time.sleep(0.05)                           # give it a moment to wake up
            self.bus.write_byte_data(address, 0x1B, 0x00)        # gyro range +/-250 deg/s
            self.bus.write_byte_data(address, 0x1C, 0x00)        # accel range +/-2 g
            self.bus.write_byte_data(address, 0x19, 4)           # sample rate 200 Hz
            self.bus.write_byte_data(address, 0x1A, 3)           # low-pass filter ~44 Hz (less vibration noise)
        except OSError:
            # OSError here means nothing answered at that address: wiring problem.
            raise IMUError(f"no reply from the MPU6050 at address 0x{address:02x}. Check its 4 wires "
                           "and run `i2cdetect -y 1` (it should show 68).")
        if self.device_id != 0x34:
            print(f"[imu] note: device ID 0x{self.device_id:02x} (a genuine MPU6050 is 0x34); "
                  "continuing, many clone boards differ.")
        # the library: chips with ID 0x38 and up keep accel offsets at 0x77, 3 bytes apart
        self.accel_offset_reg, self.accel_offset_step = (
            (self.RA_XA_OFFS_H, 2) if self.device_id < 0x38 else (0x77, 3))

    # ---------- low-level 16-bit register access (the library's I2Cdev::readWords/writeWords) ----------
    def _read_word(self, reg):
        """Read a 16-bit signed number stored in two registers (high byte first)."""
        hi, lo = self.bus.read_i2c_block_data(self.address, reg, 2)
        v = (hi << 8) | lo                          # join the two bytes
        return v - 65536 if v > 32767 else v        # convert to a negative number if needed

    def _write_word(self, reg, value):
        """Write a 16-bit signed number into two registers (high byte first)."""
        value = max(-32768, min(32767, int(value))) & 0xFFFF   # keep it in range, as 2 bytes
        self.bus.write_i2c_block_data(self.address, reg, [value >> 8, value & 0xFF])

    # ---------- calibration: a Python port of the library's CalibrateGyro/CalibrateAccel/PID ----------
    @staticmethod
    def _loop_scale(loops):
        """How strongly to correct, depending on the number of loops (copied from the library)."""
        # the library's: x = (100 - map(Loops, 1, 5, 20, 0)) * .01   (Arduino integer map)
        mapped = 20 + (loops - 1) * (0 - 20) // (5 - 1)
        return (100 - mapped) * 0.01

    def calibrate_gyro(self, loops=CALIBRATION_LOOPS):
        """Make the gyro read zero while still. The robot must be STILL."""
        x = self._loop_scale(loops)
        self._pid(self.RA_GYRO_XOUT_H, 0.3 * x, 90 * x, loops)

    def calibrate_accel(self, loops=CALIBRATION_LOOPS):
        """Make the accelerometer read 0, 0, +1 g (level). The IMU must be FLAT and STILL."""
        x = self._loop_scale(loops)
        self._pid(self.RA_ACCEL_XOUT_H, 0.3 * x, 20 * x, loops)

    def _pid(self, read_address, kP, kI, loops):
        """Adjust the chip's own offset registers until the readings are zero (gyro) or
        0/0/+1 g (accel), using a PI feedback loop. The robot must be STILL.

        How it works: read the sensor, see how far it is from the ideal value (the
        'error'), and nudge the chip's offset to cancel it. P reacts to the error now;
        I builds up over time to remove what's left. Each loop uses gentler nudges
        (kP, kI shrink by 25%), so the offsets settle precisely."""
        is_accel = read_address == self.RA_ACCEL_XOUT_H
        if is_accel:
            save_address, step = self.accel_offset_reg, self.accel_offset_step
        else:
            save_address, step = self.RA_XG_OFFS_USRH, 2
        scale = 8 if is_accel else 4          # offset register units vs reading units
        gravity = 16384                        # 1 g at the +/-2 g range
        try:
            # Start from the offsets already in the chip.
            bit_zero, i_term = [], []
            for i in range(3):                 # x, y, z
                data = self._read_word(save_address + i * step)
                bit_zero.append(data & 1)      # accel offsets: bit 0 is reserved, keep it
                i_term.append(float(data) * scale)
            for _ in range(loops):
                good_samples = 0               # readings in a row that were close enough
                c = tries = 0
                while c < 100:                 # up to 100 PI updates per loop
                    tries += 1
                    if tries > CALIBRATION_MAX_TRIES:
                        # Our addition: the library's loop never gives up if the robot moves.
                        raise IMUError("calibration couldn't settle. Keep the robot completely "
                                       "still while it starts up, then try again.")
                    e_sum = 0                  # total error across x, y, z this update
                    for i in range(3):
                        reading = float(self._read_word(read_address + i * 2))
                        if is_accel and i == 2:
                            reading -= gravity  # Z should read +1 g, not 0
                        error = -reading
                        e_sum += abs(reading)
                        p_term = kP * error
                        i_term[i] += error * 0.001 * kI
                        out = round((p_term + i_term[i]) / scale)
                        if is_accel:
                            out = (out & ~1) | bit_zero[i]   # put the reserved bit back
                        self._write_word(save_address + i * step, out)
                    if c == 99 and e_sum > 1000:   # still far off: run this loop again
                        c = 0
                        continue
                    if e_sum * (0.05 if is_accel else 1) < 5:
                        good_samples += 1
                    if e_sum < 100 and c > 10 and good_samples >= 10:
                        break                  # settled: on to the next, finer loop
                    time.sleep(0.001)
                    c += 1
                kP *= 0.75                     # gentler nudges next loop
                kI *= 0.75
                # Store the settled offsets (the I part) at the end of each loop.
                for i in range(3):
                    out = round(i_term[i] / scale)
                    if is_accel:
                        out = (out & ~1) | bit_zero[i]
                    self._write_word(save_address + i * step, out)
        except OSError as e:
            raise IMUError(f"lost contact with the IMU during calibration ({e}). Check its wires.") from e

    def set_accel_offsets(self, offsets):
        """Like the library's setX/Y/ZAccelOffset(): write saved accel offsets into the chip."""
        try:
            for i, value in enumerate(offsets):
                self._write_word(self.accel_offset_reg + i * self.accel_offset_step, value)
        except OSError as e:
            raise IMUError(f"lost contact with the IMU while loading offsets ({e}).") from e

    def active_offsets(self):
        """The offsets now stored in the chip: (accel x, y, z, gyro x, y, z). Like GetActiveOffsets()."""
        accel = [self._read_word(self.accel_offset_reg + i * self.accel_offset_step) for i in range(3)]
        gyro = [self._read_word(self.RA_XG_OFFS_USRH + i * 2) for i in range(3)]
        return (*accel, *gyro)

    def read(self):
        """Read all six measurements at once (like the library's getMotion6()).
        Retries a failed read a few times before deciding the IMU is gone."""
        for attempt in range(1, READ_RETRIES + 1):
            try:
                # 14 bytes: accel x,y,z, temperature, gyro x,y,z (2 bytes each)
                d = self.bus.read_i2c_block_data(self.address, 0x3B, 14)
                break                                   # success: stop retrying
            except OSError as e:
                if attempt == READ_RETRIES:
                    raise IMUError(f"lost contact with the IMU ({e}). Check its wires.") from e
                time.sleep(0.002)
        v = [((d[i] << 8) | d[i + 1]) for i in range(0, 14, 2)]   # join byte pairs
        v = [x - 65536 if x > 32767 else x for x in v]           # make them signed
        accel = (v[0] / 16384.0, v[1] / 16384.0, v[2] / 16384.0)  # 16384 per g at +/-2 g
        gyro = (v[4] / 131.0, v[5] / 131.0, v[6] / 131.0)          # 131 per deg/s at +/-250
        return accel, gyro                                         # (v[3] is temperature; unused)

    def close(self):
        """Release the I2C bus (safe to call even if it's already broken)."""
        try:
            self.bus.close()
        except OSError:
            pass


# =====================================================================================
# A PRETEND CHIP, FOR TESTING WITHOUT HARDWARE
# =====================================================================================

class SimulatedIMU:
    """Pretend IMU for testing on a Mac: gently rocks back and forth, with sensor noise.
    Tests can set .pitch/.roll directly, .fail = True to act unplugged, or .freeze = True."""

    def __init__(self, rocking_deg=2.0):
        self.rocking = rocking_deg      # how far it pretends to rock, in degrees
        self.pitch = self.roll = None   # set these to hold a fixed tilt (degrees)
        self.fail = self.freeze = False
        self.start = time.monotonic()
        self._last = None

    def read(self):
        """Return made-up readings matching the pretend tilt, in the same format as MPU6050.read()."""
        if self.fail:
            raise IMUError("lost contact with the IMU (simulated unplug).")
        if self.freeze and self._last:
            return self._last
        t = time.monotonic() - self.start
        p = self.pitch if self.pitch is not None else self.rocking * math.sin(2 * math.pi * t / 3.0)
        r = self.roll if self.roll is not None else 0.5 * self.rocking * math.sin(2 * math.pi * t / 6.0)
        pr, rr = math.radians(p), math.radians(r)
        n = lambda: random.gauss(0, 0.01)       # a little random sensor noise
        # gravity as an accelerometer on a body with x forward, y left, z up would see it
        accel = (-math.sin(pr) + n(), math.cos(pr) * math.sin(rr) + n(), math.cos(pr) * math.cos(rr) + n())
        gyro = (random.gauss(0, 0.3), random.gauss(0, 0.3), random.gauss(0, 0.3))
        self._last = (accel, gyro)
        return self._last

    # The same methods as the real chip, doing nothing, so the rest of the code works unchanged.
    def calibrate_gyro(self, loops=CALIBRATION_LOOPS):
        pass   # the simulated sensor has no offsets to correct

    def calibrate_accel(self, loops=CALIBRATION_LOOPS):
        pass

    def set_accel_offsets(self, offsets):
        pass

    def active_offsets(self):
        return (0, 0, 0, 0, 0, 0)

    def close(self):
        pass


# =====================================================================================
# TURNING READINGS INTO TILT
# =====================================================================================

class TiltSensor:
    """Turns raw IMU readings into pitch and roll (degrees) with a complementary filter:
    the gyro is smooth but drifts, the accelerometer doesn't drift but is shaken by every
    footstep, so the filter blends the two."""

    def __init__(self, device):
        """device is an MPU6050 or a SimulatedIMU."""
        if FORWARD_AXIS.strip("-") == LEFT_AXIS.strip("-"):
            raise IMUError("FORWARD_AXIS and LEFT_AXIS in imu.py must be different axes.")
        self.dev = device
        self.fwd = _AXES[FORWARD_AXIS]           # robot's forward, in the chip's axes
        self.left = _AXES[LEFT_AXIS]             # robot's left, in the chip's axes
        self.up = _cross(self.fwd, self.left)    # up is at right angles to both
        self.bias = (0.0, 0.0, 0.0)              # leftover gyro error, measured in calibrate()
        self.pitch = self.roll = 0.0             # current tilt, degrees
        self.pitch_rate = self.roll_rate = 0.0   # how fast it's tilting, degrees per second
        self.accel = self.gyro = (0.0, 0.0, 0.0) # latest readings (saved in the data log)
        self._last_time = None
        self._last_raw = None
        self._same_count = 0                     # for spotting a frozen sensor

    def calibrate(self, samples=100):
        """Measure the gyro's resting offset. The robot must be STILL while this runs (~0.5 s).
        Returns False if it seemed to be moving."""
        readings = []
        for _ in range(samples):
            readings.append(self.dev.read()[1])  # gyro part only
            time.sleep(0.005)
        self.bias = tuple(sum(r[i] for r in readings) / samples for i in range(3))   # average
        spread = max(max(r[i] for r in readings) - min(r[i] for r in readings) for i in range(3))
        accel, _ = self.dev.read()
        self.pitch, self.roll = self._accel_angles(accel)    # start from the gravity-based tilt
        self._last_time = time.monotonic()
        return spread < 10  # False = it was moving during calibration

    def _accel_angles(self, accel):
        """Tilt from gravity alone: whichever way gravity pulls tells us which way is down."""
        a_fwd, a_left, a_up = _dot(accel, self.fwd), _dot(accel, self.left), _dot(accel, self.up)
        if abs(a_fwd) + abs(a_left) + abs(a_up) < 0.2:
            # Gravity is always there, so near-zero readings mean the sensor isn't working.
            raise IMUError("IMU accelerometer reads almost zero; the sensor isn't working properly.")
        pitch = math.degrees(math.atan2(-a_fwd, math.hypot(a_left, a_up)))
        roll = math.degrees(math.atan2(a_left, a_up))
        return pitch, roll

    def update(self):
        """Take a new reading and update pitch and roll. Call this every loop.
        Raises IMUError if the sensor is unplugged or frozen."""
        accel, gyro = self.dev.read()
        # A real sensor's readings always jiggle slightly; many identical ones in a row
        # mean the chip has stopped updating.
        if (accel, gyro) == self._last_raw:
            self._same_count += 1
            if self._same_count >= FROZEN_LIMIT:
                raise IMUError("IMU readings stopped changing; the sensor has frozen.")
        else:
            self._same_count = 0
        self._last_raw = (accel, gyro)

        now = time.monotonic()
        dt = min(max(now - (self._last_time or now), 0.0), 0.2)   # seconds since last update (max 0.2)
        self._last_time = now
        g = tuple(gyro[i] - self.bias[i] for i in range(3))      # gyro with its offset removed
        # nose-down rotation is about the LEFT axis; right-lean rotation is about the FORWARD axis
        self.pitch_rate = _dot(g, self.left)
        self.roll_rate = _dot(g, self.fwd)
        acc_pitch, acc_roll = self._accel_angles(accel)
        # Complementary filter: 98% "previous angle + gyro rotation since then" (smooth),
        # 2% "angle from gravity" (stops the gyro drifting away over time).
        a = FILTER_ALPHA
        self.pitch = a * (self.pitch + self.pitch_rate * dt) + (1 - a) * acc_pitch
        self.roll = a * (self.roll + self.roll_rate * dt) + (1 - a) * acc_roll
        self.accel, self.gyro = accel, g
        return self.pitch, self.roll


# =====================================================================================
# STARTING THE IMU
# =====================================================================================

def load_saved_offsets():
    """Accel offsets from imu_offsets.json, or None. A missing or damaged file only means
    slightly less accurate tilt, so it gives a note instead of stopping the robot."""
    if not os.path.exists(OFFSETS_FILE):
        print("[imu] note: no saved offsets yet. For best accuracy run `python imu.py --zero` "
              "once with the IMU flat and still.")
        return None
    try:
        with open(OFFSETS_FILE) as f:
            accel = json.load(f)["accel"]
        # Check the file holds exactly 3 whole numbers in the chip's range.
        if len(accel) != 3 or not all(isinstance(v, int) and -32768 <= v <= 32767 for v in accel):
            raise ValueError("expected 3 whole numbers")
        return accel
    except (OSError, ValueError, KeyError, TypeError) as e:
        print(f"[imu] warning: couldn't use {os.path.basename(OFFSETS_FILE)} ({e}); ignoring it. "
              "Run `python imu.py --zero` again to remake it.")
        return None


def open_imu(simulate=False):
    """Create, check and calibrate the IMU. Raises IMUError with a clear reason if it can't.
    This is what walk.py calls at startup."""
    device = SimulatedIMU() if simulate else MPU6050()
    # Accelerometer: use the offsets saved by `python imu.py --zero` (measured once with the
    # IMU level), or calibrate now if CALIBRATE_ACCEL is set.
    if CALIBRATE_ACCEL:
        device.calibrate_accel(CALIBRATION_LOOPS)
    elif not simulate:
        saved = load_saved_offsets()
        if saved:
            device.set_accel_offsets(saved)
    # Gyro: calibrated at every startup (its offset drifts with temperature), robot still.
    device.calibrate_gyro(CALIBRATION_LOOPS)
    sensor = TiltSensor(device)
    if not sensor.calibrate():
        print("[imu] warning: the robot seemed to move during gyro calibration. "
              "Keep it still at startup for best results.")
    return sensor


def main():
    """Command-line tool: `--zero` calibrates and saves offsets; otherwise show live tilt."""
    simulate = "--simulate" in sys.argv
    if "--zero" in sys.argv:
        # like the library's IMU_Zero example: calibrate and print the offsets found
        try:
            device = SimulatedIMU() if simulate else MPU6050()
            print("Keep the IMU flat (chip side up) and completely still...")
            device.calibrate_accel(CALIBRATION_LOOPS)
            device.calibrate_gyro(CALIBRATION_LOOPS)
            offsets = device.active_offsets()
            names = ("accel X", "accel Y", "accel Z", "gyro X", "gyro Y", "gyro Z")
            for name, value in zip(names, offsets):
                print(f"  {name} offset: {value:6d}")
            device.close()
            if simulate:
                return                       # nothing real to save
            try:
                with open(OFFSETS_FILE, "w") as f:
                    json.dump({"accel": list(offsets[:3]), "gyro": list(offsets[3:])}, f)
                print(f"Saved to {os.path.basename(OFFSETS_FILE)}; loaded automatically from now on.")
            except OSError as e:
                print(f"Couldn't save the offsets ({e}). Write the numbers above down.")
            return
        except IMUError as e:
            sys.exit(f"IMU problem: {e}")
    # Live readout, to check the wiring and which way the axes point.
    try:
        sensor = open_imu(simulate)
    except IMUError as e:
        sys.exit(f"IMU problem: {e}")
    print("Tilt the robot by hand to check the directions:")
    print("  lean it FORWARD -> pitch should go POSITIVE")
    print("  lean it RIGHT   -> roll should go POSITIVE")
    print("If not, change FORWARD_AXIS / LEFT_AXIS at the top of imu.py. Control+C to stop.\n")
    try:
        while True:
            pitch, roll = sensor.update()
            print(f"\r  pitch {pitch:+6.1f} deg   roll {roll:+6.1f} deg   ", end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\ndone")
    except IMUError as e:
        print(f"\nIMU problem: {e}")
    finally:
        sensor.dev.close()


# Only run main() when this file is started directly (python imu.py), not when
# walk.py imports it.
if __name__ == "__main__":
    main()
