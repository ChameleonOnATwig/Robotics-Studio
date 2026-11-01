"""
install_update.py - copies the commented robot code into your project folder
WITHOUT losing your settings (calibration, servo IDs, port, tuning).

    python3 install_update.py                  # updates ~/robot-leg
    python3 install_update.py ~/some/folder    # updates a different folder

What it does, step by step:
  1. Backs up your whole project folder to robot-leg-backup-<date>-<time>.
  2. For each code file, takes the NEW (commented) version and copies YOUR values
     for the settings listed in KEEP into it.
  3. Checks the result is valid Python with valid settings.
  4. If anything goes wrong, puts every file back exactly as it was.
Your data files (imu_offsets.json, logs/) and your .venv are never touched.

A new person (e.g. your partner) doesn't need this: they just copy the files into
their own folder and calibrate their own leg (see README.md).
"""
import ast          # reads Python files as code, so settings are found reliably
import os
import shutil       # copying files and folders
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))     # the folder this installer is in

# The settings to carry over from your current files, per file. Anything not listed
# here comes from the new version. (The right leg's placeholders are not carried over:
# the new placeholder has the hip swing set to 0, since the hips move side to side.)
KEEP = {
    "leg.py": ["PORT", "JOINTS", "MAX_TEMP_C", "MIN_VIN_MV", "MAX_RETRIES", "STEP_DT", "PHASE_STEP"],
    "walk.py": ["STEP_TIME", "RAMP_STEPS", "HEALTH_EVERY", "LEAN", "DOUBLE_SUPPORT",
                "USE_IMU", "FALL_ANGLE", "LOG_DIR", "BALANCE", "BALANCE_DIRECTION"],
    "imu.py": ["ADDRESS", "I2C_BUS", "FORWARD_AXIS", "LEFT_AXIS", "CALIBRATION_LOOPS",
               "CALIBRATE_ACCEL", "FILTER_ALPHA"],
    "disconnect_test.py": [],
    "eyes.py": ["ROTATE", "FPS", "EYE_COLOR", "FAULT_COLOR", "TILT_WORRY", "SLEEP_AFTER"],
}
EXTRA_FILES = ["README.md", "start_eyes.sh"]   # copied as-is


def setting_values(source):
    """Find each top-level 'NAME = value' in a Python file. Returns {name: (value text, node)}."""
    tree = ast.parse(source)                  # raises SyntaxError if the file is broken
    found = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            found[node.targets[0].id] = (ast.get_source_segment(source, node.value), node.value)
    return found


def replace_value(source, name, new_value_text):
    """Swap the value of 'NAME = ...' in source for new_value_text, keeping everything else."""
    node = setting_values(source)[name][1]
    lines = source.splitlines(keepends=True)

    def position(line_no, byte_col):
        # Python reports columns in bytes; convert to a character position in the whole text.
        chars_in_line = len(lines[line_no - 1].encode()[:byte_col].decode())
        return sum(len(l) for l in lines[:line_no - 1]) + chars_in_line

    start = position(node.lineno, node.col_offset)
    end = position(node.end_lineno, node.end_col_offset)
    return source[:start] + new_value_text + source[end:]


def main():
    target = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else "~/robot-leg")
    if not os.path.isdir(target):
        sys.exit(f"Folder {target} doesn't exist. Give the path to your project folder.")
    if os.path.abspath(target) == HERE:
        sys.exit("Run this from the unzipped download folder, not from inside your project folder.")

    # ---- Step 1: back up the whole project folder (except the big .venv) ----
    backup = f"{target.rstrip('/')}-backup-{time.strftime('%Y-%m-%d-%H%M%S')}"
    shutil.copytree(target, backup, ignore=shutil.ignore_patterns(".venv", "__pycache__"))
    print(f"Backed up your folder to {backup}")

    # ---- Step 2: build every new file in memory first (nothing written yet) ----
    new_files, kept_report = {}, []
    try:
        for filename, names in KEEP.items():
            with open(os.path.join(HERE, filename)) as f:
                new_source = f.read()
            old_path = os.path.join(target, filename)
            if os.path.exists(old_path) and names:
                with open(old_path) as f:
                    old_settings = setting_values(f.read())   # SyntaxError if your file is broken
                for name in names:
                    if name in old_settings:
                        old_text = old_settings[name][0]
                        if old_text != setting_values(new_source)[name][0]:
                            new_source = replace_value(new_source, name, old_text)
                            kept_report.append(f"{filename}: {name}")
            # ---- Step 3: the result must still be valid Python ----
            ast.parse(new_source)
            new_files[filename] = new_source
    except (OSError, SyntaxError, KeyError) as e:
        sys.exit(f"Stopped before changing anything: {type(e).__name__}: {e}\n"
                 f"Your files are unchanged (and backed up in {backup}).")

    # ---- Write the files; if anything fails, restore the originals ----
    try:
        for filename, source in new_files.items():
            with open(os.path.join(target, filename), "w") as f:
                f.write(source)
        for filename in EXTRA_FILES:
            shutil.copy(os.path.join(HERE, filename), os.path.join(target, filename))
        # Check the settings make sense (same checks the robot runs before moving).
        sys.path.insert(0, target)
        import leg  # noqa: E402  (imported here, from the updated folder)
        leg.check_config()
    except ImportError as e:
        if "pylx16a" in str(e):
            print("(Couldn't run the settings check because pylx16a isn't installed in this "
                  "Python; the files are updated. Run `python walk.py left --seconds 5` to check.)")
        else:
            restore(target, backup, new_files, f"ImportError: {e}")
    except Exception as e:
        restore(target, backup, new_files, f"{type(e).__name__}: {e}")

    print("\nUpdated: " + ", ".join(list(new_files) + EXTRA_FILES))
    if kept_report:
        print("Kept your settings: " + "; ".join(kept_report))
    print(f"\nDone. If anything seems off, your old files are in {backup}")


def restore(target, backup, written, reason):
    """Put back every file this installer touched, from the backup."""
    for filename in list(written) + EXTRA_FILES:
        old = os.path.join(backup, filename)
        if os.path.exists(old):
            shutil.copy(old, os.path.join(target, filename))
        elif os.path.exists(os.path.join(target, filename)):
            os.remove(os.path.join(target, filename))
    sys.exit(f"Something went wrong ({reason}), so all your files were put back as they were.")


if __name__ == "__main__":
    main()
