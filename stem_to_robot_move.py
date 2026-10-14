#!/usr/bin/env python3
import argparse
import importlib.util
import sys
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
import serial

ROOT = Path(__file__).resolve().parent
PYORBBECSDK_ROOT = ROOT / "pyorbbecsdk"

for candidate in [str(ROOT), str(PYORBBECSDK_ROOT)]:
    if candidate not in sys.path:
        sys.path.insert(0, candidate)

try:
    from pyorbbecsdk.examples.utils import frame_to_bgr_image
except ModuleNotFoundError:
    utils_path = PYORBBECSDK_ROOT / "examples" / "utils.py"
    if utils_path.exists():
        spec = importlib.util.spec_from_file_location("local_examples_utils", utils_path)
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        frame_to_bgr_image = module.frame_to_bgr_image
    else:
        raise

from pyorbbecsdk import AlignFilter, Config, OBSensorType, OBStreamType, Pipeline

try:
    from pyorbbecsdk.mustard_ai_model.detect_mustard import MustardDetector
except ModuleNotFoundError:
    from mustard_ai_model.detect_mustard import MustardDetector

# --- load camera-to-robot calibration transform ---
# NOTE: redo this file after the joint calibration (CAL) below is finished,
# because the robot-side touch points depend on fk().
CALIB_PATH = ROOT / "camera_to_robot_transform.json"
if not CALIB_PATH.exists():
    CALIB_PATH = Path("camera_to_robot_transform.json")
if CALIB_PATH.exists():
    with open(CALIB_PATH, "r") as f:
        calib = json.load(f)
    R = np.array(calib["R"], dtype=np.float64)
    t = np.array(calib["t"], dtype=np.float64)
else:
    # Allows --check-pose / --verify-fk / calibrate_camera_to_robot.py to run before
    # the camera calibration exists. Detection mode will refuse to move the arm.
    print(f"WARNING: {CALIB_PATH.name} not found; camera-to-robot transform not loaded.")
    calib = None
    R = np.eye(3)
    t = np.zeros(3)

# --- arm geometry (mm, axis center to axis center) ---
# Measured 2026-10-06 in pose MOVE 10 50 110 90 15 150 (plumb line + ruler from white table):
#   shoulder axis : x = 0,   z = 127
#   elbow axis    : x = 112, z = 320
#   tilt wrist    : x = 250
#   claw middle   : x = 335
# The upper arm is L-shaped (up ~175, then ~40 sideways), so L1 is the STRAIGHT LINE
# from the shoulder axis to the elbow axis, and its angle is not the phone-level angle
# of the upright part.
ELBOW_DX_AT_REF = 112.0          # mm, horizontal shoulder -> elbow in the reference pose
ELBOW_DZ_AT_REF = 320.0 - 127.0  # mm, vertical shoulder -> elbow in the reference pose
L1 = math.hypot(ELBOW_DX_AT_REF, ELBOW_DZ_AT_REF)                     # ~223.1 mm
ARM1_LINE_AT_REF = math.degrees(math.atan2(ELBOW_DZ_AT_REF, ELBOW_DX_AT_REF))  # ~59.9 deg at arm1 servo 50
L2 = 140.0         # elbow axis -> tilt-wrist axis (plumb check: 138)
# Claw middle in the reference pose: x = 335, z = 335, i.e. 85 mm forward and 15 mm ABOVE
# the level forearm/wrist line. So the wrist -> claw-middle line points up ~10 deg.
CLAW_DX_AT_REF = 85.0
CLAW_DZ_AT_REF = 15.0
L3 = math.hypot(CLAW_DX_AT_REF, CLAW_DZ_AT_REF)                          # ~86.3 mm
CLAW_LINE_AT_REF = math.degrees(math.atan2(CLAW_DZ_AT_REF, CLAW_DX_AT_REF))  # ~10.0 deg at tilt servo 15
BASE_HEIGHT = 127.0  # white table (work surface) -> shoulder axis, measured directly

# Servo-angle safety limits (degrees). Must match the Arduino sketch.
JOINT_LIMITS = {
    "base": (0, 160),
    "arm1": (30, 150),
    "elbow": (10, 130),
    "tiltWrist": (10, 70),
    "twistWrist": (0, 179),
    "claw": (100, 179),
}

# --- joint calibration: servo angle <-> geometric angle ---
# servo_angle = servo_ref + dir * (geom_angle - geom_ref)
#
# Geometric angle definitions used by fk():
#   base      : 0 deg = arm points straight at the work area (robot +x)
#   arm1      : upper-arm angle from horizontal, 90 deg = vertical
#   elbow     : forearm angle relative to upper arm, -90 deg = forearm
#               horizontal/forward while upper arm is vertical
#   tiltWrist : claw angle relative to forearm, 0 deg = claw in line with forearm
#
# servo_ref = servo angle you read with SHOW in the reference pose
# geom_ref  = geometric angle of that reference pose (do not change)
# dir       = +1 or -1 (see measuring steps)
# Measured on the real arm 2026-10-06.
CAL = {
    "base":      {"servo_ref": 15,  "geom_ref": 0.0,   "dir": -1},  # overridden by CAL_TABLES["base"]
    "arm1":      {"servo_ref": 50,  "geom_ref": ARM1_LINE_AT_REF, "dir": -1},  # straight shoulder->elbow line
    "elbow":     {"servo_ref": 110, "geom_ref": -90.0, "dir": -1},
    "tiltWrist": {"servo_ref": 15,  "geom_ref": 0.0,   "dir": -1},
}

# Optional measured lookup tables: (servo_deg, geom_deg) pairs from the phone-level test.
# If a joint has a table, it is used instead of servo_ref/dir (linear interpolation
# between points, straight-line extension past the ends). Add more points to improve it.
#   elbow geom = forearm angle vs table - shoulder->elbow line angle (arm1 = 50)
#   tilt  geom = claw angle vs table - forearm angle vs table
CAL_TABLES = {
    # elbow: clean phone-level sweep 2026-10-06 with arm1 = 50.
    # geom = forearm angle vs table - shoulder->elbow line angle (ARM1_LINE_AT_REF)
    "elbow": [(servo, forearm - ARM1_LINE_AT_REF) for servo, forearm in [
        (50, 56.0), (60, 46.0), (70, 36.0), (80, 27.0), (90, 17.0),
        (100, 8.0), (110, 0.0), (120, -9.0), (130, -19.0),
    ]],
    # tiltWrist: phone readings of the claw body relative to the forearm, shifted by the
    # claw-line offset (wrist axis -> claw middle points ~10 deg above the claw body).
    "tiltWrist": [(servo, rel + CLAW_LINE_AT_REF) for servo, rel in [
        (15, 0.0), (40, -20.0), (70, -45.0),
    ]],
    # base: angle of the arm from the tape line (+ = LEFT, seen from behind), 2026-10-06.
    #   servo 15 = arm exactly on the tape line (visual check)
    #   10 / 40 / 70 from --verify-fk plumb measurements (atan2(y, x), r ~ 340-410 mm)
    # Past the last point the table extends in a straight line. Add a plumb-measured
    # point at 100 (and higher) if weeds will be far to the right.
    "base": [(servo, ang) for servo, ang in [
        (10, 4.6), (15, 0.0), (40, -19.3), (70, -40.2),
    ]],
}
IK_JOINTS = ["base", "arm1", "elbow", "tiltWrist"]


def _interp_extrap(x, xs, ys):
    """Piecewise-linear interpolation that keeps going straight past the end points."""
    xs = np.asarray(xs, dtype=float)
    ys = np.asarray(ys, dtype=float)
    order = np.argsort(xs)
    xs, ys = xs[order], ys[order]
    if x <= xs[0]:
        return float(ys[0] + (x - xs[0]) * (ys[1] - ys[0]) / (xs[1] - xs[0]))
    if x >= xs[-1]:
        return float(ys[-1] + (x - xs[-1]) * (ys[-1] - ys[-2]) / (xs[-1] - xs[-2]))
    return float(np.interp(x, xs, ys))


# If you re-mount the claw on the tilt-wrist servo horn so the claw points MORE UP,
# put the extra angle here (MG90S horn: 1 spline tooth = 18 deg, 2 teeth = 36 deg).
# Then check with:  python stem_to_robot_move.py --check-pose 15 50 110 15
TILT_HORN_SHIFT_DEG = 0.0

# Elbow direction. -1 = REVERSED: the elbow table is mirrored around servo 110
# (forearm level), so e.g. servo 130 now means what 90 meant before.
# Test: MOVE 15 50 110 90 15 150, then MOVE 15 50 130 90 15 150.
#   forearm goes DOWN at 130 -> use +1      forearm goes UP at 130 -> use -1
ELBOW_DIRECTION = 1    # back to the measured direction (2026-10-13): -1 made the grab ~120 mm too high
ELBOW_MIRROR_SERVO = 110


def geom_to_servo(name, geom_deg):
    if name == "elbow" and ELBOW_DIRECTION == -1:
        return 2 * ELBOW_MIRROR_SERVO - _geom_to_servo_raw(name, geom_deg)
    return _geom_to_servo_raw(name, geom_deg)


def servo_to_geom(name, servo_deg):
    if name == "elbow" and ELBOW_DIRECTION == -1:
        servo_deg = 2 * ELBOW_MIRROR_SERVO - servo_deg
    return _servo_to_geom_raw(name, servo_deg)


def _geom_to_servo_raw(name, geom_deg):
    if name == "tiltWrist":
        geom_deg = geom_deg - TILT_HORN_SHIFT_DEG
    if name in CAL_TABLES:
        pts = CAL_TABLES[name]
        return _interp_extrap(geom_deg, [g for _, g in pts], [s for s, _ in pts])
    c = CAL[name]
    return c["servo_ref"] + c["dir"] * (geom_deg - c["geom_ref"])


def _servo_to_geom_raw(name, servo_deg):
    shift = TILT_HORN_SHIFT_DEG if name == "tiltWrist" else 0.0
    if name in CAL_TABLES:
        pts = CAL_TABLES[name]
        return _interp_extrap(servo_deg, [s for s, _ in pts], [g for _, g in pts]) + shift
    c = CAL[name]
    return c["geom_ref"] + (servo_deg - c["servo_ref"]) * c["dir"]


def geom_limits_rad(name):
    lo, hi = JOINT_LIMITS[name]
    a, b = servo_to_geom(name, lo), servo_to_geom(name, hi)
    return math.radians(min(a, b)), math.radians(max(a, b))


GEOM_LIMITS = [geom_limits_rad(n) for n in IK_JOINTS]


def clip_q(q):
    q = np.array(q, dtype=float)
    for i, (lo, hi) in enumerate(GEOM_LIMITS):
        q[i] = np.clip(q[i], lo, hi)
    return q


CURRENT_POSE = {
    "base": 80.0,
    "arm1": 60.0,
    "elbow": 90.0,
    "tiltWrist": 50.0,
    "twistWrist": 90.0,
    "claw": 150.0,
}

MAX_TARGET_Z_MM = 600.0
DETECTION_WAIT_SECONDS = 1.0
MAX_IK_ERROR_MM = 10.0  # skip targets the arm cannot reach accurately

# =====================================================================
# Pick-and-place settings (pause scan -> grab -> wastebasket -> wait -> scan)
# =====================================================================
# Claw servo values. TODO: check with C+ / C- in the Serial Monitor.
CLAW_OPEN = 150         # jaws open wide enough to go around the stem
CLAW_CLOSED = 100       # measured 2026-10-11: jaws closed on the stem
TWIST_ANGLE = 90        # twistWrist during the whole pick

# Claw FACING the plant: the jaws point straight at the stem from the base side.
# GRAB_PITCH_DEG = angle of the jaws vs the table: 0 = level (facing the stem),
# -90 = pointing straight down. The program tries GRAB_PITCH_DEG first, then tilts
# down in 5-degree steps until the arm can reach without touching the table.
FACE_PLANT = True
GRAB_PITCH_DEG = 0.0
MIN_GRAB_PITCH_DEG = -90.0
FORWARD_MM = 30.0       # go down this far SHORT of the stem (toward the base), then move forward onto it
APPROACH_MM = 50.0      # first move this far ABOVE the stem, then go straight down
LIFT_MM = 100.0         # after grabbing, pull the weed up this far before carrying it
GRIP_SETTLE_S = 0.6     # time for the claw to close / open
WAIT_AFTER_PICK_S = 10  # pause before scanning again (change with --wait-after)
# Grab-point correction in MILLIMETERS (not servo degrees), so shoulder, elbow and wrist
# are all solved together and the claw lands where you want.
#   GRAB_FORWARD_MM: + = farther from the base, - = closer to the base
#   GRAB_Z_MM:       + = higher,                 - = lower
# -30 / -22 gives the same claw position as the old "elbow +10" servo offset.
GRAB_FORWARD_MM = 40.0   # 2026-10-13: +10 mm more forward (was 30)
GRAB_LEFT_MM = 15.0      # sideways: + = more to the LEFT (seen from behind), - = right (2026-10-13: was landing right)
GRAB_Z_MM = 23.0         # 2026-10-13: 15 mm lower (was 38)
# Extra servo degrees after IK (leave at 0 - servo offsets break the arm coordination).
PICK_SERVO_OFFSET = {"base": 0, "arm1": 0, "elbow": 0,
                     "twistWrist": 0, "tiltWrist": 0, "claw": 0}
PATH_STEP_MM = 0.0      # 0 = one smooth coordinated move per step (no stops). e.g. 20 = straighter path, small stops
# Same min/max as the Arduino sketch; the offset pose is clamped to these.
ARDUINO_LIMITS = {"base": (0, 180), "arm1": (30, 150), "elbow": (10, 150),
                  "twistWrist": (0, 179), "tiltWrist": (0, 100), "claw": (100, 179)}  # tilt max raised to 100: change tiltWristMaxAngle in the .ino too
CONFIRM_FRAMES = 3      # weed must be seen this many frames in a row (same spot) first
CONFIRM_RADIUS_MM = 20.0
MAX_RAISE_MM = 80.0     # if a grab pose would go below the table, aim up to this much higher
MIN_Z_MM = 5.0          # refuse any pose that puts elbow/wrist/claw below the table

# Servo angles above the wastebasket (base, arm1, elbow, tiltWrist).
# TODO: find these with the Serial Monitor (jog with MOVE until the claw is over the
# basket, high enough to clear the rim), then fill them in. Example: (130, 60, 110, 30)
# While this is None the robot will detect and plan, but will NOT pick.
# Safe HIGH pose for turning the base (arm1, elbow, tiltWrist). The base only turns while
# the arm is in this pose, so the claw/weed passes OVER the side of the wastebasket.
# (30, 130, 30) puts the claw about 345 mm above the table. If it still hits, lower
# tiltWrist (claw points up more) or ask for a new pose.
CARRY_POSE = (30, 130, 30)
# Tilt-wrist servo while HOLDING the weed (after the claw closes, until it is over the
# wastebasket). Lower = claw points more UP so the weed can't slide out.
# 0 -> claw about 13 deg above level in the carry pose (30 -> 11 deg below level).
HOLD_TILT = 0
HOME_BASE = 30          # base angle of the Arduino HOME pose
BIN_POSE = (170, 50, 150, 50)   # base, arm1, elbow, tiltWrist (measured 2026-10-11)


def fk(q):
    q1, q2, q3, q4 = q
    x = math.cos(q1) * (
        L1 * math.cos(q2)
        + L2 * math.cos(q2 + q3)
        + L3 * math.cos(q2 + q3 + q4)
    )
    y = math.sin(q1) * (
        L1 * math.cos(q2)
        + L2 * math.cos(q2 + q3)
        + L3 * math.cos(q2 + q3 + q4)
    )
    z = BASE_HEIGHT + (
        L1 * math.sin(q2)
        + L2 * math.sin(q2 + q3)
        + L3 * math.sin(q2 + q3 + q4)
    )
    return np.array([x, y, z], dtype=np.float64)


def fk_points_from_servo(pose):
    """Positions (mm) of shoulder, elbow, wrist and claw middle, for plumb-line checks."""
    q1, q2, q3, q4 = [math.radians(servo_to_geom(n, pose[n])) for n in IK_JOINTS]
    pts = {}
    r, z = 0.0, BASE_HEIGHT
    pts["shoulder"] = (r, z)
    r += L1 * math.cos(q2); z += L1 * math.sin(q2)
    pts["elbow"] = (r, z)
    r += L2 * math.cos(q2 + q3); z += L2 * math.sin(q2 + q3)
    pts["tilt_wrist"] = (r, z)
    r += L3 * math.cos(q2 + q3 + q4); z += L3 * math.sin(q2 + q3 + q4)
    pts["claw_middle"] = (r, z)
    return {k: np.array([math.cos(q1) * rr, math.sin(q1) * rr, zz]) for k, (rr, zz) in pts.items()}


def fk_from_servo(pose):
    """pose = dict of servo angles, e.g. {'base':80,'arm1':90,'elbow':90,'tiltWrist':40}"""
    q = [math.radians(servo_to_geom(n, pose[n])) for n in IK_JOINTS]
    return fk(q)


# =====================================================================
# FK verification with real ruler measurements (--verify-fk)
# =====================================================================
# Measurement frame (same frame as fk()):
#   origin : point on the table directly under the base rotation axis
#   +x     : forward along your base reference tape line (base servo_ref pose)
#   +y     : to the LEFT of the tape line (right side = negative y)
#   +z     : up from the table surface
#   Measure to the grasp point = middle between the two claw jaws.
#
# Servo poses to test: (base, arm1, elbow, tiltWrist). All are inside the
# Arduino safety limits and predicted to stay >= 75 mm above the table
# (updated 2026-10-06 for the L-shaped upper arm model).
# Edit this list if a pose would hit the camera, cables, or anything else.
VERIFY_POSES = [
    (10, 50, 110, 15),   # reference pose: upper arm vertical, forearm level
    (10, 80, 100, 40),
    (10, 80, 120, 15),
    (10, 90, 90, 40),
    (40, 80, 100, 15),
    (40, 70, 90, 15),
    (40, 80, 120, 40),
    (70, 80, 100, 40),
    (70, 90, 90, 40),
]
VERIFY_TWIST = 90
VERIFY_CLAW = 150
VERIFY_PASS_MM = 10.0
FIT_NAMES = ["base_ref", "arm1_ref", "elbow_ref", "tilt_ref", "L1", "L2", "L3", "BASE_HEIGHT"]


def fk_with_params(servo4, params):
    """FK using a candidate parameter set (used for the correction fit)."""
    b_ref, a_ref, e_ref, t_ref, l1, l2, l3, bh = params
    refs = {"base": b_ref, "arm1": a_ref, "elbow": e_ref, "tiltWrist": t_ref}
    q = []
    for name, servo in zip(IK_JOINTS, servo4):
        shift = refs[name] - CAL[name]["servo_ref"]   # fitted zero shift in servo degrees
        q.append(math.radians(servo_to_geom(name, servo - shift)))
    q1, q2, q3, q4 = q
    r = l1 * math.cos(q2) + l2 * math.cos(q2 + q3) + l3 * math.cos(q2 + q3 + q4)
    z = bh + l1 * math.sin(q2) + l2 * math.sin(q2 + q3) + l3 * math.sin(q2 + q3 + q4)
    return np.array([math.cos(q1) * r, math.sin(q1) * r, z])


def current_params():
    return np.array([
        CAL["base"]["servo_ref"], CAL["arm1"]["servo_ref"],
        CAL["elbow"]["servo_ref"], CAL["tiltWrist"]["servo_ref"],
        L1, L2, L3, BASE_HEIGHT,
    ], dtype=float)


def fit_corrections(rows, iters=50, prior_weight=0.05):
    """Damped least squares: which small parameter changes best explain the errors?
    prior_weight keeps the answer close to your measured values."""
    p0 = current_params()
    p = p0.copy()
    for _ in range(iters):
        res, J = [], []
        for servo4, meas in rows:
            pred = fk_with_params(servo4, p)
            res.extend(meas - pred)
            row_j = np.zeros((3, len(p)))
            for k in range(len(p)):
                dp = np.zeros_like(p)
                dp[k] = 1e-4
                row_j[:, k] = (fk_with_params(servo4, p + dp) - pred) / 1e-4
            J.append(row_j)
        res = np.array(res)
        J = np.vstack(J)
        # prior rows: stay near the original values
        res = np.concatenate([res, prior_weight * (p0 - p)])
        J = np.vstack([J, prior_weight * np.eye(len(p))])
        step = np.linalg.solve(J.T @ J + 1e-3 * np.eye(len(p)), J.T @ res)
        p = p + step
        if np.linalg.norm(step) < 1e-4:
            break
    errs = [np.linalg.norm(meas - fk_with_params(s4, p)) for s4, meas in rows]
    return p, float(np.mean(errs))


SERIAL_HELP = """
Serial port problem on {port}. Most common causes:
  1. Arduino IDE Serial Monitor (or another script) is still open on the same port.
     -> Close the Serial Monitor / other program, then run again.
     -> Check with:  fuser -v {port}
  2. The Arduino reset or disconnected (USB cable loose, or servo current made the
     5 V dip). -> Check with:  ls /dev/ttyACM*   (the port name may have changed)
"""


def wait_for_arduino_move(ser, timeout_s=15.0):
    end = time.time() + timeout_s
    while time.time() < end:
        try:
            line = ser.readline().decode("ascii", errors="ignore").strip()
        except serial.SerialException as exc:
            print(f"  SERIAL ERROR: {exc}")
            print(SERIAL_HELP.format(port=ser.port))
            raise SystemExit(1)
        if "Moved to absolute pose" in line:
            return True
    return False


def run_fk_verification(args):
    ser = None
    if not args.dry_run:
        print(f"Opening Arduino serial connection on {args.port}...", flush=True)
        try:
            ser = serial.Serial(args.port, 9600, timeout=0.5, exclusive=True)
        except (serial.SerialException, OSError) as exc:
            print(f"Cannot open {args.port}: {exc}")
            print(SERIAL_HELP.format(port=args.port))
            raise SystemExit(1)
        time.sleep(2)
        ser.reset_input_buffer()

    print()
    print("=========== FK VERIFICATION ===========")
    print("Frame: origin = table point under base axis, +x = along tape line,")
    print("       +y = LEFT of tape line (right is negative), +z = up from table.")
    print("Measure to the middle between the claw jaws, in mm.")
    print("Input:  x y z   |  s = skip pose  |  q = quit")
    print("SAFETY: keep one hand on the servo power switch.")
    print("========================================")

    rows, log = [], []
    try:
        for i, servo4 in enumerate(VERIFY_POSES, 1):
            b, a1, e, ti = servo4
            pred = fk_from_servo({"base": b, "arm1": a1, "elbow": e, "tiltWrist": ti})
            cmd = f"MOVE {b} {a1} {e} {VERIFY_TWIST} {ti} {VERIFY_CLAW}"
            print(f"\n[{i}/{len(VERIFY_POSES)}] {cmd}")
            print(f"  FK predicted (mm): x={pred[0]:.1f}  y={pred[1]:.1f}  z={pred[2]:.1f}")

            if ser is not None:
                ser.reset_input_buffer()
                ser.write((cmd + "\n").encode("ascii"))
                ser.flush()
                if not wait_for_arduino_move(ser):
                    print("  WARNING: no 'Moved to absolute pose' reply; check the arm before measuring.")
                time.sleep(0.5)  # let the arm settle
            else:
                print("  (dry-run) send this MOVE yourself in Serial Monitor.")

            while True:
                text = input("  Measured x y z > ").strip().lower()
                if text in ("s", "q"):
                    break
                try:
                    vals = [float(v) for v in text.replace(",", " ").split()]
                    if len(vals) == 3:
                        break
                except ValueError:
                    pass
                print("  Please type three numbers, e.g. 265 0 205")
            if text == "q":
                break
            if text == "s":
                continue

            meas = np.array(vals)
            err = meas - pred
            dist = float(np.linalg.norm(err))
            verdict = "PASS" if dist <= VERIFY_PASS_MM else "FAIL"
            print(f"  error (meas - FK): dx={err[0]:+.1f}  dy={err[1]:+.1f}  dz={err[2]:+.1f}  "
                  f"|e|={dist:.1f} mm  {verdict}")
            rows.append((servo4, meas))
            log.append([i, b, a1, e, ti, *np.round(pred, 1), *np.round(meas, 1),
                        *np.round(err, 1), round(dist, 1), verdict])
    finally:
        if ser is not None and ser.is_open:
            ser.close()

    if not rows:
        print("\nNo measurements entered.")
        return

    errs = np.array([r[11:14] for r in log], dtype=float)
    dists = np.array([r[-2] for r in log], dtype=float)
    n_pass = sum(1 for r in log if r[-1] == "PASS")
    print("\n=============== SUMMARY ===============")
    print(f"Poses measured : {len(rows)}   PASS: {n_pass}   FAIL: {len(rows) - n_pass}")
    print(f"Mean |error|   : {dists.mean():.1f} mm   Max: {dists.max():.1f} mm   "
          f"RMS: {math.sqrt((dists ** 2).mean()):.1f} mm")
    print(f"Mean bias      : dx={errs[:, 0].mean():+.1f}  dy={errs[:, 1].mean():+.1f}  "
          f"dz={errs[:, 2].mean():+.1f} mm")

    print("\nHints:")
    if abs(errs[:, 2].mean()) > 5:
        if abs(errs[:, 2].mean() - errs[0, 2]) < 10:
            print(f"  - z is off by {errs[:, 2].mean():+.1f} mm in every pose -> re-measure BASE_HEIGHT.")
        else:
            print(f"  - z error changes with the pose (pose 1: {errs[0, 2]:+.0f} mm, mean: {errs[:, 2].mean():+.0f} mm)"
                  f" -> not BASE_HEIGHT; check arm1/elbow real angles with a level, and arm sag.")
    # base direction check: poses with base != servo_ref should have y with the same sign as FK
    sign_flips = 0
    sign_checks = 0
    for (servo4, meas), r in zip(rows, log):
        fk_y = r[6]
        if abs(fk_y) > 30 and abs(meas[1]) > 30:
            sign_checks += 1
            if np.sign(fk_y) != np.sign(meas[1]):
                sign_flips += 1
    if sign_checks and sign_flips >= max(1, sign_checks // 2 + 1):
        print(f"  - y has the OPPOSITE sign in {sign_flips}/{sign_checks} turned poses -> flip base dir "
              f"({CAL['base']['dir']:+d} -> {-CAL['base']['dir']:+d}) before anything else.")
    if abs(errs[:, 0].mean()) > 30 and abs(errs[0, 0]) > 30:
        print(f"  - x is off by about {errs[:, 0].mean():+.0f} mm even in pose 1 -> check that x is measured "
              f"from the base ROTATION CENTER, not the edge of the base or board.")
    if any(m[2] <= 1 for _, m in rows):
        print("  - Some poses measured z = 0: the claw touched the table. Those poses are lower than FK "
              "predicts; check arm1/elbow angles with a level (see angle check).")
    if abs(errs[:, 1].mean()) > 5:
        print("  - y is shifted on average -> re-check base servo_ref (tape line direction).")
    if dists.mean() <= VERIFY_PASS_MM:
        print("  - Average error is within target. FK is good enough; next: redo camera_to_robot_transform.json.")
    else:
        print("  - Errors that grow with reach usually mean L1/L2/L3 or arm1/elbow servo_ref are off.")

    p_new = None
    if len(rows) >= 5:
        p_new, fit_err = fit_corrections(rows)
        p_old = current_params()
        delta = np.abs(p_new - p_old)
        if fit_err > 25 or np.any(delta[:4] > 15) or np.any(delta[4:] > 40):
            print("\nSuggested corrections: NOT reliable (needed changes are too large or the fit is poor).")
            print("  Fix the big problems first (base dir, measurement origin, angle check), then re-run.")
            p_new = None
    if len(rows) >= 5 and p_new is not None:
        print(f"\nSuggested corrections (reference only; mean error would be ~{fit_err:.1f} mm):")
        for name, old, new in zip(FIT_NAMES, p_old, p_new):
            flag = "   <-- consider changing" if abs(new - old) >= 2 else ""
            print(f"  {name:12s} {old:7.1f} -> {new:7.1f}{flag}")
        print("  Change one or two values at a time, then run --verify-fk again.")
    elif len(rows) < 5:
        print("\nMeasure at least 5 poses to get suggested corrections.")

    out = ROOT / time.strftime("fk_verify_%Y%m%d_%H%M%S.csv")
    with open(out, "w") as f:
        f.write("pose,base,arm1,elbow,tilt,fk_x,fk_y,fk_z,meas_x,meas_y,meas_z,dx,dy,dz,err_mm,result\n")
        for r in log:
            f.write(",".join(str(v) for v in r) + "\n")
    print(f"\nSaved results to {out}")


def jacobian_numeric(q, eps=1e-6):
    p0 = fk(q)
    J = np.zeros((3, len(q)))
    for i in range(len(q)):
        dq = np.zeros_like(q)
        dq[i] = eps
        p1 = fk(q + dq)
        J[:, i] = (p1 - p0) / eps
    return J


def ik_point(target_xyz, q0=None, max_iter=400, tol=1e-3, damping=1e-2):
    if q0 is None:
        # Geometric start guess: upper arm vertical, forearm forward, wrist tilted down a bit
        q = np.radians([0.0, 90.0, -90.0, -30.0])
    else:
        q = np.array(q0, dtype=float)
    q = clip_q(q)

    for _ in range(max_iter):
        p = fk(q)
        e = target_xyz - p
        if np.linalg.norm(e) < tol:
            break

        J = jacobian_numeric(q)
        JTJ = J.T @ J
        reg = damping * np.eye(JTJ.shape[0])
        dq = np.linalg.solve(JTJ + reg, J.T @ e)
        q = clip_q(q + dq)

    return q


def q_to_pose_deg(q):
    pose = {
        "base": geom_to_servo("base", math.degrees(q[0])),
        "arm1": geom_to_servo("arm1", math.degrees(q[1])),
        "elbow": geom_to_servo("elbow", math.degrees(q[2])),
        "tiltWrist": geom_to_servo("tiltWrist", math.degrees(q[3])),
        "twistWrist": CURRENT_POSE["twistWrist"],
        "claw": CURRENT_POSE["claw"],
    }
    for name, (lo, hi) in JOINT_LIMITS.items():
        pose[name] = int(round(np.clip(pose[name], lo, hi)))
    return pose


def get_intrinsic(depth_frame, color_frame=None):
    # After aligning depth to color, the color intrinsics describe the pixel grid.
    if color_frame is not None:
        try:
            return color_frame.get_stream_profile().as_video_stream_profile().get_intrinsic()
        except Exception:
            pass
    return depth_frame.get_stream_profile().get_intrinsic()


def depth_to_camera_xyz(depth_frame, x_px, y_px, radius=4, color_frame=None):
    depth_map = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
    h = depth_frame.get_height()
    w = depth_frame.get_width()
    depth_map = depth_map.reshape(h, w).astype(np.float32)

    x = int(round(x_px))
    y = int(round(y_px))
    x = max(0, min(w - 1, x))
    y = max(0, min(h - 1, y))

    x_start = max(0, x - radius)
    x_end = min(w, x + radius + 1)
    y_start = max(0, y - radius)
    y_end = min(h, y + radius + 1)
    raw_samples = depth_map[y_start:y_end, x_start:x_end]
    scale = float(depth_frame.get_depth_scale())
    samples_mm = raw_samples * scale
    valid_samples = samples_mm[np.isfinite(samples_mm) & (samples_mm > 0)]
    if valid_samples.size == 0:
        return None, 0

    # Median rejects isolated depth holes/outliers near the box-bottom pixel.
    z_mm = float(np.median(valid_samples))
    if not np.isfinite(z_mm) or z_mm <= 0:
        return None, int(valid_samples.size)

    intr = get_intrinsic(depth_frame, color_frame)
    fx = intr.fx
    fy = intr.fy
    cx = intr.cx
    cy = intr.cy

    X = (x - cx) * z_mm / fx
    Y = (y - cy) * z_mm / fy
    Z = z_mm
    return np.array([X, Y, Z], dtype=np.float64), int(valid_samples.size)


def camera_to_robot(camera_xyz):
    return R @ camera_xyz + t


def build_move_command(pose):
    return "MOVE {} {} {} {} {} {}".format(
        int(round(pose["base"])),
        int(round(pose["arm1"])),
        int(round(pose["elbow"])),
        int(round(pose["twistWrist"])),
        int(round(pose["tiltWrist"])),
        int(round(pose["claw"])),
    )


def send_move(ser, pose):
    """Send MOVE and wait until the Arduino reports the move is finished."""
    cmd = build_move_command(pose)
    print("Arduino MOVE command:", cmd, flush=True)
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode("ascii"))
    ser.flush()
    if not wait_for_arduino_move(ser):
        print("  WARNING: no 'Moved to absolute pose' reply from Arduino.", flush=True)
    time.sleep(0.02)   # short pause so path steps flow into each other


def send_home(ser):
    print("Arduino command: HOME", flush=True)
    ser.reset_input_buffer()
    ser.write(b"HOME\n")
    ser.flush()
    end = time.time() + 15
    while time.time() < end:
        try:
            line = ser.readline().decode("ascii", errors="ignore").strip()
        except serial.SerialException as exc:
            print(f"SERIAL ERROR: {exc}")
            print(SERIAL_HELP.format(port=ser.port))
            raise SystemExit(1)
        if "Moved to HOME pose" in line:
            break
    time.sleep(0.2)


def pose_below_table(pose):
    pts = fk_points_from_servo(pose)
    for name in ("elbow", "tilt_wrist", "claw_middle"):
        if pts[name][2] < MIN_Z_MM:
            return f"{name} would be at z = {pts[name][2]:.0f} mm"
    return None


def ik_facing(target_xyz, pitch_deg):
    """Exact IK with a fixed claw angle. Returns (servo pose, None) or (None, reason)."""
    x, y, z = [float(v) for v in target_xyz]
    r = math.hypot(x, y)
    line = math.radians(pitch_deg + CLAW_LINE_AT_REF)   # direction of the wrist->claw-middle line
    rw = r - L3 * math.cos(line)
    zw = z - L3 * math.sin(line) - BASE_HEIGHT
    d2 = rw * rw + zw * zw
    c3 = (d2 - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if abs(c3) > 1.0:
        return None, "out of reach"
    q3 = -math.acos(c3)                                   # elbow above the wrist
    q2 = math.atan2(zw, rw) - math.atan2(L2 * math.sin(q3), L1 + L2 * math.cos(q3))
    q4 = line - q2 - q3
    q1 = math.atan2(y, x)
    pose = {
        "base": geom_to_servo("base", math.degrees(q1)),
        "arm1": geom_to_servo("arm1", math.degrees(q2)),
        "elbow": geom_to_servo("elbow", math.degrees(q3)),
        "tiltWrist": geom_to_servo("tiltWrist", math.degrees(q4)),
    }
    for name, v in pose.items():
        lo, hi = ARDUINO_LIMITS[name]
        if not (lo - 0.5 <= v <= hi + 0.5):
            return None, f"{name} would need servo {v:.0f} (limit {lo}-{hi})"
    return {k: int(round(v)) for k, v in pose.items()}, None


def ik_pose(target_xyz, claw, pitch_deg=None):
    if pitch_deg is not None:
        pose, why = ik_facing(target_xyz, pitch_deg)
        if pose is None:
            return None, why
        pose["twistWrist"] = TWIST_ANGLE
        pose["claw"] = claw
        for name, extra in PICK_SERVO_OFFSET.items():
            lo, hi = ARDUINO_LIMITS[name]
            pose[name] = min(max(pose[name] + extra, lo), hi)
        return pose, 0.0
    return ik_pose_free(target_xyz, claw)


def ik_pose_free(target_xyz, claw):
    """IK for a point; returns (pose, error_mm)."""
    q = ik_point(np.asarray(target_xyz, dtype=float))
    pose = q_to_pose_deg(q)
    pose["twistWrist"] = TWIST_ANGLE
    pose["claw"] = claw
    for name, extra in PICK_SERVO_OFFSET.items():
        lo, hi = ARDUINO_LIMITS[name]
        pose[name] = min(max(pose[name] + extra, lo), hi)
    err = float(np.linalg.norm(fk(q) - target_xyz))
    return pose, err


def plan_pick(target_xyz):
    """Try the full forward slide first; if the start point is out of reach, use a shorter slide."""
    first = None
    for fwd in (FORWARD_MM, FORWARD_MM * 2 / 3, FORWARD_MM / 3, 0.0):
        steps, problem = _plan_pick(target_xyz, fwd)
        if first is None:
            first = (steps, problem)
        if problem is None or "BIN_POSE" in problem:
            if fwd != FORWARD_MM:
                print(f"  (forward slide shortened to {fwd:.0f} mm so the arm can reach)")
            return steps, problem
    return first


def best_grab_pose(point, claw):
    """Most level claw angle (facing the plant) that is reachable and stays above the table.
    Returns (pose, pitch_deg or None, problem)."""
    last = "out of reach"
    if FACE_PLANT:
        for pitch in np.arange(GRAB_PITCH_DEG, MIN_GRAB_PITCH_DEG - 0.1, -5.0):
            pose, why = ik_pose(point, claw, float(pitch))
            if pose is None:
                last = why
                continue
            below = pose_below_table(pose)
            if below:
                last = below + " (below the table)"
                continue
            return pose, float(pitch), None
    pose, err = ik_pose_free(point, claw)
    if err > MAX_IK_ERROR_MM:
        return None, None, f"IK error {err:.1f} mm (out of reach); facing grab: {last}"
    below = pose_below_table(pose)
    if below:
        return None, None, below + " (below the table)"
    return pose, None, None


def solve_key_point(point, claw):
    """Best pose for one point, raised in 5 mm steps if it would go below the table.
    Returns (pose, used_point, pitch, raise_mm, problem)."""
    problem = None
    for raise_mm in np.arange(0.0, MAX_RAISE_MM + 0.1, 5.0):
        used = point + np.array([0.0, 0.0, raise_mm])
        pose, pitch, problem = best_grab_pose(used, claw)
        if problem is None:
            return pose, used, pitch, raise_mm, None
        if "below the table" not in problem:
            break
    return None, None, None, 0.0, problem


def pose_on_path(point, pitch, claw):
    """Pose for an in-between point; keeps the claw angle if possible."""
    if pitch is not None:
        pose, why = ik_pose(point, claw, pitch)
        if pose is not None and not pose_below_table(pose):
            return pose
    pose, _, _ = best_grab_pose(point, claw)
    return pose


def straight_path(label, p0, pitch0, p1, pitch1, claw, end_pose):
    """Split a move into short steps so the elbow and shoulder move TOGETHER and the
    claw follows a straight line (the Arduino moves every joint at the same speed,
    which otherwise makes the claw swing in an arc)."""
    if PATH_STEP_MM <= 0:
        return [(label, end_pose)]
    n = max(1, int(math.ceil(np.linalg.norm(p1 - p0) / PATH_STEP_MM)))
    out = []
    for i in range(1, n):
        f = i / n
        pt = p0 + f * (p1 - p0)
        pitch = None if pitch0 is None or pitch1 is None else pitch0 + f * (pitch1 - pitch0)
        pose = pose_on_path(pt, pitch, claw)
        if pose is not None:
            out.append((f"{label} ({i}/{n})", pose))
    out.append((f"{label} ({n}/{n})" if n > 1 else label, end_pose))
    return out


def _plan_pick(target_xyz, forward_mm):
    """Plan every step BEFORE moving. Returns (steps, problem). steps = [(label, pose)]."""
    target = np.asarray(target_xyz, dtype=float)
    # unit vector pointing from the base outward to the stem (horizontal)
    r = float(np.hypot(target[0], target[1]))
    outward = np.array([target[0] / r, target[1] / r, 0.0]) if r > 1e-6 else np.array([1.0, 0.0, 0.0])
    left = np.array([-outward[1], outward[0], 0.0])   # 90 deg to the left of "outward"
    target = target + GRAB_FORWARD_MM * outward + GRAB_LEFT_MM * left + np.array([0.0, 0.0, GRAB_Z_MM])
    behind = target - forward_mm * outward
    hover = behind + np.array([0.0, 0.0, APPROACH_MM])
    lift = target + np.array([0.0, 0.0, LIFT_MM])
    steps = []
    prev = None   # (used_point, pitch)
    for label, point, claw in [
        ("1 move above, just short of the weed", hover, CLAW_OPEN),
        ("2 go down and forward onto the stem", target, CLAW_OPEN),
        ("3 close the claw", None, CLAW_CLOSED),
        ("4 pull the weed up", lift, CLAW_CLOSED),
    ]:
        if point is None:                      # close the claw without moving the arm
            steps.append((label, dict(steps[-1][1], claw=claw)))
            continue
        pose, used, pitch, raise_mm, problem = solve_key_point(point, claw)
        if problem:
            return steps, f"step '{label}': {problem}"
        tag = f"  [claw angle {pitch:.0f} deg]" if pitch is not None else "  [normal grab]"
        if raise_mm > 0:
            tag += f"  [raised {raise_mm:.0f} mm to stay above the table]"
        if prev is None:
            steps.append((label + tag, pose))
        else:
            steps += straight_path(label + tag, prev[0], prev[1], used, pitch, claw, pose)
        prev = (used, pitch)
    if BIN_POSE is None:
        return steps, "BIN_POSE is not set (see the TODO near the top of the file)"
    b, a1, e, ti = BIN_POSE
    bin_pose = {"base": b, "arm1": a1, "elbow": e, "tiltWrist": ti,
                "twistWrist": TWIST_ANGLE, "claw": CLAW_CLOSED}
    below = pose_below_table(bin_pose)
    if below:
        return steps, f"BIN_POSE: {below} (below the table)"
    ca1, ce, cti = CARRY_POSE
    # holding the weed: claw tilted UP (HOLD_TILT) so it doesn't fall out
    hold_here = {"base": steps[-1][1]["base"], "arm1": ca1, "elbow": ce, "tiltWrist": HOLD_TILT,
                 "twistWrist": TWIST_ANGLE, "claw": CLAW_CLOSED}
    hold_bin = dict(hold_here, base=b)
    # empty claw on the way back: normal carry tilt
    back_bin = {"base": b, "arm1": ca1, "elbow": ce, "tiltWrist": cti,
                "twistWrist": TWIST_ANGLE, "claw": CLAW_OPEN}
    steps.append(("5 raise the arm HIGH, claw tilted up (base not turning yet)", hold_here))
    steps.append(("6 turn to the wastebasket while high, claw tilted up", hold_bin))
    steps.append(("7 lower into the wastebasket, claw still tilted up", dict(bin_pose, tiltWrist=HOLD_TILT)))
    steps.append(("8 tilt the claw down over the wastebasket", bin_pose))
    steps.append(("9 open the claw (drop the weed)", dict(bin_pose, claw=CLAW_OPEN)))
    steps.append(("10 raise the arm HIGH again", back_bin))
    steps.append(("11 turn back while high", dict(back_bin, base=HOME_BASE)))
    return steps, None


def run_pick(ser, steps):
    for label, pose in steps:
        print(f"  >> {label}", flush=True)
        send_move(ser, pose)
        if "close the claw" in label or "open the claw" in label or "tilt the claw down" in label:
            time.sleep(GRIP_SETTLE_S)
    print("  >> 12 go HOME (out of the camera view)", flush=True)
    send_home(ser)


def paused_wait(seconds, window="stem target"):
    """Wait while keeping the camera window responsive. Returns False if Q/Esc pressed."""
    end = time.time() + seconds
    while time.time() < end:
        left = int(end - time.time()) + 1
        print(f"\r  scanning resumes in {left:2d} s ", end="", flush=True)
        key = cv2.waitKey(200) & 0xFF
        if key in (27, ord("q")):
            print()
            return False
    print()
    return True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Detect plant stem, convert to robot target, and send MOVE to Arduino.")
    parser.add_argument("--port", type=str, default="/dev/ttyACM0", help="Arduino serial port")
    parser.add_argument("--dry-run", action="store_true", help="Print target and pose without sending to Arduino")
    parser.add_argument("--weights", type=str, default=None, help="Path to model weights (.pt)")
    parser.add_argument("--conf", type=float, default=0.90, help="Detection confidence threshold")
    parser.add_argument("--iou", type=float, default=0.7, help="NMS IoU threshold")
    parser.add_argument("--imgsz", type=int, default=640, help="Model image size")
    parser.add_argument("--device", type=str, default="cpu", help="Model device")
    parser.add_argument(
        "--wait-after",
        type=float,
        default=WAIT_AFTER_PICK_S,
        help="Seconds to wait after dropping a weed before scanning again (default 10)",
    )
    parser.add_argument(
        "--check-pose",
        type=float,
        nargs=4,
        metavar=("BASE", "ARM1", "ELBOW", "TILT"),
        help="Print FK grasp-point position (mm) for these servo angles, then exit (no camera needed)",
    )
    parser.add_argument(
        "--verify-fk",
        action="store_true",
        help="Move through VERIFY_POSES, enter ruler measurements, and report FK error (no camera needed)",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Print low-confidence detections and camera/depth/IK diagnostics",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    if args.check_pose is not None:
        b, a1, e, ti = args.check_pose
        pose = {"base": b, "arm1": a1, "elbow": e, "tiltWrist": ti}
        geom = {n: round(servo_to_geom(n, pose[n]), 1) for n in IK_JOINTS}
        print("servo_deg:", pose)
        print("geom_deg :", geom)
        print("fk_xyz_mm:", np.round(fk_from_servo(pose), 1))
        print("joint positions (x y z mm):")
        for k, v in fk_points_from_servo(pose).items():
            print(f"  {k:12s} {v[0]:7.1f} {v[1]:7.1f} {v[2]:7.1f}")
        return

    if args.verify_fk:
        run_fk_verification(args)
        return

    if calib is None and not args.dry_run:
        print("No camera_to_robot_transform.json yet. Run calibrate_camera_to_robot.py first "
              "(or use --dry-run).")
        return

    detector = MustardDetector(model_path=args.weights, conf=args.conf, iou=args.iou, imgsz=args.imgsz, device=args.device)

    pipeline = Pipeline()
    config = Config()
    color_profiles = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
    depth_profiles = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
    config.enable_stream(color_profiles.get_default_video_stream_profile())
    config.enable_stream(depth_profiles.get_default_video_stream_profile())
    pipeline.start(config)
    align = AlignFilter(align_to_stream=OBStreamType.COLOR_STREAM)
    frame_number = 0
    ser = None
    confirm_count = 0
    confirm_xyz = None
    picked = 0

    try:
        if not args.dry_run:
            print(f"Opening Arduino serial connection on {args.port}...", flush=True)
            try:
                ser = serial.Serial(args.port, 9600, timeout=0.5, exclusive=True)
            except (serial.SerialException, OSError) as exc:
                print(f"Cannot open {args.port}: {exc}")
                print(SERIAL_HELP.format(port=args.port))
                return
            # Opening a Uno serial port may reset the board; allow setup() to finish once.
            time.sleep(2)
            ser.reset_input_buffer()
            send_home(ser)

        while True:
            frame_number += 1
            frames = pipeline.wait_for_frames(1000)
            if frames is None:
                if args.debug:
                    print(f"[debug] frame {frame_number}: no frames", flush=True)
                continue
            frames = align.process(frames)
            if frames is None:
                #if args.debug:
                #    print(f"[debug] frame {frame_number}: alignment failed", flush=True)
                continue

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if color_frame is None or depth_frame is None:
                if args.debug:
                    print(
                        f"[debug] frame {frame_number}: missing color/depth frame "
                        f"(color={color_frame is not None}, depth={depth_frame is not None})",
                        flush=True,
                    )
                continue

            bgr = frame_to_bgr_image(color_frame)
            if bgr is None:
                if args.debug:
                    print(f"[debug] frame {frame_number}: color conversion failed", flush=True)
                continue

            inference_conf = 0.05 if args.debug else detector.conf
            results = detector.model(
                bgr,
                conf=inference_conf,
                iou=detector.iou,
                imgsz=detector.imgsz,
                device=detector.device,
                verbose=False,
            )
            result = results[0]

            if result.boxes is not None and len(result.boxes) > 0:
                confidences = result.boxes.conf.cpu().numpy()
                best_index = int(np.argmax(confidences))
                best = result.boxes[best_index]
                x1_box, y1_box, x2_box, y2_box = map(int, best.xyxy[0].tolist())
                confidence = float(best.conf[0])
                color = (0, 255, 0)
                cv2.rectangle(bgr, (x1_box, y1_box), (x2_box, y2_box), color, 2)
                cv2.putText(
                    bgr,
                    f"mustard {confidence:.2f}",
                    (x1_box, max(20, y1_box - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    color,
                    2,
                )

                best_x1, best_y1, best_x2, best_y2 = map(float, best.xyxy[0].tolist())
                stem_x = int(round((best_x1 + best_x2) / 2.0))
                stem_y = int(round(best_y2 - (best_y2 - best_y1) / 10.0))
                cv2.drawMarker(
                    bgr,
                    (stem_x, stem_y),
                    (0, 0, 255),
                    cv2.MARKER_CROSS,
                    markerSize=14,
                    thickness=2,
                )

            cv2.imshow("stem target", bgr)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break

            if result.boxes is not None and len(result.boxes) > 0:
                if args.debug:
                    confidences = result.boxes.conf.cpu().numpy()
                    #print(
                    #    f"[debug] frame {frame_number}: {len(result.boxes)} detection(s), "
                    #    f"confidence range {confidences.min():.3f}-{confidences.max():.3f}",
                    #    flush=True,
                    #)
                    for index, box in enumerate(result.boxes):
                        coords = [round(value, 1) for value in box.xyxy[0].tolist()]
                        # print(
                        #     f"[debug]   box {index}: conf={float(box.conf[0]):.3f}, "
                        #     f"xyxy={coords}",
                        #     flush=True,
                        # )

                x1, y1, x2, y2 = map(float, best.xyxy[0].tolist())

                x_center = (x1 + x2) / 2.0
                stem_y = y2 - (y2 - y1) / 10.0

                camera_xyz, depth_sample_count = depth_to_camera_xyz(depth_frame, x_center, stem_y, color_frame=color_frame)
                if camera_xyz is None:
                    print(
                        f"Depth unavailable around stem target pixel "
                        f"({x_center:.1f}, {stem_y:.1f}); no valid samples in 9x9 window.",
                        flush=True,
                    )
                    continue

                if camera_xyz[2] > MAX_TARGET_Z_MM:
                    print(
                        f"Target Z {camera_xyz[2]:.1f} mm exceeds maximum "
                        f"{MAX_TARGET_Z_MM:.1f} mm; skipping target.",
                        flush=True,
                    )
                    continue

                robot_xyz = camera_to_robot(camera_xyz)

                # --- make sure it's the same weed for a few frames (avoid false hits) ---
                if confirm_xyz is not None and np.linalg.norm(robot_xyz - confirm_xyz) <= CONFIRM_RADIUS_MM:
                    confirm_count += 1
                    confirm_xyz = 0.5 * (confirm_xyz + robot_xyz)
                else:
                    confirm_count = 1
                    confirm_xyz = robot_xyz.copy()
                if confirm_count < CONFIRM_FRAMES:
                    continue
                target_xyz = confirm_xyz.copy()
                confirm_count, confirm_xyz = 0, None

                print("\n================ WEED FOUND: SCANNING PAUSED ================")
                print(f"detection_confidence: {float(best.conf[0]):.3f}")
                print(f"stem_pixel: ({x_center:.1f}, {stem_y:.1f}, {camera_xyz[2]:.1f})")
                print(f"valid_depth_samples: {depth_sample_count}")
                print("camera_xyz:", np.round(camera_xyz, 2))
                print("robot_xyz (target):", np.round(target_xyz, 2))

                steps, problem = plan_pick(target_xyz)
                for label, pose in steps:
                    print(f"  plan {label}: {build_move_command(pose)}")

                if problem:
                    print(f"NOT picking: {problem}", flush=True)
                    cv2.waitKey(1)
                    time.sleep(DETECTION_WAIT_SECONDS)
                    print("Scanning again.\n", flush=True)
                    continue

                if args.dry_run:
                    print("Dry-run: plan only, arm not moved.")
                else:
                    assert ser is not None
                    run_pick(ser, steps)
                    picked += 1
                    print(f"Weed #{picked} in the wastebasket.", flush=True)

                print(f"Waiting {args.wait_after:.0f} s before scanning again...", flush=True)
                if not paused_wait(args.wait_after):
                    break
                # throw away camera frames that were queued while the arm was moving
                drain_end = time.time() + 0.5
                while time.time() < drain_end:
                    pipeline.wait_for_frames(100)
                print("================ SCANNING RESUMED ================\n", flush=True)

            elif args.debug:
                print(f"[debug] frame {frame_number}: no detections", flush=True)

    finally:
        cv2.destroyAllWindows()
        try:
            pipeline.stop()
        finally:
            if ser is not None and ser.is_open:
                ser.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped (Ctrl+C).")
