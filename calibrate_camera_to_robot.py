#!/usr/bin/env python3
"""
Camera-to-robot calibration for the Shortpod Mustard weeding arm.

Creates camera_to_robot_transform.json (R, t) so that

    robot_xyz = R @ camera_xyz + t

using the "touch point" method:
  1. Place a small marker in the work area (e.g. a LEGO stack or bottle cap).
  2. Click its center in the camera window -> camera_xyz (from aligned depth).
  3. Jog the arm until the middle of the claw jaws is at the marker -> robot_xyz (from FK).
  4. Repeat for 6-10 markers, then fit R, t with the Kabsch / SVD method.

It reuses the calibrated arm math (FK/IK) and depth code from stem_to_robot_move.py,
so keep both files in the same folder.

Usage:
  python calibrate_camera_to_robot.py                 # full interactive calibration
  python calibrate_camera_to_robot.py --recompute     # refit from saved points
  python calibrate_camera_to_robot.py --recompute --exclude 3,7

IMPORTANT: close the Arduino Serial Monitor first, and do not move the camera after
calibrating. If the camera moves, run this again.
"""
import argparse
import json
import shutil
import time
from pathlib import Path

import cv2
import numpy as np
import serial

import stem_to_robot_move as arm

ROOT = Path(__file__).resolve().parent
TRANSFORM_PATH = ROOT / "camera_to_robot_transform.json"
POINTS_PATH = ROOT / "camera_calibration_points.json"

JOINTS = ["base", "arm1", "elbow", "tiltWrist"]
JOG_KEYS = {"b": "base", "a": "arm1", "e": "elbow", "t": "tiltWrist", "c": "claw"}

# Starting pose for jogging when no transform guess is available yet:
# arm straight ahead, reaching forward and low (claw ~380 mm forward, ~110 mm high).
START_POSE = {"base": 15, "arm1": 80, "elbow": 120, "tiltWrist": 15}
HOVER_MM = 40.0          # when a guess exists, first move this far ABOVE the marker
DEPTH_SAMPLES = 15       # frames averaged per click
GOOD_MM = 10.0
MIN_Z_MM = 5.0           # refuse poses where the claw/wrist/elbow would go below the table
MAX_JOG_STEP = 10        # degrees per jog command


# ---------------------------------------------------------------------------
# Math
# ---------------------------------------------------------------------------
def kabsch(cam_pts, rob_pts):
    """Best rigid transform robot = R @ cam + t (least squares, no scaling/mirroring)."""
    P = np.asarray(cam_pts, dtype=float)
    Q = np.asarray(rob_pts, dtype=float)
    cp, cq = P.mean(axis=0), Q.mean(axis=0)
    H = (P - cp).T @ (Q - cq)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, 1.0, d])
    R = Vt.T @ D @ U.T
    t = cq - R @ cp
    return R, t


def residuals(R, t, cam_pts, rob_pts):
    pred = (R @ np.asarray(cam_pts, float).T).T + t
    return np.linalg.norm(pred - np.asarray(rob_pts, float), axis=1)


def spread_warning(cam_pts):
    P = np.asarray(cam_pts, float)
    sv = np.linalg.svd(P - P.mean(axis=0), compute_uv=False)
    if sv[1] < 0.15 * sv[0]:
        return "Points are almost on one LINE. Spread markers left/right AND near/far."
    return None


def fit_and_report(points, exclude=()):
    used = [p for p in points if p["id"] not in exclude]
    if len(used) < 4:
        print(f"Need at least 4 points (have {len(used)}).")
        return None
    cam = [p["camera_xyz"] for p in used]
    rob = [p["robot_xyz"] for p in used]
    R, t = kabsch(cam, rob)
    err = residuals(R, t, cam, rob)

    print("\n================ CALIBRATION RESULT ================")
    print(f"{'id':>3} {'camera_xyz (mm)':>26} {'robot_xyz (mm)':>26} {'error':>7}")
    for p, e in zip(used, err):
        c = np.round(p["camera_xyz"], 1)
        r = np.round(p["robot_xyz"], 1)
        flag = "" if e <= GOOD_MM else "  <-- check"
        print(f"{p['id']:>3} {str(c):>26} {str(r):>26} {e:6.1f}{flag}")
    rms = float(np.sqrt(np.mean(err ** 2)))
    print(f"\nPoints used: {len(used)}   mean: {err.mean():.1f} mm   max: {err.max():.1f} mm   RMS: {rms:.1f} mm")
    if rms <= GOOD_MM:
        print("Result: GOOD (RMS <= 10 mm).")
    elif rms <= 20:
        print("Result: OK. If one point is much worse than the rest, re-run with --recompute --exclude <id>.")
    else:
        print("Result: POOR. Check that the camera did not move, the claw middle was really on")
        print("        each marker, and the depth reading was valid (not on an edge/shadow).")
    w = spread_warning(cam)
    if w:
        print("WARNING:", w)
    print("====================================================")
    return R, t, err, rms, [p["id"] for p in used]


def save_transform(R, t, rms, used_ids):
    if TRANSFORM_PATH.exists():
        backup = TRANSFORM_PATH.with_suffix(time.strftime(".%Y%m%d_%H%M%S.bak.json"))
        shutil.copy(TRANSFORM_PATH, backup)
        print(f"Old transform backed up to {backup.name}")
    data = {
        "R": R.tolist(),
        "t": t.tolist(),
        "rms_mm": round(rms, 2),
        "points_used": used_ids,
        "created": time.strftime("%Y-%m-%d %H:%M:%S"),
        "note": "robot_xyz = R @ camera_xyz + t  (mm). Redo if the camera or arm moves.",
    }
    with open(TRANSFORM_PATH, "w") as f:
        json.dump(data, f, indent=2)
    print(f"Saved {TRANSFORM_PATH.name}")


def save_points(points):
    with open(POINTS_PATH, "w") as f:
        json.dump(points, f, indent=2)


# ---------------------------------------------------------------------------
# Arduino
# ---------------------------------------------------------------------------
def open_serial(port):
    try:
        ser = serial.Serial(port, 9600, timeout=0.5, exclusive=True)
    except (serial.SerialException, OSError) as exc:
        print(f"Cannot open {port}: {exc}")
        print(arm.SERIAL_HELP.format(port=port))
        raise SystemExit(1)
    time.sleep(2)  # opening the port resets the Uno
    ser.reset_input_buffer()
    return ser


def wait_for(ser, token, timeout_s=15.0):
    end = time.time() + timeout_s
    while time.time() < end:
        try:
            line = ser.readline().decode("ascii", errors="ignore").strip()
        except serial.SerialException as exc:
            print(f"SERIAL ERROR: {exc}")
            print(arm.SERIAL_HELP.format(port=ser.port))
            raise SystemExit(1)
        if token in line:
            return True
    print(f"  WARNING: no '{token}' reply from Arduino.")
    return False


def clamp_pose(pose):
    out = dict(pose)
    for name in JOINTS + ["claw"]:
        lo, hi = arm.JOINT_LIMITS[name]
        out[name] = int(round(min(max(out[name], lo), hi)))
    return out


def send_pose(ser, pose):
    pose = clamp_pose(pose)
    cmd = f"MOVE {pose['base']} {pose['arm1']} {pose['elbow']} 90 {pose['tiltWrist']} {pose['claw']}"
    if ser is None:
        print(f"  (dry-run) {cmd}")
        return pose
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode("ascii"))
    ser.flush()
    wait_for(ser, "Moved to absolute pose")
    time.sleep(0.3)
    return pose


def send_home(ser):
    if ser is None:
        print("  (dry-run) HOME")
        return
    ser.reset_input_buffer()
    ser.write(b"HOME\n")
    ser.flush()
    wait_for(ser, "Moved to HOME pose")
    time.sleep(0.3)


# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------
class Camera:
    def __init__(self):
        self.pipeline = arm.Pipeline()
        config = arm.Config()
        color_profiles = self.pipeline.get_stream_profile_list(arm.OBSensorType.COLOR_SENSOR)
        depth_profiles = self.pipeline.get_stream_profile_list(arm.OBSensorType.DEPTH_SENSOR)
        config.enable_stream(color_profiles.get_default_video_stream_profile())
        config.enable_stream(depth_profiles.get_default_video_stream_profile())
        self.pipeline.start(config)
        self.align = arm.AlignFilter(align_to_stream=arm.OBStreamType.COLOR_STREAM)

    def read(self):
        frames = self.pipeline.wait_for_frames(1000)
        if frames is None:
            return None, None, None
        frames = self.align.process(frames)
        if frames is None:
            return None, None, None
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        if color is None or depth is None:
            return None, None, None
        bgr = arm.frame_to_bgr_image(color)
        return bgr, color, depth

    def stop(self):
        self.pipeline.stop()


def capture_marker(cam, point_id, known_points):
    """Show the live image; user clicks the marker center. Returns camera_xyz or None."""
    win = f"Point {point_id}: click marker center | ENTER=accept  R=redo  Q=finish"
    click = {"px": None}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            click["px"] = (x, y)

    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    result = None
    samples = []
    last_px = None

    while True:
        bgr, color, depth = cam.read()
        if bgr is None:
            continue
        view = bgr.copy()
        for p in known_points:                      # already used markers
            u, v = p["pixel"]
            cv2.circle(view, (int(u), int(v)), 6, (255, 160, 0), 2)
            cv2.putText(view, str(p["id"]), (int(u) + 8, int(v) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 160, 0), 1)

        if click["px"] is not None:
            if click["px"] != last_px:
                samples, result, last_px = [], None, click["px"]
            u, v = click["px"]
            if len(samples) < DEPTH_SAMPLES:
                xyz, n = arm.depth_to_camera_xyz(depth, u, v, radius=3, color_frame=color)
                if xyz is not None:
                    samples.append(xyz)
            if len(samples) >= DEPTH_SAMPLES:
                s = np.array(samples)
                result = np.median(s, axis=0)
                spread = float(np.max(np.linalg.norm(s - result, axis=1)))
                txt = f"cam xyz = {np.round(result).astype(int).tolist()} mm  (noise {spread:.0f} mm)"
                col = (0, 200, 0) if spread < 8 else (0, 140, 255)
            else:
                txt = f"reading depth... {len(samples)}/{DEPTH_SAMPLES}"
                col = (0, 200, 255)
            cv2.drawMarker(view, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 18, 2)
            cv2.putText(view, txt, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
        else:
            cv2.putText(view, "Click the marker center", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

        cv2.imshow(win, view)
        key = cv2.waitKey(1) & 0xFF
        if key in (13, 10, 32) and result is not None:   # Enter / Space
            break
        if key in (ord("r"), ord("R")):
            click["px"], samples, result, last_px = None, [], None, None
        if key in (ord("q"), ord("Q"), 27):
            result = None
            click["px"] = None
            break

    cv2.destroyWindow(win)
    cv2.waitKey(1)
    if result is None:
        return None, None
    return result, click["px"]


# ---------------------------------------------------------------------------
# Jogging
# ---------------------------------------------------------------------------
JOG_HELP = """
Jog the arm until the MIDDLE of the claw jaws is at the marker center.
  b+2  b-5     base      (+ = servo up = turns RIGHT)
  a+2  a-2     arm1      (shoulder)
  e+2  e-2     elbow
  t+2  t-2     tiltWrist
  c+5  c-5     claw open/close
  m B A E T    absolute move, e.g.  m 15 85 115 40
  show         print pose and FK position
  ok           accept this point
  skip         skip this marker
"""


def pose_is_safe(pose):
    pts = arm.fk_points_from_servo(pose)
    for name in ("elbow", "tilt_wrist", "claw_middle"):
        if pts[name][2] < MIN_Z_MM:
            return False, f"{name} would be at z = {pts[name][2]:.0f} mm (below the table)"
    return True, ""


def jog_to_marker(ser, pose):
    print(JOG_HELP)
    ok, why = pose_is_safe(clamp_pose(pose))
    if not ok:
        print(f"  Start pose not safe ({why}); using the default start pose.")
        pose = dict(START_POSE, claw=pose.get("claw", 150))
    pose = send_pose(ser, pose)
    while True:
        fk = arm.fk_from_servo(pose)
        print(f"  pose: base {pose['base']} arm1 {pose['arm1']} elbow {pose['elbow']} "
              f"tilt {pose['tiltWrist']} claw {pose['claw']}   ->  claw middle "
              f"x={fk[0]:.0f} y={fk[1]:.0f} z={fk[2]:.0f} mm")
        cmd = input("  jog> ").strip().lower()
        if cmd == "ok":
            return pose
        if cmd == "skip":
            return None
        if cmd in ("show", ""):
            continue
        if cmd.startswith("m "):
            try:
                b, a, e, t = [float(v) for v in cmd[2:].split()]
                new = dict(pose, base=b, arm1=a, elbow=e, tiltWrist=t)
            except ValueError:
                print("  usage: m BASE ARM1 ELBOW TILT")
                continue
        elif len(cmd) >= 3 and cmd[0] in JOG_KEYS and cmd[1] in "+-":
            try:
                step = int(cmd[2:])
            except ValueError:
                print("  usage: e+2   (joint letter, + or -, degrees)")
                continue
            if step > MAX_JOG_STEP:
                print(f"  max {MAX_JOG_STEP} degrees per step (use 'm' for big moves)")
                continue
            name = JOG_KEYS[cmd[0]]
            new = dict(pose)
            new[name] = pose[name] + (step if cmd[1] == "+" else -step)
        else:
            print("  unknown command (type: b+2, e-2, m 15 85 115 40, ok, skip)")
            continue
        clamped = clamp_pose(new)
        if clamped != {k: int(round(v)) for k, v in new.items()}:
            print("  (clamped to the Arduino safety limits)")
        ok, why = pose_is_safe(clamped)
        if not ok:
            print(f"  REFUSED: {why}")
            continue
        pose = send_pose(ser, clamped)


def guess_pose(points, camera_xyz, claw):
    """With >= 3 points, predict where the marker is and hover above it."""
    if len(points) < 3:
        return None
    R, t = kabsch([p["camera_xyz"] for p in points], [p["robot_xyz"] for p in points])
    target = R @ np.asarray(camera_xyz) + t
    hover = target + np.array([0.0, 0.0, HOVER_MM])
    q = arm.ik_point(hover)
    if np.linalg.norm(arm.fk(q) - hover) > 15:
        print(f"  Guess {np.round(target).astype(int).tolist()} mm is out of reach; jog manually.")
        return None
    pose = arm.q_to_pose_deg(q)
    print(f"  Predicted marker at {np.round(target).astype(int).tolist()} mm; moving {HOVER_MM:.0f} mm above it.")
    return {"base": pose["base"], "arm1": pose["arm1"], "elbow": pose["elbow"],
            "tiltWrist": pose["tiltWrist"], "claw": claw}


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def interactive(args):
    points = []
    if args.resume and POINTS_PATH.exists():
        points = json.load(open(POINTS_PATH))
        print(f"Resumed {len(points)} saved points from {POINTS_PATH.name}")

    ser = None if args.dry_run else open_serial(args.port)
    cam = Camera()
    claw = args.claw
    last_pose = dict(START_POSE, claw=claw)

    print("""
================ CAMERA -> ROBOT CALIBRATION ================
For each point:
  1. The arm goes HOME (out of the camera view).
  2. Put a marker in the work area: 270-450 mm from the base, straight ahead or
     to the RIGHT (the arm only turns ~14 deg left). Use different heights too
     (e.g. LEGO stacks 1-4 bricks tall).
  3. Click the marker CENTER in the camera window, press ENTER.
  4. Jog the arm until the middle of the claw jaws is at that same marker point.
     Type 'ok'.
Aim for 6-10 points spread left/right AND near/far. Press Q in the camera
window when you are done.
SAFETY: keep one hand on the servo power switch.
==============================================================""")

    try:
        while True:
            point_id = (max([p["id"] for p in points]) + 1) if points else 1
            send_home(ser)
            input(f"\n[Point {point_id}] Place the marker, then press ENTER to open the camera...")
            camera_xyz, pixel = capture_marker(cam, point_id, points)
            if camera_xyz is None:
                break
            print(f"  camera_xyz = {np.round(camera_xyz, 1).tolist()} mm at pixel {pixel}")

            start = guess_pose(points, camera_xyz, claw) or last_pose
            pose = jog_to_marker(ser, start)
            if pose is None:
                print("  skipped.")
                continue
            robot_xyz = arm.fk_from_servo(pose)
            points.append({
                "id": point_id,
                "pixel": [int(pixel[0]), int(pixel[1])],
                "camera_xyz": [float(v) for v in camera_xyz],
                "servo": {k: int(pose[k]) for k in JOINTS},
                "robot_xyz": [float(v) for v in robot_xyz],
            })
            last_pose = dict(pose)
            save_points(points)
            print(f"  saved point {point_id}: robot_xyz = {np.round(robot_xyz, 1).tolist()} mm "
                  f"({len(points)} points total)")
            if len(points) >= 4:
                R, t = kabsch([p["camera_xyz"] for p in points], [p["robot_xyz"] for p in points])
                err = residuals(R, t, [p["camera_xyz"] for p in points], [p["robot_xyz"] for p in points])
                print(f"  running fit: RMS {np.sqrt(np.mean(err ** 2)):.1f} mm")
    finally:
        if ser is not None:
            send_home(ser)
        cv2.destroyAllWindows()
        cam.stop()
        if ser is not None and ser.is_open:
            ser.close()

    finish(points, exclude=())


def finish(points, exclude):
    res = fit_and_report(points, exclude)
    if res is None:
        return
    R, t, err, rms, used = res
    ans = input("\nSave this as camera_to_robot_transform.json? [y/N] ").strip().lower()
    if ans == "y":
        save_transform(R, t, rms, used)
        print("Next: python stem_to_robot_move.py --dry-run, put a weed in view, and check that")
        print("robot_xyz matches where the weed really is (ruler from the base center).")
    else:
        print("Not saved. Points are kept in", POINTS_PATH.name, "- use --recompute later.")


def parse_args():
    ap = argparse.ArgumentParser(description="Camera-to-robot touch-point calibration")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--dry-run", action="store_true", help="no Arduino; print MOVE commands only")
    ap.add_argument("--claw", type=int, default=150, help="claw servo value while calibrating")
    ap.add_argument("--resume", action="store_true", help="continue adding to saved points")
    ap.add_argument("--recompute", action="store_true", help="refit from saved points (no camera/arm)")
    ap.add_argument("--exclude", default="", help="comma-separated point ids to leave out, e.g. 3,7")
    return ap.parse_args()


def main():
    args = parse_args()
    exclude = {int(v) for v in args.exclude.split(",") if v.strip()}
    if args.recompute:
        if not POINTS_PATH.exists():
            print(f"No {POINTS_PATH.name} found. Run the calibration first.")
            return
        finish(json.load(open(POINTS_PATH)), exclude)
        return
    interactive(args)


if __name__ == "__main__":
    main()
