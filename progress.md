# Smart Weeding Robot (智慧拔草機) – Progress

FLL Innovation Project · Last updated: 2026-10-13

## Goal

A camera finds a Shortpod Mustard plant, and the robot arm grabs it by the stem, pulls it out and drops it in a wastebasket. Then it waits and starts scanning again.

```
camera image -> YOLO finds the plant -> stem pixel + depth -> camera XYZ (mm)
  -> camera_to_robot_transform.json -> robot XYZ (mm)
  -> inverse kinematics -> servo angles -> USB serial "MOVE ..." -> Arduino -> servos
```

### Status
| Part | Status |
|---|---|
| Plant detection (YOLO `best_v16.pt`) | Working |
| Stem pixel → camera XYZ (aligned depth) | Working |
| Arm math (FK/IK) checked against real measurements | Working, average error 15.5 mm |
| Camera-to-robot calibration | Done (touch-point calibration script) |
| Pick-and-place cycle (pause → grab → bin → wait → scan) | Working, being fine-tuned on the real arm |
| Lagging issue | **Open**, see the "Next steps" section |

---

## 1. Hardware

| Part | Details |
|---|---|
| Arm | Custom-built arm with 7 servos and 6 joints. Motors 2 and 3 move together as one joint (arm1/arm2) |
| Controller | Arduino Uno R3, `Servo.h` library, USB serial at 9600 baud |
| Arduino sketch | `/home/admin/git/shortpod_mustard_robot_arm/arduino/main.ino` (shared as `shortpod_mustard_robot_arm.ino`) |
| Computer | Raspberry Pi 5, Python 3.13 venv, port `/dev/ttyACM0` |
| Camera | Orbbec Gemini 330 series (Gemini 335), color 1706 × 960 |
| Camera mount | Fixed, in front of the robot about 20 cm to the left of the base motor, about 21 cm high. Points straight forward, level with the table, image top = robot forward |
| Servo power | 5 V 15 A supply. Worst case is about 8.1 A, so there's enough margin |
| Work surface | White table = z 0. The arm sits on 17 mm plywood |

### Servos

| Motor | Pin | Joint | Motion |
|---|---|---|---|
| 1 | D11 | base | Turns left/right. A larger value turns it RIGHT, seen from behind |
| 2 + 3 | D9 + D10 | arm1 / arm2 (shoulder) | Upper arm up/down. arm2 = 180 − arm1 (mirrored) |
| 4 | D6 | elbow | Forearm up/down. A larger value moves the forearm DOWN |
| 5 | D4 | tiltWrist | Claw up/down (pitch). A larger value tilts the claw DOWN |
| 6 | D5 | twistWrist | Claw rotation only |
| 7 | D3 | claw | Open (150) / closed (100) |

MOVE order: `MOVE base arm1 elbow twist tilt claw`

### Arduino limits and HOME (current sketch)

| Joint | Min | Max | HOME |
|---|---|---|---|
| base | 0 | **180** (was 160) | 30 |
| arm1 | 30 | 150 | 30 |
| elbow | 10 | 150 | 150 |
| tiltWrist | **0** (was 10) | **100** (was 70) | 50 |
| twistWrist | 0 | 179 | 90 |
| claw | 100 | 179 | 150 |

---

## 2. Coordinate frame (robot)

- **Origin:** the point on the white table directly under the base rotation center.
- **+x:** forward along the tape line, where the arm points at base servo 15.
- **+y:** to the LEFT of the tape line, seen from behind the robot. Right = negative.
- **+z:** up from the white table.
- **Units:** mm. The target point is the middle between the two claw jaws.

---

## 3. Arm model (measured, in `stem_to_robot_move.py`)

The first numbers in the AI notes were estimates. Plumb-line measurements replaced them:

| Item | First estimate (AI notes) | Measured and used now |
|---|---|---|
| Shoulder height above table | 6 cm | **127 mm** |
| Shoulder → elbow | 22 cm | **L1 ≈ 223 mm** (L-shaped link: 112 mm forward, 193 mm up) |
| Elbow → wrist | 15 cm | **L2 = 140 mm** |
| Wrist → claw middle | 7 cm | **L3 ≈ 86 mm** (85 mm forward, 15 mm up, so the line sits about 10° above the forearm) |

Reference pose `MOVE 10 50 110 90 15 150`: shoulder (0, 127), elbow (112, 320), wrist (250, 320), claw (335, 335).

| Joint | Calibration |
|---|---|
| base | Table: servo 10 = +4.6°, 15 = 0° (straight ahead), 40 = −19.3°, 70 = −40.2° |
| arm1 | Linear, servo 50 = upper-arm line at 59.9°, 1° per servo degree, direction −1 |
| elbow | 9-point sweep table: 50 to 130 servo → forearm +56° to −19°. `ELBOW_DIRECTION = 1` (checked again 2026-10-13) |
| tiltWrist | Table: 15 → 0°, 40 → −20°, 70 → −45° (claw vs forearm). `TILT_HORN_SHIFT_DEG = 0` |

### FK verification history
| Run | Mean error | What we fixed |
|---|---|---|
| 1 | 289 mm | Base direction, x measured from the wrong point |
| 2 | 44.9 mm | L-shaped upper arm, claw offset |
| 3 | **15.5 mm** | Base table (servo 15 = straight ahead) |

### Reach
On the table, the arm reaches about 270–450 mm from the base. It's most accurate at 300–400 mm, straight ahead to about 100° right. It can only turn about 14° left.

---

## 4. Camera-to-robot calibration

- **Old method:** 6 points in the AI notes (camera xyz + 6 servo values). These became wrong once the arm model changed.
- **New method:** `calibrate_camera_to_robot.py`.
  1. Click a marker (a LEGO tower at stem height) in the camera window.
  2. Jog the claw onto it (`b+2`, `a-2`, `e+2`, `t-2`, `m B A E T`) and type `ok`.
  3. Repeat for 6–10 points, 300–380 mm out.
  4. The script fits R and t with Kabsch/SVD.
  - Output: `camera_to_robot_transform.json` and `camera_calibration_points.json`.
  - Options: `--resume`, `--recompute --exclude <id>`, `--no-limits` (Python limits off; the below-table check stays on).
- **Status:** completed 2026-10-07.

Example arm positions from the AI notes (pose → claw position from the arm math):
```
base 30 arm1 78 elbow 130 tilt 45 -> x=320 y=-66  z=67
base 0  arm1 84 elbow 130 tilt 24 -> x=330 y=81   z=46
base 20 arm1 95 elbow 81  tilt 40 -> x=423 y=-29  z=97
base 45 arm1 100 elbow 103 tilt 25 -> x=354 y=-149 z=9
base 55 arm1 70 elbow 130 tilt 20 -> x=306 y=-175 z=135
base 33 arm1 89 elbow 86  tilt 31 -> x=410 y=-101 z=133
base 25 arm1 86 elbow 107 tilt 20 -> x=394 y=-53  z=100
```

---

## 5. Detection rules

| Rule | Setting |
|---|---|
| Point used | Stem: box center x, 10% above the bottom of the box |
| When to act | Only when stable: the same spot (within 20 mm) in 3 frames in a row (`CONFIRM_FRAMES = 3`) |
| Low confidence | Ignored (`--conf`, e.g. 0.3) |
| No depth / too far (z > 600 mm) | Do nothing |
| Out of reach | Do nothing (`NOT picking: ...`) |
| Debug output | x, y, z for the pixel, camera and robot frames. Servo angles rounded to whole degrees |

---

## 6. Pick-and-place cycle (current)

1. Weed confirmed → **SCANNING PAUSED**. Every move is planned and safety-checked first.
2. Move above the weed, a little short of it, with the claw open.
3. Go down and forward onto the stem in one smooth move.
4. Close the claw (100).
5. Pull the weed up 100 mm. The claw starts tilting up.
6. Raise the arm HIGH with the claw tilted up (`HOLD_TILT = 0`), so the weed doesn't fall.
7. Turn to the wastebasket while high, so the arm doesn't hit the side of the box.
8. Lower into the wastebasket, claw still tilted up.
9. Tilt the claw down over the bin.
10. Open the claw to drop the weed.
11. Raise high again, turn back while high.
12. HOME → wait (`--wait-after`, default 10 s) → **SCANNING RESUMED**.

### Smooth motion
- **Arduino:** `MOVE` is coordinated, so all joints start and finish at the same time. It doesn't print the full pose after a Python `MOVE`, so moves don't pause. At startup the sketch prints `Sketch version: 2026-10-13 coordinated MOVE`.
- **Python:** one move per step (`PATH_STEP_MM = 0`). Each step waits for the Arduino's `Moved to absolute pose` reply.

### Claw angle
The program picks the most level claw angle the arm can reach (`FACE_PLANT = True`). A fully level claw isn't possible at table height, because the wrist servo hits 0. The fix for that is to re-mount the claw horn 1–2 spline teeth and set `TILT_HORN_SHIFT_DEG`.

### Current tuning values

| Setting | Value | Meaning / history |
|---|---|---|
| `GRAB_FORWARD_MM` | **40** | Farther from the base: −30 → −10 → 10 → 20 → 30 → 40 |
| `GRAB_LEFT_MM` | **15** | It was landing right of the stem |
| `GRAB_Z_MM` | **23** | −22 too low, ~+98 too high, then 38, then 23 |
| `CLAW_OPEN` / `CLAW_CLOSED` | 150 / 100 | Measured |
| `TWIST_ANGLE` | 90 | |
| `APPROACH_MM` / `LIFT_MM` | 50 / 100 | Hover height / pull-up height |
| `HOLD_TILT` | 0 | Claw tilted up while holding the weed |
| `CARRY_POSE` | (30, 130, 30) | arm1, elbow, tilt for turning high. Claw about 350 mm up |
| `BIN_POSE` | **(170, 50, 150, 50)** | Base turned farther (was 160) |
| `MIN_Z_MM` / `MAX_RAISE_MM` | 5 / 80 | Never below the table: the target is raised automatically if needed |
| `ELBOW_DIRECTION` | 1 | −1 made the grab about 120 mm too high |

---

## 7. Timeline

| Date | What we did |
|---|---|
| Before Oct | First AI notes: joint list, camera setup, 6 calibration points, kinematics math, stem detection → depth → transform → IK → MOVE, debug x/y/z, rounded angles, z limit |
| Oct 5–6 | Learned DH vs. simple planar model. Added servo↔math angle mapping (`CAL`), `--check-pose`, `--verify-fk`. Measured the base, elbow sweep, wrist, plumb-line geometry. FK error 289 → 45 → **15.5 mm** |
| Oct 6 | Found the Serial Monitor was blocking the port. Checked servo buzzing and power |
| Oct 7 | Wrote `calibrate_camera_to_robot.py`. Marker tips: LEGO towers at stem height. Safety: refuse below-table moves, max jog step. **Camera calibration completed.** Wrote the first progress file |
| Oct 8 | Added a `--no-limits` option to the calibration script |
| Oct 11 | Pick-and-place cycle: pause → grab → wastebasket → wait → scan. Bin pose, claw closed = 100. High carry so the arm doesn't hit the box. Fixed "Device or resource busy" (Arduino IDE `serial-monitor` holding the port) |
| Oct 11–13 | Grab tuning: forward slide, wrist tilt limit raised to 100, then claw-facing angle search. Never below the table (auto-raise). Straight-line path, then coordinated Arduino `MOVE` (all joints together, no stops). Tilt min 0. Elbow direction test (back to +1). Grab height/forward/left corrections. Claw tilted up while carrying. Base to 170 for the bin (limit 180) |

---

## 8. Files

| File | Purpose |
|---|---|
| `arduino/main.ino` (`shortpod_mustard_robot_arm.ino`) | Servo control, limits, HOME, jog commands, coordinated `MOVE` |
| `stem_to_robot_move.py` | Detection → depth → transform → IK → pick-and-place cycle. All arm calibration and tuning settings |
| `calibrate_camera_to_robot.py` | Camera-to-robot touch-point calibration |
| `camera_to_robot_transform.json` | R, t (robot = R · camera + t) |
| `camera_calibration_points.json` | Raw touch points (refit with `--recompute`) |
| `fk_verify_*.csv` | FK check results (presentation evidence) |
| `images/IMG_6519.JPG`, `IMG_6521.JPG`, `IMG_6522.JPG` | Servo directions, top view, side view |

## 9. Commands

```bash
# run the robot (close the Arduino Serial Monitor first!)
python3 stem_to_robot_move.py --port /dev/ttyACM0 --conf 0.3 --weights ai_models/best_v16.pt --wait-after 5
python3 stem_to_robot_move.py --dry-run ...          # plan only, arm doesn't move

# checks
python3 stem_to_robot_move.py --check-pose 15 50 110 15   # expect claw about (337, 0, 335)
python3 stem_to_robot_move.py --verify-fk

# camera calibration
python3 calibrate_camera_to_robot.py [--resume | --recompute --exclude 4 | --no-limits]

# serial port busy?
fuser -v /dev/ttyACM0        # "serial-monitor" = close the Arduino IDE Serial Monitor tab
```

Stop with Q in the camera window or Ctrl+C, not Ctrl+Z, which pauses the program and keeps the port locked.

---

## 10. Next steps

- [ ] **Lagging issue.** First find out which part lags: the camera window, the detection, or the arm. Ideas to try:
  - Run without `--debug`, because printing every frame is slow.
  - Use a smaller YOLO input size, e.g. `--imgsz 320` or `416`. On a Pi 5 CPU this is the biggest speed-up.
  - Export the model to NCNN or ONNX for the Pi (`yolo export model=best_v16.pt format=ncnn`).
  - Lower the color resolution (e.g. 1280 × 720 or 640 × 480), or run detection only every 2nd–3rd frame.
  - Arm lag: upload the coordinated-`MOVE` sketch. Optionally raise the baud rate to 115200 on both sides, or lower `MOVE_STEP_DELAY_MS` (15 ms per degree now).
- [ ] Keep tuning `GRAB_FORWARD_MM`, `GRAB_LEFT_MM`, `GRAB_Z_MM` on real plants, 10 mm at a time.
- [ ] Check that the claw (tilted up) clears the box rim when lowering into the bin, and that base 170 is right.
- [ ] Optional: re-mount the claw horn 1–2 teeth so it can face the stem more level, then set `TILT_HORN_SHIFT_DEG` and measure the reference pose.
- [ ] Optional: wrist measurements at tilt 0, 5, 10 (the table starts at 15).
- [ ] Commit the working versions to Git: both scripts, both JSON files, and the sketch.
- [ ] FLL presentation: calibration story (289 → 15.5 mm), the pick-and-place video, lessons learned.

---

## 11. Lessons learned

1. A servo angle is not a math angle. Every joint needs an offset, a direction, and sometimes a table.
2. Measure the real geometry: the L-shaped upper arm and the claw offset caused errors of over 100 mm.
3. Use one coordinate frame everywhere: origin under the base, +x forward, +y left, +z up.
4. Change one thing at a time and re-test.
5. Correct the grab in millimeters, not servo degrees. Servo offsets break the joint coordination.
6. The Arduino moved each joint 1° at a time. Coordinated moves make the motion smooth.
7. Only one program can use the serial port. Close the Serial Monitor before running scripts.
8. Safety first: limits, the below-table check, out-of-reach skipping, and a hand on the power switch.
9. If the camera or base moves, recalibrate.

## 12. Skills learned

- **Robotics:** forward/inverse kinematics, DH parameters vs. a planar model, joint calibration tables, workspace and joint limits, coordinated motion, pick-and-place planning.
- **Computer vision:** YOLO detection, camera intrinsics, depth-to-color alignment, pixel + depth → 3D, Kabsch/SVD camera-to-robot calibration.
- **Measurement:** plumb line, phone level, protractor, error analysis (mean/max/RMS), sanity checks against physical limits.
- **Electronics:** servo power budget, common ground, serial ports and baud rate, servo stall and gear slip.
- **Software:** Python CLI tools, Arduino serial protocol, safety checks, debugging layer by layer, version control.
- **Teamwork and communication:** explaining the camera → transform → IK chain, showing improvement with data, checking AI suggestions against real measurements.
