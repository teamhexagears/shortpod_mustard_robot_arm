# Smart Weeding Robot Arm

This project controls a small Arduino servo robot arm. The Arduino sketch is in [`arduino/main.ino`](arduino/main.ino).

The arm has seven servo objects:

| Arduino Pin | Joint |
| ---: | --- |
| `3` | Claw |
| `4` | Tilt Wrist |
| `5` | Twist Wrist |
| `8` | Elbow |
| `9` | Arm 1 |
| `10` | Arm 2 (Mirror on Arm 1) |
| `11` | Base |

## Arduino Functions

## Serial Commands

Set the Serial Monitor to **9600 baud** with **Newline** line ending.

| Command | Action |
| --- | --- |
| `HELP` | Print the command list |
| `SHOW` | Print the current pose |
| `HOME` | Move to the HOME pose |
| `SLEEP` | Move to the HOME pose |
| `WAKEUP` | Run the Wakeup sequence |
| `FLOWER` | Run the Flower sequence |
| `TEST` | Move the Base through a short test sequence |
| `C+` / `C-` | Move the Claw |
| `TI+` / `TI-` | Move the Tilt Wrist |
| `TW+` / `TW-` | Move the Twist Wrist |
| `E+` / `E-` | Move the Elbow |
| `A+` / `A-` | Move Arm 1 and mirrored Arm 2 |
| `B+` / `B-` | Move the Base |

### `doActionFlower()`

Runs the saved Flower movement sequence by setting several joints to preset angles with pauses between movements.

### `doActionWakeup()`

Runs the saved Wakeup movement sequence. It moves several joints back and forth, then returns the Twist Wrist to its final position.

### `applyPose()`

Checks every joint angle against its safety limits, writes the angles to the servos, mirrors Arm 1 to Arm 2, and prints the current pose.

### `showPosition()`

Prints the current commanded angle for every joint to the Serial Monitor.

### `showHelp()`

Prints the available Serial commands and movement instructions.

### `goHome()`

Copies the saved HOME angle for each joint into the current angle variables, applies the pose, and reports that the arm moved HOME.

### `moveJoint(string jointName, int direction)`

Moves one named joint by a relative number of degrees.

The function then applies the safety limits and moves the servos.

### `turn_motor(string jointName, int angle, int delay_ms)`

Sets one joint to a specific angle, applies the pose, and waits for the requested number of milliseconds. The current implementation identifies joints with numeric indexes:

### `processCommand(char *cmd)`

Interprets a complete Serial command and calls the appropriate function. It handles general commands, action sequences, HOME, and individual joint movement commands.

### `setup()`

Runs once when the Arduino starts. It starts Serial communication, attaches each servo to its pin, moves the arm HOME, and prints the help text and current pose.


## Safety Variables

Each joint has three important values:

- `*MinAngle`: the lowest allowed angle
- `*MaxAngle`: the highest allowed angle
- `*HomeAngle`: the saved HOME position

The current angles use the HOME values when the Arduino starts. Change the limits only after checking the physical robot arm carefully.

## Important Notes

- Arm 2 is controlled automatically with `180 - arm1Angle`.
- `processCommand()` calls `showLimits()`, but the current `main.ino` does not contain a `showLimits()` function definition. The `LIMITS` command therefore needs that function restored or added before compiling successfully.
- The file includes `<iostream>` and `<string>` for `std::string`. Arduino projects may need the correct C++/Arduino IntelliSense configuration for these headers.
- The opening comment in `main.ino` still lists `W1` and `W2`, while the current command parser uses `TI` and `TW`. The command table above documents the parser's current behavior.
