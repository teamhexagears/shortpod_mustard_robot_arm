#include <Arduino.h>
#line 1 "/home/admin/git/shortpod_mustard_robot_arm/shortpod_mustard_robot_arm.ino"
/*
  Smart Weeding Robot Arm - Calibration Code

  Servo wiring:
  D3  = Claw
  D4  = Tilt Wrist
  D5  = Twist Wrist
  D6  = Elbow
  D9  = Arm 1
  D10 = Arm 2 (mirrored: 180 - Arm 1)
  D11 = Base

  Serial Monitor settings:
  - Baud rate: 9600
  - Line ending: Newline

  Commands:
  HELP
  SHOW
  LIMITS
  HOME
  WAKEUP
  TEST
  MOVE <base> <arm1> <elbow> <tilt> <twist> <claw>


  C+ / C-     = Claw
  W1+ / W1-   = Tilt Wrist
  W2+ / W2-   = Twist Wrist
  E+ / E-     = Elbow
  A+ / A-     = Arm 1; Arm 2 mirrors automatically
  B+ / B-     = Base
*/

/* Include Libraries */
#include <Servo.h>

/* Initialize Motors */
// ---------- Servo objects ----------
Servo clawServo;    // D3
Servo tiltWristServo;   // D4
Servo twistWristServo;  // D5
Servo elbowServo;   // D6
Servo arm1Servo;    // D9
Servo arm2Servo;    // D10, mirrored with Arm 1
Servo baseServo;    // D11

// ---------- Pin mapping ----------
const byte CLAW_PIN = 3;
const byte TILT_WRIST_PIN = 4;
const byte TWIST_WRIST_PIN = 5;
const byte ELBOW_PIN = 6;
const byte ARM1_PIN = 9;
const byte ARM2_PIN = 10;
const byte BASE_PIN = 11;

const int small_movement = 2;
const int mid_movement = 5;
const int big_movement = 10;

/*
---------- Calibration behavior ----------
  IMPORTANT:
  These are limits to reduce the risk of crashes.
  You MUST replace them with safe values measured on your own arm.
*/

// Minimum safe angles for each joint
int clawMinAngle = 100;
int tiltWristMinAngle = 0;
int twistWristMinAngle = 0;
int elbowMinAngle = 10;
int arm1MinAngle = 30;
int baseMinAngle = 0;

// Maximum safe angles for each joint
int clawMaxAngle = 179;
int tiltWristMaxAngle = 90;
int twistWristMaxAngle = 179;
int elbowMaxAngle = 130;
int arm1MaxAngle = 150;
int baseMaxAngle = 160;

// HOME position angles for each joint (reset position)
int clawHomeAngle = 150;
int tiltWristHomeAngle = 50;
int twistWristHomeAngle = 90;
int elbowHomeAngle = 90;
int arm1HomeAngle = 60;
int baseHomeAngle = 80;

// Initilal joint angles (start at HOME position)
int clawAngle = clawHomeAngle;
int tiltWristAngle = tiltWristHomeAngle;
int twistWristAngle = twistWristHomeAngle;
int elbowAngle = elbowHomeAngle;
int arm1Angle = arm1HomeAngle;
int baseAngle = baseHomeAngle;

// Initiliaze input for serial commands
char commandBuffer[32];
byte commandIndex = 0;

/* ---------- Functions ---------- */
// Make sure the joint angles are within the safe limits
#line 106 "/home/admin/git/shortpod_mustard_robot_arm/shortpod_mustard_robot_arm.ino"
int checkAngleBoundary(int value, int lowLimit, int highLimit);
#line 113 "/home/admin/git/shortpod_mustard_robot_arm/shortpod_mustard_robot_arm.ino"
void applyPose();
#line 131 "/home/admin/git/shortpod_mustard_robot_arm/shortpod_mustard_robot_arm.ino"
void showPosition();
#line 158 "/home/admin/git/shortpod_mustard_robot_arm/shortpod_mustard_robot_arm.ino"
void showHelp();
#line 185 "/home/admin/git/shortpod_mustard_robot_arm/shortpod_mustard_robot_arm.ino"
void goHome();
#line 199 "/home/admin/git/shortpod_mustard_robot_arm/shortpod_mustard_robot_arm.ino"
void moveToAbsolutePose(int base, int arm1, int elbow, int tiltWrist, int twistWrist, int claw);
#line 210 "/home/admin/git/shortpod_mustard_robot_arm/shortpod_mustard_robot_arm.ino"
void moveJoint(const char *jointName, int direction);
#line 233 "/home/admin/git/shortpod_mustard_robot_arm/shortpod_mustard_robot_arm.ino"
void turnMotor(const char *jointName, int angle, int delay_ms);
#line 255 "/home/admin/git/shortpod_mustard_robot_arm/shortpod_mustard_robot_arm.ino"
void doActionFlower();
#line 272 "/home/admin/git/shortpod_mustard_robot_arm/shortpod_mustard_robot_arm.ino"
void doActionWakeup();
#line 294 "/home/admin/git/shortpod_mustard_robot_arm/shortpod_mustard_robot_arm.ino"
void processCommand(char *cmd);
#line 436 "/home/admin/git/shortpod_mustard_robot_arm/shortpod_mustard_robot_arm.ino"
void setup();
#line 469 "/home/admin/git/shortpod_mustard_robot_arm/shortpod_mustard_robot_arm.ino"
void loop();
#line 106 "/home/admin/git/shortpod_mustard_robot_arm/shortpod_mustard_robot_arm.ino"
int checkAngleBoundary(int value, int lowLimit, int highLimit) {
  if (value < lowLimit) return lowLimit;
  if (value > highLimit) return highLimit;
  return value;
}

// Apply the current joint angles to the servos and print the current pose
void applyPose() {
  clawAngle = checkAngleBoundary(clawAngle, clawMinAngle, clawMaxAngle);
  tiltWristAngle = checkAngleBoundary(tiltWristAngle, tiltWristMinAngle, tiltWristMaxAngle);
  twistWristAngle = checkAngleBoundary(twistWristAngle, twistWristMinAngle, twistWristMaxAngle);
  elbowAngle = checkAngleBoundary(elbowAngle, elbowMinAngle, elbowMaxAngle);
  arm1Angle = checkAngleBoundary(arm1Angle, arm1MinAngle, arm1MaxAngle);
  baseAngle = checkAngleBoundary(baseAngle, baseMinAngle, baseMaxAngle);
  clawServo.write(clawAngle);
  tiltWristServo.write(tiltWristAngle);
  twistWristServo.write(twistWristAngle);
  elbowServo.write(elbowAngle);
  arm1Servo.write(arm1Angle);
  arm2Servo.write(180 - arm1Angle);  // Arm 2 mirrors Arm 1
  baseServo.write(baseAngle);
  showPosition();
}

// Display the current commanded angle for each joint
void showPosition() {
  Serial.println();
  Serial.println(F("===== CURRENT COMMANDED POSE ====="));

  Serial.print(F("Claw    = "));
  Serial.println(clawAngle);

  Serial.print(F("Tilt Wrist = "));
  Serial.println(tiltWristAngle);

  Serial.print(F("Twist Wrist = "));
  Serial.println(twistWristAngle);

  Serial.print(F("Elbow   = "));
  Serial.println(elbowAngle);

  Serial.print(F("Arm 1   = "));
  Serial.println(arm1Angle);

  Serial.print(F("Arm 2   = "));
  Serial.println(180 - arm1Angle);

  Serial.print(F("Base    = "));
  Serial.println(baseAngle);
}

// Display the available serial commands
void showHelp() {
  Serial.println();
  Serial.println(F("====== WEEDING ARM CALIBRATION ======"));
  Serial.println(F("Serial Monitor: 9600 baud + Newline"));
  Serial.println();
  Serial.println(F("General commands:"));
  Serial.println(F("  HELP       Show this list"));
  Serial.println(F("  SHOW       Print current pose"));
  Serial.println(F("  LIMITS     Print safe angle limits"));
  Serial.println(F("  HOME       Move to the HOME pose"));
  Serial.println(F("  WAKEUP     Move all motors around"));
  Serial.println(F("  MOVE x y z a b c   Absolute joint move (base, arm1, elbow, tilt, twist, claw)"));
  Serial.println();
  Serial.println(F("Move one joint by degrees (relative):"));
  Serial.println(F("  C+ / C-    Claw"));
  Serial.println(F("  W1+ / W1-  Tilt Wrist"));
  Serial.println(F("  W2+ / W2-  Twist Wrist"));
  Serial.println(F("  E+ / E-    Elbow"));
  Serial.println(F("  A+ / A-    Arm 1; Arm 2 mirrors automatically"));
  Serial.println(F("  B+ / B-    Base"));
  Serial.println();
  Serial.println(F("Example: type B+ and press Send."));
  Serial.println(F("======================================"));
  Serial.println();
}

// Move all joints to their saved HOME angles
void goHome() {
  clawAngle = clawHomeAngle;
  tiltWristAngle = tiltWristHomeAngle;
  twistWristAngle = twistWristHomeAngle;
  elbowAngle = elbowHomeAngle;
  arm1Angle = arm1HomeAngle;
  baseAngle = baseHomeAngle;

  applyPose();

  Serial.println(F("Moved to HOME pose."));
}

// Move to an absolute pose: base, arm1, elbow, tiltWrist, twistWrist, claw
void moveToAbsolutePose(int base, int arm1, int elbow, int tiltWrist, int twistWrist, int claw) {
  baseAngle = checkAngleBoundary(base, baseMinAngle, baseMaxAngle);
  arm1Angle = checkAngleBoundary(arm1, arm1MinAngle, arm1MaxAngle);
  elbowAngle = checkAngleBoundary(elbow, elbowMinAngle, elbowMaxAngle);
  tiltWristAngle = checkAngleBoundary(tiltWrist, tiltWristMinAngle, tiltWristMaxAngle);
  twistWristAngle = checkAngleBoundary(twistWrist, twistWristMinAngle, twistWristMaxAngle);
  clawAngle = checkAngleBoundary(claw, clawMinAngle, clawMaxAngle);
  applyPose();
}

// Move one joint by a relative number of degrees
void moveJoint(const char *jointName, int direction) {
  if (strcmp(jointName, "claw") == 0) {
    clawAngle += direction;
  } else if (strcmp(jointName, "tiltWrist") == 0) {
    tiltWristAngle += direction;
  } else if (strcmp(jointName, "twistWrist") == 0) {
    twistWristAngle += direction;
  } else if (strcmp(jointName, "elbow") == 0) {
    elbowAngle += direction;
  } else if (strcmp(jointName, "arm1") == 0) {
    arm1Angle += direction;
  } else if (strcmp(jointName, "base") == 0) {
    baseAngle += direction;
  } else {
    Serial.println(F("Unknown joint name."));
    return;
  }

  applyPose();
}

// Turn one joint to a specific angle, and wait
// Specify 0 for delay_ms to skip the wait
void turnMotor(const char *jointName, int angle, int delay_ms) {
  if (strcmp(jointName, "claw") == 0) {
    clawAngle = angle;
  } else if (strcmp(jointName, "tiltWrist") == 0) {
    tiltWristAngle = angle;
  } else if (strcmp(jointName, "twistWrist") == 0) {
    twistWristAngle = angle;
  } else if (strcmp(jointName, "elbow") == 0) {
    elbowAngle = angle;
  } else if (strcmp(jointName, "arm1") == 0) {
    arm1Angle = angle;
  } else if (strcmp(jointName, "base") == 0) {
    baseAngle = angle;
  } else {
    Serial.println(F("Unknown joint name."));
    return;
  }
  applyPose();
  delay(delay_ms);
}

// Move all joints to their saved "FLOWER" angles
void doActionFlower() {
    int sleep_time = 1000;
    turnMotor("base", 80, sleep_time);
    turnMotor("arm1", 140, sleep_time);
    turnMotor("elbow", 70, sleep_time);
    turnMotor("tiltWrist", 10, sleep_time);
    turnMotor("twistWrist", 90, sleep_time);
    turnMotor("claw", 110, sleep_time);
    turnMotor("twistWrist", 155, sleep_time);
    turnMotor("arm1", 60, sleep_time);
    turnMotor("base", 160, sleep_time);
    turnMotor("tiltWrist", 160, sleep_time);
    turnMotor("arm1", 120, sleep_time);
    turnMotor("claw", 150, sleep_time);
}

// Move all joints to their saved "WAKEUP" angles
void doActionWakeup() {
    int sleep_time = 1000;
    turnMotor("arm1", 70, sleep_time);
    turnMotor("arm1", 40, sleep_time);
    turnMotor("arm1", 60, sleep_time);
    turnMotor("base", 30, sleep_time);
    turnMotor("base", 140, sleep_time);
    turnMotor("base", 80, sleep_time);
    turnMotor("elbow", 100, sleep_time);
    turnMotor("elbow", 50, sleep_time);
    turnMotor("tiltWrist", 70, sleep_time);
    turnMotor("tiltWrist", 0, sleep_time);
    turnMotor("claw", 170, sleep_time);
    turnMotor("claw", 115, sleep_time);
    turnMotor("twistWrist", 175, 0);
    turnMotor("twistWrist", 5, 0);
    turnMotor("twistWrist", 175, 0);
    turnMotor("twistWrist", 5, 0);
    turnMotor("twistWrist", 90, 0);
}

// Interpret and execute a command received over Serial
void processCommand(char *cmd) {
  if (strcmp(cmd, "HELP") == 0) {
    showHelp();
    return;
  }
      Serial.print(F("COMMAND:"));
            Serial.println(cmd);


  if (strncmp(cmd, "MOVE", 4) == 0) {
      Serial.println(F("I AM MOVING!"));
    char *rest = cmd + 4;
    int values[6] = {0, 0, 0, 0, 0, 0};
    int count = 0;

    while (*rest == ' ') rest++;

    char *token = strtok(rest, " ");
    while (token != NULL && count < 6) {
      values[count++] = atoi(token);
      token = strtok(NULL, " ");
    }

    if (count != 6) {
      Serial.println(F("MOVE requires 6 values: base arm1 elbow tilt twist claw"));
      return;
    }

    moveToAbsolutePose(values[0], values[1], values[2], values[3], values[4], values[5]);
    Serial.print(F("Moved to absolute pose: "));
    for (int i = 0; i < 6; i++) {
      Serial.print(values[i]);
      if (i < 5) Serial.print(F(" "));
    }
    Serial.println();
    return;
  }

  if (strcmp(cmd, "SHOW") == 0) {
    showPosition();
    return;
  }

  if (strcmp(cmd, "TEST") == 0) {
    int sleep_time = 1000;
    turnMotor("base", 30, sleep_time);
    turnMotor("base", 140, sleep_time);
    turnMotor("base", 80, sleep_time);
    return;
  }

  if (strcmp(cmd, "WAKEUP") == 0) {
    doActionWakeup();
    return;
  }

  if (strcmp(cmd, "FLOWER") == 0) {
    doActionFlower();
    return;
  }

  if (strcmp(cmd, "HOME") == 0 || strcmp(cmd, "SLEEP") == 0) {
    goHome();
    return;
  }

  // C+ or C- : Claw
  if (strlen(cmd) == 2 && cmd[0] == 'C') {
    if (cmd[1] == '+') {
      moveJoint("claw", mid_movement);
      return;
    }
    if (cmd[1] == '-') {
      moveJoint("claw", mid_movement * -1);
      return;
    }
  }

  // TI+ or TI- : Tilt Wrist
  if (strlen(cmd) == 3 && cmd[0] == 'T' && cmd[1] == 'I') {
    if (cmd[2] == '+') {
      moveJoint("tiltWrist", mid_movement);
      return;
    }
    if (cmd[2] == '-') {
      moveJoint("tiltWrist", mid_movement * -1);
      return;
    }
  }

  // TW+ or TW- : Twist Wrist
  if (strlen(cmd) == 3 && cmd[0] == 'T' && cmd[1] == 'W') {
    if (cmd[2] == '+') {
      moveJoint("twistWrist", mid_movement);
      return;
    }
    if (cmd[2] == '-') {
      moveJoint("twistWrist", mid_movement * -1);
      return;
    }
  }

  // E+ or E- : Elbow
  if (strlen(cmd) == 2 && cmd[0] == 'E') {
    if (cmd[1] == '+') {
      moveJoint("elbow", 10);
      return;
    }
    if (cmd[1] == '-') {
      moveJoint("elbow", -10);
      return;
    }
  }

  // A+ or A- : Arm 1 + Arm 2 mirror
  if (strlen(cmd) == 2 && cmd[0] == 'A') {
    if (cmd[1] == '+') {
      moveJoint("arm1", big_movement);
      return;
    }
    if (cmd[1] == '-') {
      moveJoint("arm1", big_movement * -1);
      return;
    }
  }

  // B+ or B- : Base
  if (strlen(cmd) == 2 && cmd[0] == 'B') {
    if (cmd[1] == '+') {
      moveJoint("base", big_movement);
      return;
    }
    if (cmd[1] == '-') {
      moveJoint("base", big_movement * -1);
      return;
    }
  }

  Serial.println(F("Unknown command. Type HELP."));
}

// Start Serial communication and attach all servos
void setup() {
  Serial.begin(9600);
  clawServo.attach(CLAW_PIN);
  tiltWristServo.attach(TILT_WRIST_PIN);
  twistWristServo.attach(TWIST_WRIST_PIN);
  elbowServo.attach(ELBOW_PIN);
  arm1Servo.attach(ARM1_PIN);
  arm2Servo.attach(ARM2_PIN);
  baseServo.attach(BASE_PIN);
  Serial.println(F("INFO> Running Setup"));

  /*
    FIRST TEST SAFETY:

    Keep this commented during the first power-on:
      // applyPose();

    This prevents all connected servos from immediately moving
    to the temporary example pose.

    After you have calibrated a verified safe HOME pose:
    1. Replace the six joint angle variables with your HOME values.
    2. Replace goHome() values with the same HOME values.
    3. Remove the two slashes below to enXX`lable applyPose().
  */
  applyPose();

  Serial.println(F("Smart Weeding Arm Calibration Ready"));
  showHelp();
  Serial.println(F("INFO> Init Setup Completed"));
}

// Read complete commands from Serial and process them
void loop() {
  while (Serial.available() > 0) {
    // Read the incoming input:
    char received = Serial.read();

    // Carritage return is ignored, only newline is used to terminate commands
    if (received == '\r') {
      continue;
    }

    // Newline indicates start parsing command
    if (received == '\n') {
      commandBuffer[commandIndex] = '\0';
      processCommand(commandBuffer);
      commandIndex = 0;
    // If command buffer is not full, add character to buffer
    } else if (commandIndex < sizeof(commandBuffer) - 1) {
      commandBuffer[commandIndex++] = received;
    }
  }
}
