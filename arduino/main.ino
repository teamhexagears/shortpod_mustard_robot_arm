/*
  Smart Weeding Robot Arm - Calibration Code

  Servo wiring:
  D3  = Claw
  D4  = Wrist 1
  D5  = Wrist 2
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


  C+ / C-     = Claw
  W1+ / W1-   = Wrist 1
  W2+ / W2-   = Wrist 2
  E+ / E-     = Elbow
  A+ / A-     = Arm 1; Arm 2 mirrors automatically
  B+ / B-     = Base
*/

/* Include Libraries */
#include <Servo.h>

/* Initialize Motors */
// ---------- Servo objects ----------
Servo clawServo;    // D3
Servo wrist1Servo;  // D4
Servo wrist2Servo;  // D5
Servo elbowServo;   // D6
Servo arm1Servo;    // D9
Servo arm2Servo;    // D10, mirrored with Arm 1
Servo baseServo;    // D11

// ---------- Pin mapping ----------
const byte CLAW_PIN = 3;
const byte WRIST1_PIN = 4;
const byte WRIST2_PIN = 5;
const byte ELBOW_PIN = 6;
const byte ARM1_PIN = 9;
const byte ARM2_PIN = 10;
const byte BASE_PIN = 11;

const int small_movement = 2;
const int mid_movement = 5;
const int big_movement = 10;

// ---------- Calibration behavior ----------
// Pose value order:
// [0] Claw
// [1] Wrist 1
// [2] Wrist 2
// [3] Elbow
// [4] Arm 1
// [5] Base
const byte POSE_COUNT = 6;

/*
  Temporary safety limits.

  IMPORTANT:
  These are only starting limits to reduce the risk of crashes.
  You MUST replace them with safe values measured on your own arm.
*/
int minAngle[POSE_COUNT] = {
  100,  // Claw
  0,  // Wrist 1
  0,  // Wrist 2
  10,  // Elbow
  30,  // Arm 1
  0   // Base
};

int maxAngle[POSE_COUNT] = {
  179,   // Claw
  90,  // Wrist 1
  179,  // Wrist 2
  130,  // Elbow
  150,  // Arm 1
  160   // Base
};

/*
  Initial pose.

  WARNING:
  This is a placeholder. It may not be safe for your physical arm.
  During first testing, leave applyPose() commented out in setup().
*/
int pose[POSE_COUNT] = {
  150,  // Claw
  30,  // Wrist 1
  90,  // Wrist 2
  90,  // Elbow
  40,  // Arm 1
  80   // Base
};

char commandBuffer[32];
byte commandIndex = 0;

// ---------- Helper functions ----------

int checkAngleBoundary(int value, int lowLimit, int highLimit) {
  if (value < lowLimit) return lowLimit;
  if (value > highLimit) return highLimit;
  return value;
}

void applyPose() {
  for (byte i = 0; i < POSE_COUNT; i++) {
    pose[i] = checkAngleBoundary(pose[i], minAngle[i], maxAngle[i]);
  }
  clawServo.write(pose[0]);
  wrist1Servo.write(pose[1]);
  wrist2Servo.write(pose[2]);
  elbowServo.write(pose[3]);
  arm1Servo.write(pose[4]);
  arm2Servo.write(180 - pose[4]);  // Arm 2 mirrors Arm 1
  baseServo.write(pose[5]);
  showPosition();
}

void showPosition() {
  Serial.println();
  Serial.println(F("===== CURRENT COMMANDED POSE ====="));

  Serial.print(F("Claw    = "));
  Serial.println(pose[0]);

  Serial.print(F("Wrist 1 = "));
  Serial.println(pose[1]);

  Serial.print(F("Wrist 2 = "));
  Serial.println(pose[2]);

  Serial.print(F("Elbow   = "));
  Serial.println(pose[3]);

  Serial.print(F("Arm 1   = "));
  Serial.println(pose[4]);

  Serial.print(F("Arm 2   = "));
  Serial.println(180 - pose[4]);

  Serial.print(F("Base    = "));
  Serial.println(pose[5]);
}

void showLimits() {
  Serial.println();
  Serial.println(F("===== CURRENT SAFETY LIMITS ====="));

  Serial.print(F("Claw:    "));
  Serial.print(minAngle[0]);
  Serial.print(F(" to "));
  Serial.println(maxAngle[0]);

  Serial.print(F("Wrist 1: "));
  Serial.print(minAngle[1]);
  Serial.print(F(" to "));
  Serial.println(maxAngle[1]);

  Serial.print(F("Wrist 2: "));
  Serial.print(minAngle[2]);
  Serial.print(F(" to "));
  Serial.println(maxAngle[2]);

  Serial.print(F("Elbow:   "));
  Serial.print(minAngle[3]);
  Serial.print(F(" to "));
  Serial.println(maxAngle[3]);

  Serial.print(F("Arm 1:   "));
  Serial.print(minAngle[4]);
  Serial.print(F(" to "));
  Serial.println(maxAngle[4]);

  Serial.print(F("Arm 2:   "));
  Serial.print(180 - maxAngle[4]);
  Serial.print(F(" to "));
  Serial.println(180 - minAngle[4]);

  Serial.print(F("Base:    "));
  Serial.print(minAngle[5]);
  Serial.print(F(" to "));
  Serial.println(maxAngle[5]);

  Serial.println(F("================================="));
  Serial.println();
}

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
  Serial.println();
  Serial.println(F("Move one joint by degrees (relative):"));
  Serial.println(F("  C+ / C-    Claw"));
  Serial.println(F("  W1+ / W1-  Wrist 1"));
  Serial.println(F("  W2+ / W2-  Wrist 2"));
  Serial.println(F("  E+ / E-    Elbow"));
  Serial.println(F("  A+ / A-    Arm 1; Arm 2 mirrors automatically"));
  Serial.println(F("  B+ / B-    Base"));
  Serial.println();
  Serial.println(F("Example: type B+ and press Send."));
  Serial.println(F("======================================"));
  Serial.println();
}

void goHome() {
  // Replace these six numbers after you have calibrated your real HOME pose.
  pose[0] = 150;   // Claw
  pose[1] = 50;   // Wrist 1
  pose[2] = 90;   // Wrist 2
  pose[3] = 90;   // Elbow
  pose[4] = 60;  // Arm 1; Arm 2 is 180 - Arm 1
  pose[5] = 80;   // Base

  applyPose();

  Serial.println(F("Moved to HOME pose."));
}

void moveJoint(byte poseIndex, int direction) {
  pose[poseIndex] += direction;
  applyPose();

  Serial.print(F("Moved "));
  switch (poseIndex) {
    case 0: Serial.println(F("Claw")); break;
    case 1: Serial.println(F("Wrist 1")); break;
    case 2: Serial.println(F("Wrist 2")); break;
    case 3: Serial.println(F("Elbow")); break;
    case 4: Serial.println(F("Arm 1 and mirrored Arm 2")); break;
    case 5: Serial.println(F("Base")); break;
  }
}

void turn_motor(int motor_index, int angle, int delay_ms) {
  pose[motor_index] = angle;
  applyPose();
  delay(delay_ms);
}

void processCommand(char *cmd) {
  if (strcmp(cmd, "HELP") == 0) {
    showHelp();
    return;
  }

  if (strcmp(cmd, "SHOW") == 0) {
    showPosition();
    return;
  }

  if (strcmp(cmd, "LIMITS") == 0) {
    showLimits();
    return;
  }

  if (strcmp(cmd, "TEST") == 0) {
    int sleep_time = 1000;
    turn_motor(5, 30, sleep_time);
    turn_motor(5, 140, sleep_time);
    turn_motor(5, 80, sleep_time);
    return;
  }

  if (strcmp(cmd, "WAKEUP") == 0) {
    // Pose value order:
    // [0] Claw
    // [1] Wrist 1
    // [2] Wrist 2
    // [3] Elbow
    // [4] Arm 1
    // [5] Base

    int sleep_time = 1000;
    turn_motor(4, 70, sleep_time);
    turn_motor(4, 30, sleep_time);
    turn_motor(4, 60, sleep_time);
    turn_motor(5, 30, sleep_time);
    turn_motor(5, 140, sleep_time);
    turn_motor(5, 80, sleep_time);
    turn_motor(3, 100, sleep_time);
    turn_motor(3, 50, sleep_time);
    turn_motor(1, 70, sleep_time);
    turn_motor(1, 0, sleep_time);
    turn_motor(0, 170, sleep_time);
    turn_motor(0, 110, sleep_time);
    turn_motor(2, 175, 0);
    turn_motor(2, 5, 0);
    turn_motor(2, 175, 0);
    turn_motor(2, 5, 0);
    turn_motor(2, 90, 0);
    return;
  }

  if (strcmp(cmd, "FLOWER") == 0) {
    /*
10:35:47.750 -> ===== CURRENT COMMANDED POSE =====
10:35:47.783 -> Claw    = 150
10:35:47.783 -> Wrist 1 = 10
10:35:47.815 -> Wrist 2 = 90
10:35:47.815 -> Elbow   = 70
10:35:47.852 -> Arm 1   = 140
10:35:47.852 -> Arm 2   = 40
10:35:47.888 -> Base    = 80
    */
    // Pose value order:
    // [0] Claw
    // [1] Wrist 1
    // [2] Wrist 2
    // [3] Elbow
    // [4] Arm 1
    // [5] Base

    int sleep_time = 1000;
    turn_motor(5, 80, sleep_time);
    turn_motor(4, 140, sleep_time);
    turn_motor(3, 70, sleep_time);
    turn_motor(1, 10, sleep_time);
    turn_motor(2, 90, sleep_time);
    turn_motor(0, 110, sleep_time);
    turn_motor(2, 155, sleep_time);
    turn_motor(4, 60, sleep_time);
    turn_motor(5, 160, sleep_time);
    turn_motor(1, 160, sleep_time);
    turn_motor(4, 120, sleep_time);
    turn_motor(0, 150, sleep_time);
    
  }

  if (strcmp(cmd, "HOME") == 0 || strcmp(cmd, "SLEEP") == 0) {
    goHome();
    return;
  }

  // C+ or C- : Claw
  if (strlen(cmd) == 2 && cmd[0] == 'C') {
    if (cmd[1] == '+') {
      moveJoint(0, mid_movement);
      return;
    }
    if (cmd[1] == '-') {
      moveJoint(0, mid_movement * -1);
      return;
    }
  }

  // W1+ or W1- : Wrist 1
  if (strlen(cmd) == 3 && cmd[0] == 'W' && cmd[1] == '1') {
    if (cmd[2] == '+') {
      moveJoint(1, mid_movement);
      return;
    }
    if (cmd[2] == '-') {
      moveJoint(1, mid_movement * -1);
      return;
    }
  }

  // W2+ or W2- : Wrist 2
  if (strlen(cmd) == 3 && cmd[0] == 'W' && cmd[1] == '2') {
    if (cmd[2] == '+') {
      moveJoint(2, mid_movement);
      return;
    }
    if (cmd[2] == '-') {
      moveJoint(2, mid_movement * -1);
      return;
    }
  }

  // E+ or E- : Elbow
  if (strlen(cmd) == 2 && cmd[0] == 'E') {
    if (cmd[1] == '+') {
      moveJoint(3, 10);
      return;
    }
    if (cmd[1] == '-') {
      moveJoint(3, -10);
      return;
    }
  }

  // A+ or A- : Arm 1 + Arm 2 mirror
  if (strlen(cmd) == 2 && cmd[0] == 'A') {
    if (cmd[1] == '+') {
      moveJoint(4, big_movement);
      return;
    }
    if (cmd[1] == '-') {
      moveJoint(4, big_movement * -1);
      return;
    }
  }

  // B+ or B- : Base
  if (strlen(cmd) == 2 && cmd[0] == 'B') {
    if (cmd[1] == '+') {
      moveJoint(5, big_movement);
      return;
    }
    if (cmd[1] == '-') {
      moveJoint(5, big_movement * -1);
      return;
    }
  }

  Serial.println(F("Unknown command. Type HELP."));
}

void setup() {
  Serial.begin(9600);
  clawServo.attach(CLAW_PIN);
  wrist1Servo.attach(WRIST1_PIN);
  wrist2Servo.attach(WRIST2_PIN);
  elbowServo.attach(ELBOW_PIN);
  arm1Servo.attach(ARM1_PIN);
  arm2Servo.attach(ARM2_PIN);
  baseServo.attach(BASE_PIN);

  /*
    FIRST TEST SAFETY:

    Keep this commented during the first power-on:
      // applyPose();

    This prevents all connected servos from immediately moving
    to the temporary example pose.

    After you have calibrated a verified safe HOME pose:
    1. Replace pose[] values with your HOME values.
    2. Replace goHome() values with the same HOME values.
    3. Remove the two slashes below to enXX`lable applyPose().
  */

  //applyPose();
  goHome();
  
  Serial.println(F("Smart Weeding Arm Calibration Ready"));
  showHelp();
  showPosition();
  Serial.println(F("INFO> Init Setup Completed"));

}

void loop() {
  while (Serial.available() > 0) {
    char received = Serial.read();

    if (received == '\r') {
      continue;
    }

    if (received == '\n') {
      commandBuffer[commandIndex] = '\0';
      processCommand(commandBuffer);
      commandIndex = 0;
    } else if (commandIndex < sizeof(commandBuffer) - 1) {
      commandBuffer[commandIndex++] = received;
    } else {
      commandIndex = 0;
      Serial.println(F("Command too long. Cleared. Type HELP."));
    }
  }
}