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

  C+ / C-     = Claw
  W1+ / W1-   = Wrist 1
  W2+ / W2-   = Wrist 2
  E+ / E-     = Elbow
  A+ / A-     = Arm 1; Arm 2 mirrors automatically
  B+ / B-     = Base
*/

#include <Servo.h>

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

// ---------- Calibration behavior ----------
const int STEP = 2;  // Change to 1 for finer, slower adjustment.

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
  30,  // Wrist 1
  0,  // Wrist 2
  30,  // Elbow
  60,  // Arm 1
  20   // Base
};

int maxAngle[POSE_COUNT] = {
  179,   // Claw
  150,  // Wrist 1
  359,  // Wrist 2
  150,  // Elbow
  120,  // Arm 1
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
  90,  // Wrist 1
  90,  // Wrist 2
  90,  // Elbow
  90,  // Arm 1
  90   // Base
};

char commandBuffer[32];
byte commandIndex = 0;

// ---------- Helper functions ----------

int clampAngle(int value, int lowLimit, int highLimit) {
  if (value < lowLimit) return lowLimit;
  if (value > highLimit) return highLimit;
  return value;
}

void applyPose() {
  for (byte i = 0; i < POSE_COUNT; i++) {
    pose[i] = clampAngle(pose[i], minAngle[i], maxAngle[i]);
  }

  clawServo.write(pose[0]);
  wrist1Servo.write(pose[1]);
  wrist2Servo.write(pose[2]);
  elbowServo.write(pose[3]);
  arm1Servo.write(pose[4]);
  arm2Servo.write(180 - pose[4]);  // Arm 2 mirrors Arm 1
  baseServo.write(pose[5]);
  showPose();
}

void showPose() {
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

  Serial.println();
  Serial.print(F("Copy this 6-number pose: "));
  Serial.print(pose[0]);
  Serial.print(',');
  Serial.print(pose[1]);
  Serial.print(',');
  Serial.print(pose[2]);
  Serial.print(',');
  Serial.print(pose[3]);
  Serial.print(',');
  Serial.print(pose[4]);
  Serial.print(',');
  Serial.println(pose[5]);

  Serial.println(F("Order: Claw,Wrist1,Wrist2,Elbow,Arm1,Base"));
  Serial.println(F("=================================="));
  Serial.println();
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
  Serial.println();
  Serial.println(F("Move one joint by STEP degrees:"));
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
  pose[1] = 80;   // Wrist 1
  pose[2] = 90;   // Wrist 2
  pose[3] = 90;   // Elbow
  pose[4] = 110;  // Arm 1; Arm 2 is 180 - Arm 1
  pose[5] = 50;   // Base

  applyPose();

  Serial.println(F("Moved to HOME pose."));
  //showPose();
}


void moveJoint(byte poseIndex, int direction) {
  pose[poseIndex] += direction * STEP;
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

  //showPose();
}

void testMove(int angle1, int angle2) {
  pose[0] = angle1;
  applyPose();
  delay(1000);
  pose[0] = angle2;
  applyPose();
}

void processCommand(char *cmd) {
  if (strcmp(cmd, "HELP") == 0) {
    showHelp();
    return;
  }

  if (strcmp(cmd, "SHOW") == 0) {
    showPose();
    return;
  }

  if (strcmp(cmd, "LIMITS") == 0) {
    showLimits();
    return;
  }

  if (strcmp(cmd, "TEST") == 0) {
    testMove(179, 105);
    return;
  }

  if (strcmp(cmd, "HOME") == 0) {
    goHome();
    return;
  }

  // C+ or C- : Claw
  if (strlen(cmd) == 2 && cmd[0] == 'C') {
    if (cmd[1] == '+') {
      moveJoint(0, 5);
      return;
    }
    if (cmd[1] == '-') {
      moveJoint(0, -5);
      return;
    }
  }
/*
  // W1+ or W1- : Wrist 1
  if (strlen(cmd) == 3 && cmd[0] == 'W' && cmd[1] == '1') {
    if (cmd[2] == '+') {
      moveJoint(1, 5);
      return;
    }
    if (cmd[2] == '-') {
      moveJoint(1, -5);
      return;
    }
  }

  // W2+ or W2- : Wrist 2
  if (strlen(cmd) == 3 && cmd[0] == 'W' && cmd[1] == '2') {
    if (cmd[2] == '+') {
      moveJoint(2, 5);
      return;
    }
    if (cmd[2] == '-') {
      moveJoint(2, -5);
      return;
    }
  }
*/
  // W1+, W1-, W2+, W2- : Wrist motors
  if (strlen(cmd) == 3 && cmd[0] == 'W' && (cmd[1] == '1' || cmd[1] == '2')) {

    byte wristIndex = (cmd[1] == '1') ? 1 : 2;

    if (cmd[2] == '+') {
      moveJoint(wristIndex, 5);
      return;
    }
    if (cmd[2] == '-') {
      moveJoint(wristIndex, -5);
      return;
    }
  }

  // E+ or E- : Elbow
  if (strlen(cmd) == 2 && cmd[0] == 'E') {
    if (cmd[1] == '+') {
      moveJoint(3, 1);
      return;
    }
    if (cmd[1] == '-') {
      moveJoint(3, -1);
      return;
    }
  }

  // A+ or A- : Arm 1 + Arm 2 mirror
  if (strlen(cmd) == 2 && cmd[0] == 'A') {
    if (cmd[1] == '+') {
      moveJoint(4, 1);
      return;
    }
    if (cmd[1] == '-') {
      moveJoint(4, -1);
      return;
    }
  }

  // B+ or B- : Base
  if (strlen(cmd) == 2 && cmd[0] == 'B') {
    if (cmd[1] == '+') {
      moveJoint(5, 1);
      return;
    }
    if (cmd[1] == '-') {
      moveJoint(5, -1);
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

  // applyPose();

  Serial.println(F("Smart Weeding Arm Calibration Ready"));
  showHelp();
  showPose();
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