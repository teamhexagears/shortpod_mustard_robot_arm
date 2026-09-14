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


  C+ / C-     = Claw
  W1+ / W1-   = Tilt Wrist
  W2+ / W2-   = Twist Wrist
  E+ / E-     = Elbow
  A+ / A-     = Arm 1; Arm 2 mirrors automatically
  B+ / B-     = Base
*/

/* Include Libraries */
#include <Servo.h>
#include <iostream>
#include <string>

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

// Move one joint by a relative number of degrees
void moveJoint(std::string jointName, int direction) {
  if (jointName == "claw") {
    clawAngle += direction;
  } else if (jointName == "tiltWrist") {
    tiltWristAngle += direction;
  } else if (jointName == "twistWrist") {
    twistWristAngle += direction;
  } else if (jointName == "elbow") {
    elbowAngle += direction;
  } else if (jointName == "arm1") {
    arm1Angle += direction;
  } else if (jointName == "base") {
    baseAngle += direction;
  } else {
    Serial.println(F("Unknown joint name."));
    return;
  }

  applyPose();
}

// Turn one joint to a specific angle, and wait
// Specify 0 for delay_ms to skip the wait
void turn_motor(int motor_index, int angle, int delay_ms) {
  switch (motor_index) {
    case 0: clawAngle = angle; break;
    case 1: tiltWristAngle = angle; break;
    case 2: twistWristAngle = angle; break;
    case 3: elbowAngle = angle; break;
    case 4: arm1Angle = angle; break;
    case 5: baseAngle = angle; break;
  }
  applyPose();
  delay(delay_ms);
}

// Move all joints to their saved "FLOWER" angles
void doActionFlower() {
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

// Move all joints to their saved "WAKEUP" angles
void doActionWakeup() {
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
}

// Interpret and execute a command received over Serial
void processCommand(char *cmd) {
  if (strcmp(cmd, "HELP") == 0) {
    showHelp();
    return;
  }

  if (strcmp(cmd, "SHOW") == 0) {
    showPosition();
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

  //applyPose();
  goHome();
  
  Serial.println(F("Smart Weeding Arm Calibration Ready"));
  showHelp();
  showPosition();
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