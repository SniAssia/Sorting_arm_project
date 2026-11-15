/*
 * SEVEN — Arduino-only 3-DOF color-route demo + gripper
 * Base D6, shoulder D9, elbow D10, gripper D11.
 * Upload this sketch INSTEAD of the original main.ino.
 *
 * Model from your report: L1 = L2 = 12 cm, origin at shoulder pivot.
 *   h = L1*cos(s) + L2*cos(s+e-PI)
 *   x = h*sin(b); y = h*cos(b)
 *   z = L1*sin(s) + L2*sin(s+e-PI)
 *
 * The sketch has no joint sensors, collision detection, camera, or grip feedback.
 * "Done" means command interpolation ended, NOT measured arrival.
 * Only servo command limits are 0..180: these are NOT mechanical safe limits.
 * First verify startup alignment / clearance before sending ARM.
 * Default gripper calibration: OPEN=90, CLOSED=115 (adjust constants below).
 * The gripper starts OPEN at 90 degrees when ARM is sent: check clearance first.
 * After pickup: first lift to AP, then return to HOME before moving to the bin.
 */
#include <Arduino.h>
#include <Servo.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

const byte PIN_BASE = 6, PIN_SHOULDER = 9, PIN_ELBOW = 10, PIN_GRIPPER = 11;
const float L1 = 12.0f, L2 = 12.0f;   // cm; check actual pivot distances
const unsigned long STEP_MS = 100;     // ~10 degrees/s, conservative first test
const float STEP_DEG = 1.0f;
const unsigned long GRIPPER_STEP_MS = 30; // 1 degree / 30 ms
const int GRIPPER_OPEN_ANGLE = 90;         // Default OPEN position
const int GRIPPER_CLOSED_ANGLE = 115;      // Default CLOSED position
const int GRIPPER_START_ANGLE = GRIPPER_OPEN_ANGLE; // Start open (check clearance!)


bool autoAdvance = false;
unsigned long waypointFinishedAt = 0;
const unsigned long WAYPOINT_PAUSE_MS = 1000;

// Original Seven physical orientation: base normal, shoulder normal,
// elbow reversed. If you calibrated using direct Servo.write() angles,
// check this mapping BEFORE attaching the arm. Do not change blindly.
const bool REVERSED[3] = {false, false, false};
const float TRIM_DEG[3] = {0.0f, 0.0f, 0.0f};

// Only SOFTWARE angle limits requested: 0..180 degrees for every joint.
// These are NOT mechanical safe limits for the assembled robot.
const float LOWER[3] = {0.0f, 0.0f, 0.0f};
const float UPPER[3] = {180.0f, 180.0f, 180.0f};

struct Joints { float b, s, e; };
struct Point { float x, y, z; };
struct NamedPose { const char *name; Point xyz; bool defined; };

// Set coordinates in cm using "SET NAME x y z" after measuring the work area.
// Fixed measured positions, centimeters; HOME = FK(105,73,60).
// All paths still require physical clearance checks.
enum PoseId { HOME, AP, PICK, AR, DR, AG, DG, AB, DB, POSE_COUNT };
NamedPose poses[POSE_COUNT] = {
  {"HOME", {11.29f, -3.03f,  2.70f}, true},
  {"AP",   {3.93f, 14.68f, 14.18f}, true},
  {"PICK", {3.93f, 14.68f, -4.00f}, true},
  {"AR",   {13.14f, 6.13f, 8.00f}, true},
  {"DR",   {13.14f, 6.13f, 2.00f}, true},
  {"AG",   {13.14f, -6.13f, 8.00f}, true},
  {"DG",   {13.14f, -6.13f, 2.00f}, true},
  {"AB",   {3.7528f, -14.0055f, 8.00f}, true},
  {"DB",   {3.7528f, -14.0055f, 2.00f}, true}
};

Servo motors[3];
Servo gripper;
int gripCommanded = GRIPPER_START_ANGLE;
int gripTarget = GRIPPER_START_ANGLE;
int gripOpenAngle = GRIPPER_OPEN_ANGLE;       // Ready on every reset
int gripClosedAngle = GRIPPER_CLOSED_ANGLE;   // Ready on every reset
bool gripperMoving = false;
bool routeOpening = false;          // Route startup: open before approaching PICK
bool routeGripActionPending = false; // At PICK / drop, wait for gripper
unsigned long lastGripStep = 0;
const byte PINS[3] = {PIN_BASE, PIN_SHOULDER, PIN_ELBOW};
Joints commanded = {105, 73, 60};
Joints target = {105, 73, 60};
bool armed = false, moving = false;
bool sequenceActive = false, waitingNext = false;
byte sequenceColor = 0, sequenceStep = 0;
unsigned long lastStep = 0;
char lineBuf[80];
byte lineLength = 0;

float rad(float degrees) { return degrees * PI / 180.0f; }
float deg(float radians) { return radians * 180.0f / PI; }
float clampF(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

bool anglesFinite(Joints q) {
  return !isnan(q.b) && !isinf(q.b) &&
         !isnan(q.s) && !isinf(q.s) &&
         !isnan(q.e) && !isinf(q.e);
}

// Used by IK: a target that cannot be reached with 0..180 model angles
// must NOT be silently misrepresented as the requested XYZ position.
bool anglesValid(Joints q) {
  if (!anglesFinite(q)) return false;
  const float a[3] = {q.b, q.s, q.e};
  for (byte i = 0; i < 3; ++i)
    if (a[i] < LOWER[i] || a[i] > UPPER[i]) return false;
  return true;
}

Joints clampJoints(Joints q) {
  q.b = clampF(q.b, LOWER[0], UPPER[0]);
  q.s = clampF(q.s, LOWER[1], UPPER[1]);
  q.e = clampF(q.e, LOWER[2], UPPER[2]);
  return q;
}

// The ONLY conversion from model joint angles to physical servo commands.
// A physical output is always clamped to 0..180, even after reversal/trim.
int servoCommand(float modelAngle, byte i) {
  float a = clampF(modelAngle, 0.0f, 180.0f);
  float pwm = (REVERSED[i] ? 180.0f - a : a) + TRIM_DEG[i];
  return (int)(clampF(pwm, 0.0f, 180.0f) + 0.5f);
}

Point forwardKinematics(Joints q) {
  float b = rad(q.b), s = rad(q.s), e = rad(q.e);
  float h = L1 * cosf(s) + L2 * cosf(s + e - (float)PI);
  float z = L1 * sinf(s) + L2 * sinf(s + e - (float)PI);
  return {h * sinf(b), h * cosf(b), z};
}

bool inverseKinematics(Point p, Joints &result) {
  // Conservative workspace bounds from Seven; not a collision guarantee.
  if (isnan(p.x) || isnan(p.y) || isnan(p.z) ||
      isinf(p.x) || isinf(p.y) || isinf(p.z) || p.x < 1.0f || p.z < -6.0f)
    return false;
  float horizontal = sqrt(p.x*p.x + p.y*p.y);
  float distance = sqrt(horizontal*horizontal + p.z*p.z);
  if (distance < 3.0f || distance > 23.0f) return false;
  float cosE = (L1*L1 + L2*L2 - distance*distance) / (2.0f*L1*L2);
  float cosA = (L1*L1 + distance*distance - L2*L2) / (2.0f*L1*distance);
  result.b = deg(atan2(p.x, p.y));
  result.s = deg(atan2(p.z, horizontal) + acos(clampF(cosA, -1.0f, 1.0f)));
  result.e = deg(acos(clampF(cosE, -1.0f, 1.0f)));
  if (!anglesValid(result)) return false;
  Point check = forwardKinematics(result);
  float dx = check.x - p.x, dy = check.y - p.y, dz = check.z - p.z;
  float error = sqrt(dx*dx + dy*dy + dz*dz);
  return error < 0.1f;
}

void writeJoints(Joints q) {
  float a[3] = {q.b, q.s, q.e};
  for (byte i = 0; i < 3; ++i) {
    motors[i].write(servoCommand(a[i], i));
  }
}

void printJoints(Joints q) {
  Serial.print(F(" b=")); Serial.print(q.b,1);
  Serial.print(F(" s=")); Serial.print(q.s,1);
  Serial.print(F(" e=")); Serial.print(q.e,1);
}
void printPoint(Point p) {
  Serial.print(F(" x=")); Serial.print(p.x,2);
  Serial.print(F(" y=")); Serial.print(p.y,2);
  Serial.print(F(" z=")); Serial.print(p.z,2);
}

bool startMove(Joints q) {
  if (!armed) { Serial.println(F("ERR: ARM first")); return false; }
  if (!anglesFinite(q)) { Serial.println(F("ERR: non-finite angle")); return false; }

  // Manual M commands outside 0..180 are CLAMPED, not rejected.
  Joints bounded = clampJoints(q);
  if (bounded.b != q.b || bounded.s != q.s || bounded.e != q.e)
    Serial.println(F("NOTE: Angle(s) clamped to 0..180"));

  target = bounded;
  moving = true;
  lastStep = millis();
  Serial.print(F("MOVING toward model angles:")); printJoints(target);
  Serial.println();
  return true;
}

// Command servo slowly; commanded position is only an estimate, not feedback.
bool startGripMove(int angle) {
  if (!armed) {
    Serial.println(F("ERR: ARM first"));
    return false;
  }
  if (angle < 0 || angle > 180) {
    Serial.println(F("ERR: gripper angle must be 0..180"));
    return false;
  }
  gripTarget = angle;
  gripperMoving = (gripTarget != gripCommanded);
  lastGripStep = millis();
  Serial.print(F("GRIPPER target "));
  Serial.println(gripTarget);
  if (!gripperMoving) Serial.println(F("Gripper already at commanded target."));
  return true;
}

void finishWaypoint() {
  if (sequenceStep == 7) {
    sequenceActive = false;
    autoAdvance = false;
    waitingNext = false;
    Serial.println(F("Color route complete (gripper close/release commanded)."));
  } else {
    waitingNext = true;
    waypointFinishedAt = millis();
    if (autoAdvance) {
      Serial.println(F("Waypoint done; auto-advancing after pause."));
    } else {
      Serial.println(F("Verify clearance, then send NEXT."));
    }
  }
}

void startWaypoint();

void updateGripper() {
  if (!gripperMoving || millis() - lastGripStep < GRIPPER_STEP_MS) return;
  lastGripStep = millis();
  gripCommanded += (gripTarget > gripCommanded) ? 1 : -1;
  gripper.write(gripCommanded);
  if (gripCommanded == gripTarget) {
    gripperMoving = false;
    Serial.println(F("Gripper command interpolation complete."));
    if (sequenceActive && routeOpening) {
      routeOpening = false;
      startWaypoint();
    } else if (sequenceActive && routeGripActionPending) {
      routeGripActionPending = false;
      finishWaypoint();
    }
  }
}

byte waypointIndex(byte color, byte step) {
  // AP -> PICK -> AP (lift) -> HOME (reset joints) -> ABOVE_BIN
  // -> DROP_BIN -> ABOVE_BIN -> HOME
  if (step == 0 || step == 2) return AP;
  if (step == 1) return PICK;
  if (step == 3 || step == 7) return HOME;
  byte above = AR + 2*color; // color: 0=red, 1=green, 2=blue
  return (step == 5) ? above + 1 : above;
}

bool checkRoute(byte color) {
  for (byte step = 0; step < 8; ++step) {
    byte id = waypointIndex(color, step);
    Joints q;
    if (!poses[id].defined || !inverseKinematics(poses[id].xyz, q)) {
      Serial.print(F("ERR: undefined/unreachable pose "));
      Serial.println(poses[id].name);
      return false;
    }
  }
  Serial.println(F("Route model check OK (NOT a collision check)."));
  return true;
}

void startWaypoint() {
  byte id = waypointIndex(sequenceColor, sequenceStep);
  Joints q;
  if (!inverseKinematics(poses[id].xyz, q)) {
    Serial.println(F("ERR: inverse kinematics"));
    sequenceActive = false;
    return;
  }
  Serial.print(F("STEP ")); Serial.print(sequenceStep + 1);
  Serial.print(F("/8  ")); Serial.println(poses[id].name);
  waitingNext = false;
  if (!startMove(q)) sequenceActive = false;
}

float approach(float a, float b) {
  if (fabs(b-a) <= STEP_DEG) return b;
  return a + ((b > a) ? STEP_DEG : -STEP_DEG);
}
void updateMotion() {
  if (!moving || millis() - lastStep < STEP_MS) return;
  lastStep = millis();
  commanded.b = approach(commanded.b, target.b);
  commanded.s = approach(commanded.s, target.s);
  commanded.e = approach(commanded.e, target.e);
  writeJoints(commanded);
  if (fabs(commanded.b-target.b) < 0.01f &&
      fabs(commanded.s-target.s) < 0.01f &&
      fabs(commanded.e-target.e) < 0.01f) {
    moving = false;
    Serial.println(F("Arm command interpolation complete; verify physical position."));
    if (!sequenceActive) return;

    // Pickup: close after reaching PICK. Drop: open after reaching the bin.
    if (sequenceStep == 1 || sequenceStep == 5) {
      int desired = (sequenceStep == 1) ? gripClosedAngle : gripOpenAngle;
      if (sequenceStep == 1)
        Serial.println(F("At PICK: closing gripper."));
      else
        Serial.println(F("At DROP: opening gripper."));

      if (gripCommanded != desired) {
        routeGripActionPending = true;
        startGripMove(desired);
      } else {
        finishWaypoint();
      }
    } else {
      finishWaypoint();
    }
  }
}

bool readFloat(float &out) {
  char *token = strtok(NULL, " ,\t");
  if (!token) return false;
  char *end = NULL;
  out = strtod(token, &end);
  return end != token && *end == '\0' && !isnan(out) && !isinf(out);
}
bool readThree(float &x, float &y, float &z) {
  return readFloat(x) && readFloat(y) && readFloat(z)
         && strtok(NULL, " ,\t") == NULL;
}
int poseId(const char *s) {
  if (!s) return -1;
  for (byte i = 0; i < POSE_COUNT; ++i)
    if (strcmp(s, poses[i].name) == 0) return i;
  return -1;
}
int colorId(const char *s) {
  if (!s) return -1;
  if (strcmp(s, "RED") == 0) return 0;
  if (strcmp(s, "GREEN") == 0) return 1;
  if (strcmp(s, "BLUE") == 0) return 2;
  return -1;
}

void showHelp() {
  Serial.println(F("COMMANDS (Serial Monitor: 115200, Newline):"));
  Serial.println(F(" ARM             attach 4 servos; arm 105/73/60, grip 90"));
  Serial.println(F(" GRIP 100        manually move gripper to a chosen angle"));
  Serial.println(F(" SAVE OPEN       override OPEN angle (default 90, RAM only)"));
  Serial.println(F(" SAVE CLOSED     override CLOSED angle (default 115, RAM only)"));
  Serial.println(F(" OPEN / CLOSE    move gripper to configured positions"));
  Serial.println(F(" M b s e         move model angles (clamped 0..180)"));
  Serial.println(F(" XYZ x y z       solve IK and move (centimeters)"));
  Serial.println(F(" FK              print predicted commanded XYZ"));
  Serial.println(F(" SET AP x y z    teach a pose (also PICK AR DR AG DG AB DB HOME)"));
  Serial.println(F(" POSES           print stored target coordinates"));
  Serial.println(F(" CHECK RED       validate model route (RED/GREEN/BLUE)"));
  Serial.println(F(" RUN RED         8-step sort: lift AP, return HOME before bin"));
  Serial.println(F(" MANUAL RED      route, requires NEXT after each waypoint"));
  Serial.println(F(" NEXT            advance manual route"));
  Serial.println(F(" HOME            move to taught HOME pose"));
  Serial.println(F(" STOP            cancel motions; servos HOLD their commands"));
  Serial.println(F(" STATUS          show arm and gripper internal state"));
  Serial.println(F("Defaults OPEN=90 CLOSED=115; SAVE overrides only until reboot."));
  Serial.println(F("No camera, collision detection, or position feedback."));
}

void processLine(char *line) {
  for (char *p = line; *p; ++p) *p = toupper((unsigned char)*p);
  char *cmd = strtok(line, " ,\t");
  if (!cmd) return;
  if (strcmp(cmd, "HELP") == 0) { showHelp(); return; }
  if (strcmp(cmd, "STOP") == 0) {
    moving = false; sequenceActive = false; waitingNext = false;
    gripperMoving = false; routeOpening = false;
    routeGripActionPending = false; autoAdvance = false;
    Serial.println(F("STOP: updates cancelled; servos HOLD last commanded angles."));
    return;
  }
  if (strcmp(cmd, "STATUS") == 0) {
    Serial.print(F("armed=")); Serial.print(armed);
    Serial.print(F(" moving=")); Serial.print(moving);
    Serial.print(F(" route=")); Serial.print(sequenceActive);
    printJoints(commanded);
    Serial.print(F(" gripper=")); Serial.print(gripCommanded);
    Serial.print(F(" gripMoving=")); Serial.print(gripperMoving);
    Serial.print(F(" open=")); Serial.print(gripOpenAngle);
    Serial.print(F(" closed=")); Serial.println(gripClosedAngle);
    return;
  }
  if (strcmp(cmd, "ARM") == 0) {

  if (armed) {
    Serial.println(F("Already armed"));
    return;
  }

  // Initial position of the robot
  commanded = {105.0f, 73.0f, 60.0f};
  target = commanded;

  float angles[3] = {
    commanded.b,
    commanded.s,
    commanded.e
  };

  for (byte i = 0; i < 3; i++) {

    int servoAngle = servoCommand(angles[i], i);

    // Set the desired pulse before attaching
    motors[i].write(servoAngle);
    motors[i].attach(PINS[i]);

    // Explicitly send it again after attaching
    motors[i].write(servoAngle);

    Serial.print(F("Servo "));
    Serial.print(i);
    Serial.print(F(" commanded to "));
    Serial.println(servoAngle);
  }

  // Start gripper at an explicitly known command (verify 90 degrees is safe).
  gripper.write(gripCommanded);
  gripper.attach(PIN_GRIPPER);
  gripper.write(gripCommanded);
  armed = true;

  Serial.println(F("ARMED: joints 105/73/60; gripper 90"));
  return;
}
  if (strcmp(cmd, "GRIP") == 0) {
    if (moving || sequenceActive || gripperMoving) {
      Serial.println(F("ERR: STOP current operation first")); return;
    }
    char *arg = strtok(NULL, " ,\t");
    if (!arg || strtok(NULL, " ,\t") != NULL) {
      Serial.println(F("ERR: usage GRIP angle")); return;
    }
    char *endptr;
    long angle = strtol(arg, &endptr, 10);
    if (endptr == arg || *endptr != '\0' || angle < 0 || angle > 180) {
      Serial.println(F("ERR: angle must be integer 0..180")); return;
    }
    startGripMove((int)angle);
    return;
  }
  if (strcmp(cmd, "SAVE") == 0) {
    if (!armed || moving || sequenceActive || gripperMoving) {
      Serial.println(F("ERR: ARM first and finish motions")); return;
    }
    char *which = strtok(NULL, " ,\t");
    if (!which || strtok(NULL, " ,\t") != NULL) {
      Serial.println(F("ERR: SAVE OPEN or SAVE CLOSED")); return;
    }
    if (strcmp(which, "OPEN") == 0) {
      gripOpenAngle = gripCommanded;
      Serial.print(F("OPEN angle saved: ")); Serial.println(gripOpenAngle);
    } else if (strcmp(which, "CLOSED") == 0) {
      gripClosedAngle = gripCommanded;
      Serial.print(F("CLOSED angle saved: ")); Serial.println(gripClosedAngle);
    } else Serial.println(F("ERR: SAVE OPEN or SAVE CLOSED"));
    return;
  }
  if (strcmp(cmd, "OPEN") == 0 || strcmp(cmd, "CLOSE") == 0) {
    if (moving || sequenceActive || gripperMoving) {
      Serial.println(F("ERR: STOP current operation first")); return;
    }
    bool opening = strcmp(cmd, "OPEN") == 0;
    int angle = opening ? gripOpenAngle : gripClosedAngle;
    if (angle < 0) {
      Serial.println(F("ERR: OPEN/CLOSED angle not configured"));
      return;
    }
    startGripMove(angle);
    return;
  }
  if (strcmp(cmd, "POSES") == 0) {
    for (byte i=0; i<POSE_COUNT; ++i) {
      Serial.print(poses[i].name); Serial.print(F(": "));
      if (poses[i].defined) printPoint(poses[i].xyz);
      else Serial.print(F("UNDEFINED"));
      Serial.println();
    }
    return;
  }
  if (strcmp(cmd, "FK") == 0) {
    Serial.print(F("Predicted for commanded angles:"));
    printPoint(forwardKinematics(commanded)); Serial.println(); return;
  }
  if (strcmp(cmd, "SET") == 0) {
    if (moving || sequenceActive) { Serial.println(F("ERR: STOP first")); return; }
    int id = poseId(strtok(NULL, " ,\t"));
    float x,y,z;
    if (id < 0 || !readThree(x,y,z)) { Serial.println(F("ERR: SET NAME x y z")); return; }
    Point p = {x,y,z}; Joints q;
    if (!inverseKinematics(p,q)) { Serial.println(F("ERR: unreachable/limits")); return; }
    poses[id].xyz = p; poses[id].defined = true;
    Serial.print(F("SAVED ")); Serial.print(poses[id].name);
    printJoints(q); Serial.println(F(" (RAM only; not persisted)"));
    return;
  }
  if (strcmp(cmd, "CHECK") == 0 ||
    strcmp(cmd, "RUN") == 0 ||
    strcmp(cmd, "MANUAL") == 0) {

    bool isCheck = strcmp(cmd, "CHECK") == 0;
    bool isAutomatic = strcmp(cmd, "RUN") == 0;

    int color = colorId(strtok(NULL, " ,\t"));

    if (color < 0) {
      Serial.println(F("ERR: RED / GREEN / BLUE"));
      return;
    }

    if (moving || sequenceActive) {
      Serial.println(F("ERR: Robot busy"));
      return;
    }

    if (!checkRoute(color)) return;
    if (isCheck) return;

    if (!armed) {
      Serial.println(F("ERR: ARM first"));
      return;
    }
    if (gripperMoving) {
      Serial.println(F("ERR: wait for the gripper to stop"));
      return;
    }
    if (gripOpenAngle < 0 || gripClosedAngle < 0 ||
        gripOpenAngle == gripClosedAngle) {
      Serial.println(F("ERR: OPEN and CLOSED angles must differ"));
      return;
    }

    autoAdvance = isAutomatic;
    sequenceColor = color;
    sequenceStep = 0;
    sequenceActive = true;

    // Ensure gripper is open before descending towards the pickup point.
    if (gripCommanded != gripOpenAngle) {
      routeOpening = true;
      Serial.println(F("Opening gripper before route..."));
      startGripMove(gripOpenAngle);
    } else {
      startWaypoint();
    }
    return;
  }
  if (strcmp(cmd, "NEXT") == 0) {
    if (!sequenceActive || !waitingNext || moving) {
      Serial.println(F("ERR: NEXT only after previous step finishes")); return;
    }
    ++sequenceStep; startWaypoint(); return;
  }
  if (moving || sequenceActive) { Serial.println(F("ERR: STOP current operation first")); return; }
  if (strcmp(cmd, "M") == 0) {
    Joints q;
    if (!readThree(q.b,q.s,q.e)) { Serial.println(F("ERR: M b s e")); return; }
    startMove(q); return;
  }
  if (strcmp(cmd, "XYZ") == 0) {
    Point p; Joints q;
    if (!readThree(p.x,p.y,p.z) || !inverseKinematics(p,q)) {
      Serial.println(F("ERR: XYZ unreachable or incorrect")); return;
    }
    startMove(q); return;
  }
  if (strcmp(cmd, "HOME") == 0) {
    Joints q;
    if (!poses[HOME].defined || !inverseKinematics(poses[HOME].xyz,q)) {
      Serial.println(F("ERR: HOME not reachable")); return;
    }
    startMove(q); return;
  }
  Serial.println(F("Unknown command; send HELP"));
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("SEVEN 3-DOF + gripper / Arduino serial"));
  Serial.println(F("All servo outputs clamped to 0..180; NOT mechanical safety limits."));
  Serial.println(F("Before ARM: verify joints and GRIPPER 90 deg are safe."));
  showHelp();
  // No servos attached until ARM, to avoid motion at upload/reset.
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLength) {
        lineBuf[lineLength] = '\0';
        processLine(lineBuf);
        lineLength = 0;
      }
    } else if (lineLength < sizeof(lineBuf)-1) {
      lineBuf[lineLength++] = c;
    } else {
      lineLength = 0;
      Serial.println(F("ERR: command too long"));
    }
  }
  updateMotion();
  updateGripper();
  if (sequenceActive &&
    autoAdvance &&
    waitingNext &&
    !moving && !gripperMoving && !routeOpening && !routeGripActionPending &&
    millis() - waypointFinishedAt >= WAYPOINT_PAUSE_MS) {

    waitingNext = false;
    ++sequenceStep;
    startWaypoint();
  }
}