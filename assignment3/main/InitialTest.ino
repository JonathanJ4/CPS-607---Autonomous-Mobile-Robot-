#include <math.h>

// =====================================================
// A03 WALL FOLLOWING (PI control + FSM)
// Robot starts automatically START_DELAY_MS after power-on/reset.
// =====================================================

// ================= TUNING VALUES (edit here) =================

// ---- Motor balance (per-wheel PWM scale) ----
const float RIGHT_SCALE = 0.69;   // right motor is stronger -> scaled down
const float LEFT_SCALE = 1.00;

// ---- Speed ----
const float FORWARD_SPEED = 20.0; // cm/s

// ---- Wall following ----
const float TARGET_DISTANCE = 15.0;  // cm
const float SENSOR_SPACING = 13.0;   // cm between FR and BR sensors (MEASURE IT)
const float MAX_OMEGA = 0.6;         // rad/s limit while following

// PI gains: omega = -(Kp_d*e + Ki_d*Int(e) + Kp_h*theta + Ki_h*Int(theta))
const float Kp_d = 0.035;
const float Ki_d = 0.004;
const float Kp_h = 1.5;
const float Ki_h = 0.0;
const float INTEGRAL_LIMIT = 10.0;

// ---- Corners ----
const float FRONT_TURN_DIST = 15.0;  // cm: front wall -> left turn
const int FRONT_HITS_NEEDED = 2;     // consecutive close readings
const float WALL_LOST_DIST = 35.0;   // cm: farther than this = no wall
const int LOST_COUNT = 2;            // consecutive lost readings = wall lost

const float TURN_OMEGA = 3.0;        // rad/s pivot rate
const float LEFT_TURN_SCALE = 1.0;   // raise if left pivot < 90 deg
const float RIGHT_TURN_SCALE = 1.0;  // raise if right pivot < 90 deg
const unsigned long ADVANCE_MS = 100; // drive on after BR loses wall, then pivot right

// ---- Misc ----
const unsigned long START_DELAY_MS = 3000;
const bool DEBUG = true;             // set false to disable serial prints

// =============================================================

// ---------------- PINS ----------------
const int FRONT_TRIG = 3;
const int FRONT_ECHO = 4;
const int FR_TRIG = 11;
const int FR_ECHO = 12;
const int BR_TRIG = 7;
const int BR_ECHO = 8;

const int RIGHT_MOTOR_1B = 5;
const int RIGHT_MOTOR_1A = 6;
const int LEFT_MOTOR_1B = 9;
const int LEFT_MOTOR_1A = 10;

// ---------------- ROBOT ----------------
const float WHEEL_RADIUS = 3.2;     // cm
const float HALF_WHEELBASE = 6.85;  // cm

// A01 calibration: phi (rad/s) = M_F * pwm + C_F
const float M_F = 0.0381;
const float C_F = 1.91;

const unsigned long ADVANCE_TIMEOUT_MS = 1500;
const unsigned long SEARCH_TIMEOUT_MS = 4000;
const unsigned long SETTLE_MS = 250;

const unsigned long LEFT_TURN_MS =
    (unsigned long)((PI / 2.0) / TURN_OMEGA * 1000.0 * LEFT_TURN_SCALE);
const unsigned long RIGHT_TURN_MS =
    (unsigned long)((PI / 2.0) / TURN_OMEGA * 1000.0 * RIGHT_TURN_SCALE);

// ---------------- SENSOR FILTERING / TIMING ----------------
const float FILTER_ALPHA = 0.5;  // EMA weight of new (median) value
const unsigned long SENSOR_INTERVAL = 35;
const unsigned long CONTROL_INTERVAL = 100;

const int S_FRONT = 0;
const int S_FR = 1;
const int S_BR = 2;

float filt[3] = {0, 0, 0};
float hist[3][3];
int histN[3] = {0, 0, 0};
int lostCnt[3] = {0, 0, 0};
bool haveData[3] = {false, false, false};
int frontHits = 0;

unsigned long lastSensorTime = 0;
unsigned long lastControlTime = 0;
int sensorIndex = 0;
bool sensorCycleComplete = false;

// ---------------- FSM ----------------
enum State {
  WAIT_START,
  FOLLOW,
  LEFT_PIVOT,
  RIGHT_ADVANCE,
  RIGHT_PIVOT,
  SETTLE,
  RIGHT_SEARCH,
  HALTED
};

State state = WAIT_START;
State afterSettle = FOLLOW;
unsigned long stateStart = 0;
bool brLostSeen = false;
unsigned long brLostAt = 0;

float integralDistance = 0;
float integralHeading = 0;

// =====================================================
// MOTORS (signed PWM: + = forward, - = reverse)
// =====================================================
void motorSetup() {
  pinMode(RIGHT_MOTOR_1A, OUTPUT);
  pinMode(RIGHT_MOTOR_1B, OUTPUT);
  pinMode(LEFT_MOTOR_1A, OUTPUT);
  pinMode(LEFT_MOTOR_1B, OUTPUT);
}

void stopMotors() {
  analogWrite(RIGHT_MOTOR_1A, 0);
  analogWrite(RIGHT_MOTOR_1B, 0);
  analogWrite(LEFT_MOTOR_1A, 0);
  analogWrite(LEFT_MOTOR_1B, 0);
}

void setMotorPWM(int rightPWM, int leftPWM) {
  rightPWM = constrain(rightPWM, -255, 255);
  leftPWM = constrain(leftPWM, -255, 255);

  // Right motor: forward on 1A
  if (rightPWM >= 0) {
    analogWrite(RIGHT_MOTOR_1A, rightPWM);
    analogWrite(RIGHT_MOTOR_1B, 0);
  } else {
    analogWrite(RIGHT_MOTOR_1A, 0);
    analogWrite(RIGHT_MOTOR_1B, -rightPWM);
  }

  // Left motor: forward on 1B
  if (leftPWM >= 0) {
    analogWrite(LEFT_MOTOR_1A, 0);
    analogWrite(LEFT_MOTOR_1B, leftPWM);
  } else {
    analogWrite(LEFT_MOTOR_1A, -leftPWM);
    analogWrite(LEFT_MOTOR_1B, 0);
  }
}

// Wheel speed (rad/s) -> signed PWM, with per-wheel scale
int wheelSpeedToPWM(float phi, float scale) {
  float mag = fabs(phi);
  int base = round((mag - C_F) / M_F);
  if (base <= 0) return 0;
  int pwm = constrain((int)round(base * scale), 0, 255);
  return (phi >= 0) ? pwm : -pwm;
}

// v in cm/s, omega in rad/s (positive omega = turn left / CCW)
void driveRobot(float v, float omega) {
  float phiRight = (v + HALF_WHEELBASE * omega) / WHEEL_RADIUS;
  float phiLeft = (v - HALF_WHEELBASE * omega) / WHEEL_RADIUS;
  setMotorPWM(wheelSpeedToPWM(phiRight, RIGHT_SCALE),
              wheelSpeedToPWM(phiLeft, LEFT_SCALE));
}

// =====================================================
// ULTRASONIC SENSORS
// =====================================================
void ultrasonicSetup() {
  pinMode(FRONT_TRIG, OUTPUT);
  pinMode(FRONT_ECHO, INPUT);
  pinMode(FR_TRIG, OUTPUT);
  pinMode(FR_ECHO, INPUT);
  pinMode(BR_TRIG, OUTPUT);
  pinMode(BR_ECHO, INPUT);
}

float readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 12000);  // 12 ms timeout
  if (duration == 0) return -1;

  float d = duration * 0.0343 / 2.0;
  if (d < 2 || d > 200) return -1;
  return d;
}

float median3(float a, float b, float c) {
  return max(min(a, b), min(max(a, b), c));
}

// Median-of-3 then exponential moving average.
// Invalid readings or readings beyond WALL_LOST_DIST count as "lost".
void processSideSample(int i, float raw) {
  bool valid = (raw > 0 && raw <= WALL_LOST_DIST);
  if (!valid) {
    if (lostCnt[i] < 100) lostCnt[i]++;
    return;
  }
  lostCnt[i] = 0;

  hist[i][2] = hist[i][1];
  hist[i][1] = hist[i][0];
  hist[i][0] = raw;
  if (histN[i] < 3) histN[i]++;

  float med = (histN[i] < 3) ? raw : median3(hist[i][0], hist[i][1], hist[i][2]);

  if (!haveData[i]) {
    filt[i] = med;
    haveData[i] = true;
  } else {
    filt[i] = FILTER_ALPHA * med + (1.0 - FILTER_ALPHA) * filt[i];
  }
}

void resetSideFilters() {
  for (int i = S_FR; i <= S_BR; i++) {
    histN[i] = 0;
    lostCnt[i] = 0;
    haveData[i] = false;
  }
  frontHits = 0;
  sensorIndex = 0;
  sensorCycleComplete = false;
}

// Non-blocking round robin: one sensor per SENSOR_INTERVAL
// (staggering avoids ultrasonic cross-talk)
void updateSensors() {
  unsigned long now = millis();
  if (now - lastSensorTime < SENSOR_INTERVAL) return;
  lastSensorTime = now;

  switch (sensorIndex) {
    case 0: {
      float d = readDistance(FRONT_TRIG, FRONT_ECHO);
      if (d > 0 && d < FRONT_TURN_DIST) frontHits++;
      else frontHits = 0;
      break;
    }
    case 1:
      processSideSample(S_FR, readDistance(FR_TRIG, FR_ECHO));
      break;
    case 2:
      processSideSample(S_BR, readDistance(BR_TRIG, BR_ECHO));
      sensorCycleComplete = true;
      break;
  }
  sensorIndex = (sensorIndex + 1) % 3;
}

// =====================================================
// PI WALL FOLLOWING
// =====================================================
void wallFollowController(float dt) {
  // Need both side readings to compute heading; otherwise go straight
  if (!haveData[S_FR] || !haveData[S_BR] || lostCnt[S_BR] >= LOST_COUNT) {
    driveRobot(FORWARD_SPEED, 0);
    return;
  }

  float fr = filt[S_FR];
  float br = filt[S_BR];

  // Heading relative to wall (+ = pointing away from wall)
  float theta = atan2(fr - br, SENSOR_SPACING);

  // Perpendicular distance from robot to wall
  float wallDistance = ((fr + br) / 2.0) * cos(theta);

  float distanceError = wallDistance - TARGET_DISTANCE;  // + = too far
  float headingError = theta;

  float omegaUnsat = -(Kp_d * distanceError + Ki_d * integralDistance +
                       Kp_h * headingError + Ki_h * integralHeading);

  // Anti-windup: only integrate while not saturated
  if (fabs(omegaUnsat) < MAX_OMEGA) {
    integralDistance = constrain(integralDistance + distanceError * dt,
                                 -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    integralHeading = constrain(integralHeading + headingError * dt,
                                -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  }

  float omega = constrain(omegaUnsat, -MAX_OMEGA, MAX_OMEGA);
  driveRobot(FORWARD_SPEED, omega);

  if (DEBUG) {
    Serial.print("FR:");
    Serial.print(fr, 1);
    Serial.print(" BR:");
    Serial.print(br, 1);
    Serial.print(" D:");
    Serial.print(wallDistance, 1);
    Serial.print(" Ang:");
    Serial.print(theta * 180.0 / PI, 1);
    Serial.print(" W:");
    Serial.println(omega, 3);
  }
}

// =====================================================
// FSM
// =====================================================
void enterState(State s) {
  state = s;
  stateStart = millis();

  switch (s) {
    case WAIT_START:
      stopMotors();
      break;
    case FOLLOW:
      integralDistance = 0;
      integralHeading = 0;
      lastControlTime = millis();
      sensorCycleComplete = false;
      if (DEBUG) Serial.println("-> FOLLOW");
      break;
    case LEFT_PIVOT:
      driveRobot(0, TURN_OMEGA);
      afterSettle = FOLLOW;
      if (DEBUG) Serial.println("-> LEFT_PIVOT");
      break;
    case RIGHT_ADVANCE:
      brLostSeen = false;
      driveRobot(FORWARD_SPEED, 0);
      if (DEBUG) Serial.println("-> RIGHT_ADVANCE");
      break;
    case RIGHT_PIVOT:
      driveRobot(0, -TURN_OMEGA);
      afterSettle = RIGHT_SEARCH;
      if (DEBUG) Serial.println("-> RIGHT_PIVOT");
      break;
    case SETTLE:
      stopMotors();
      resetSideFilters();
      if (DEBUG) Serial.println("-> SETTLE");
      break;
    case RIGHT_SEARCH:
      driveRobot(FORWARD_SPEED, 0);
      if (DEBUG) Serial.println("-> RIGHT_SEARCH");
      break;
    case HALTED:
      stopMotors();
      if (DEBUG) Serial.println("-> HALTED");
      break;
  }
}

void runFSM() {
  unsigned long now = millis();

  switch (state) {
    case WAIT_START:
      if (now - stateStart >= START_DELAY_MS) {
        resetSideFilters();
        enterState(FOLLOW);
      }
      break;

    case FOLLOW:
      // Priority 1: wall ahead -> left turn
      if (frontHits >= FRONT_HITS_NEEDED) {
        enterState(LEFT_PIVOT);
        return;
      }
      // Priority 2: lost the wall on the right -> right corner
      if (haveData[S_FR] && lostCnt[S_FR] >= LOST_COUNT) {
        enterState(RIGHT_ADVANCE);
        return;
      }
      // Normal PI control at fixed rate, once per fresh sensor cycle
      if (sensorCycleComplete && now - lastControlTime >= CONTROL_INTERVAL) {
        float dt = (now - lastControlTime) / 1000.0;
        lastControlTime = now;
        sensorCycleComplete = false;
        wallFollowController(dt);
      }
      break;

    case LEFT_PIVOT:
      if (now - stateStart >= LEFT_TURN_MS) enterState(SETTLE);
      break;

      case RIGHT_ADVANCE:
      // Pivot ADVANCE_MS after the front-right sensor lost the wall
      // (back-right sensor is no longer used)
      if (now - stateStart >= ADVANCE_MS) enterState(RIGHT_PIVOT);
      break;

    case RIGHT_PIVOT:
      if (now - stateStart >= RIGHT_TURN_MS) enterState(SETTLE);
      break;

    case SETTLE:
      if (now - stateStart >= SETTLE_MS) enterState(afterSettle);
      break;

    case RIGHT_SEARCH:
      // Drive straight until both side sensors see the new wall
      if (haveData[S_FR] && haveData[S_BR] &&
          lostCnt[S_FR] == 0 && lostCnt[S_BR] == 0) {
        enterState(FOLLOW);
      } else if (now - stateStart > SEARCH_TIMEOUT_MS) {
        enterState(HALTED);
      }
      break;

    case HALTED:
      stopMotors();
      break;
  }
}

// =====================================================
// SETUP / LOOP
// =====================================================
void setup() {
  if (DEBUG) Serial.begin(115200);
  motorSetup();
  ultrasonicSetup();
  stopMotors();
  enterState(WAIT_START);
}

void loop() {
  updateSensors();
  runFSM();
}