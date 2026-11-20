#include <math.h>

// ---- Motor balance (per-wheel PWM scale) ----
const float RIGHT_SCALE = 0.62;   
const float LEFT_SCALE = 1.00;

// ---- Speed ----
const float FORWARD_SPEED = 26.0; // cm/s

// ---- Wall following ----
const float TARGET_DISTANCE = 8;  // cm
const float SENSOR_SPACING = 14.0;   // cm between FR and BR sensors
const float MAX_OMEGA = 0.8;         // rad/s limit while following

// PI gains: omega = -(Kp_d*e + Ki_d*Int(e) + Kp_h*theta + Ki_h*Int(theta))
const float Kp_d = 0.035;
const float Ki_d = 0.004;
const float Kp_h = 1.5;
const float Ki_h = 0.1;
const float INTEGRAL_LIMIT = 10.0;

// ---- Smoothness / responsiveness ----
const float DIST_DEADBAND = 0.5;     // cm
const float HEAD_DEADBAND = 0.02;    // rad 
const float OMEGA_TAU = 0.08;       
const float OMEGA_SLEW = 4.0;        // rad/s^2

// ---- Left corner (in-place pivot) ----
const float FRONT_TURN_DIST = 22.0;  // cm
const int FRONT_HITS_NEEDED = 2;     // consecutive close readings
const float TURN_OMEGA = 3.0;        // rad/s pivot rate
const float LEFT_TURN_SCALE = 1.3;   

// ---- Right corner (curved turn) ----
// Arc radius = CURVE_SPEED / CURVE_OMEGA 
const float CURVE_SPEED = 20.0;      // cm/s speed during the curve
const float CURVE_OMEGA = 0.8;       // rad/s during the curve
const int FR_REGAIN_COUNT = 2;       // consecutive valid FR readings to end the curve
const unsigned long CURVE_TIMEOUT_MS = 4000;  // give up if FR never sees a wall

const float WALL_LOST_DIST = 35.0;   // cm
const float CORNER_MAX_DIST = 28.0;  // FR must have been this close to count as a corner
const int LOST_COUNT = 2;            // consecutive lost readings = wall lost

// ---- Misc ----
const unsigned long START_DELAY_MS = 3000;
const bool DEBUG = true;             
const int DEBUG_EVERY = 3;          

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

const float M_F = 0.0381;
const float C_F = 1.91;

const unsigned long SETTLE_MS = 250;

const unsigned long LEFT_TURN_MS =
    (unsigned long)((PI / 2.0) / TURN_OMEGA * 1000.0 * LEFT_TURN_SCALE);

// ---------------- SENSOR FILTERING / TIMING ----------------
const float FILTER_ALPHA = 0.6;  
const unsigned long SENSOR_INTERVAL = 15;   // ms between single-sensor reads
const unsigned long CONTROL_INTERVAL = 40;  // min ms between controller updates
const unsigned long SMOOTH_INTERVAL = 10;   // ms between motor command updates

const int S_FRONT = 0;
const int S_FR = 1;
const int S_BR = 2;

// Side pair refreshes every 2 slots (40 ms), front every 5 slots (100 ms).
const int PATTERN_LEN = 5;
const int SLOT_PATTERN[PATTERN_LEN] = {S_FR, S_BR, S_FR, S_BR, S_FRONT};

float filt[3] = {0, 0, 0};
float hist[3][3];
int histN[3] = {0, 0, 0};
int lostCnt[3] = {0, 0, 0};
int validStreak[3] = {0, 0, 0};  
bool haveData[3] = {false, false, false};
int frontHits = 0;

unsigned long lastSensorTime = 0;
unsigned long lastControlTime = 0;
unsigned long lastSmoothTime = 0;
int sensorIndex = 0;
bool sensorCycleComplete = false;
int debugCounter = 0;

// ---------------- FSM ----------------
enum State {
  WAIT_START,
  FOLLOW,
  LEFT_PIVOT,
  SETTLE,
  RIGHT_CURVE,
  HALTED
};

State state = WAIT_START;
unsigned long stateStart = 0;

float integralDistance = 0;
float integralHeading = 0;

float omegaTarget = 0;  
float omegaCmd = 0;   

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

  if (rightPWM >= 0) {
    analogWrite(RIGHT_MOTOR_1A, rightPWM);
    analogWrite(RIGHT_MOTOR_1B, 0);
  } else {
    analogWrite(RIGHT_MOTOR_1A, 0);
    analogWrite(RIGHT_MOTOR_1B, -rightPWM);
  }

  if (leftPWM >= 0) {
    analogWrite(LEFT_MOTOR_1A, 0);
    analogWrite(LEFT_MOTOR_1B, leftPWM);
  } else {
    analogWrite(LEFT_MOTOR_1A, -leftPWM);
    analogWrite(LEFT_MOTOR_1B, 0);
  }
}

// Wheel speed (rad/s)
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

  unsigned long duration = pulseIn(echoPin, HIGH, 12000); 
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
    validStreak[i] = 0;
    return;
  }
  lostCnt[i] = 0;
  if (validStreak[i] < 100) validStreak[i]++;

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
    validStreak[i] = 0;
    haveData[i] = false;
  }
  frontHits = 0;
  sensorIndex = 0;
  sensorCycleComplete = false;
}

// Non-blocking round robin
void updateSensors() {
  unsigned long now = millis();
  if (now - lastSensorTime < SENSOR_INTERVAL) return;
  lastSensorTime = now;

  switch (SLOT_PATTERN[sensorIndex]) {
    case S_FRONT: {
      float d = readDistance(FRONT_TRIG, FRONT_ECHO);
      if (d > 0 && d < FRONT_TURN_DIST) frontHits++;
      else frontHits = 0;
      break;
    }
    case S_FR:
      processSideSample(S_FR, readDistance(FR_TRIG, FR_ECHO));
      break;
    case S_BR:
      processSideSample(S_BR, readDistance(BR_TRIG, BR_ECHO));
      sensorCycleComplete = true;  // a fresh FR+BR pair is available
      break;
  }
  sensorIndex = (sensorIndex + 1) % PATTERN_LEN;
}

// =====================================================
// PI Controller
// =====================================================

// Shrinks the error toward zero inside +-db
float softDeadband(float x, float db) {
  if (x > db) return x - db;
  if (x < -db) return x + db;
  return 0;
}

// Computes omegaTarget only. Does not touch the motors.
void wallFollowController(float dt) {
  // Need both side readings to compute heading; otherwise go straight
  if (!haveData[S_FR] || !haveData[S_BR] || lostCnt[S_BR] >= LOST_COUNT) {
    omegaTarget = 0;
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

  float dErrP = softDeadband(distanceError, DIST_DEADBAND);
  float hErrP = softDeadband(headingError, HEAD_DEADBAND);

  float omegaUnsat = -(Kp_d * dErrP + Ki_d * integralDistance +
                       Kp_h * hErrP + Ki_h * integralHeading);

  if (fabs(omegaUnsat) < MAX_OMEGA) {
    integralDistance = constrain(integralDistance + distanceError * dt,
                                 -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    integralHeading = constrain(integralHeading + headingError * dt,
                                -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  }

  omegaTarget = constrain(omegaUnsat, -MAX_OMEGA, MAX_OMEGA);

  if (DEBUG && (++debugCounter >= DEBUG_EVERY)) {
    debugCounter = 0;
    Serial.print("FR:");
    Serial.print(fr, 1);
    Serial.print(" BR:");
    Serial.print(br, 1);
    Serial.print(" D:");
    Serial.print(wallDistance, 1);
    Serial.print(" Ang:");
    Serial.print(theta * 180.0 / PI, 1);
    Serial.print(" Wt:");
    Serial.print(omegaTarget, 3);
    Serial.print(" Wc:");
    Serial.println(omegaCmd, 3);
  }
}

void smoothAndDrive() {
  unsigned long now = millis();
  if (now - lastSmoothTime < SMOOTH_INTERVAL) return;
  float dt = (now - lastSmoothTime) / 1000.0;
  lastSmoothTime = now;

  float a = dt / (OMEGA_TAU + dt);
  float desired = omegaCmd + a * (omegaTarget - omegaCmd);

  float maxStep = OMEGA_SLEW * dt;
  omegaCmd += constrain(desired - omegaCmd, -maxStep, maxStep);

  driveRobot(FORWARD_SPEED, omegaCmd);
}

// =====================================================
// FSM
// =====================================================
void enterState(State s) {
  state = s;
  stateStart = millis();

  switch (s) {
    case WAIT_START:
      omegaCmd = 0;
      omegaTarget = 0;
      stopMotors();
      break;
    case FOLLOW:
      integralDistance = 0;
      integralHeading = 0;
      omegaTarget = 0;
      lastControlTime = millis();
      lastSmoothTime = millis();
      sensorCycleComplete = false;
      if (DEBUG) Serial.println("-> FOLLOW");
      break;
    case LEFT_PIVOT:
      driveRobot(0, TURN_OMEGA);
      if (DEBUG) Serial.println("-> LEFT_PIVOT");
      break;
    case SETTLE:
      omegaCmd = 0;
      omegaTarget = 0;
      stopMotors();
      resetSideFilters();
      if (DEBUG) Serial.println("-> SETTLE");
      break;
    case RIGHT_CURVE:
      histN[S_FR] = 0;
      haveData[S_FR] = false;
      validStreak[S_FR] = 0;
      omegaCmd = -CURVE_OMEGA; 
      driveRobot(CURVE_SPEED, -CURVE_OMEGA);
      if (DEBUG) {
        Serial.print("-> RIGHT_CURVE  last FR=");
        Serial.print(filt[S_FR], 1);
        Serial.print(" BR=");
        Serial.println(filt[S_BR], 1);
      }
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
      // Priority 1: wall ahead -> left turn (in place)
      if (frontHits >= FRONT_HITS_NEEDED) {
        enterState(LEFT_PIVOT);
        return;
      }
      // Priority 2: front-right sensor lost the wall -> curve right.
      if (haveData[S_FR] && lostCnt[S_FR] >= LOST_COUNT &&
          filt[S_FR] < CORNER_MAX_DIST) {
        enterState(RIGHT_CURVE);
        return;
      }
      // PI controller, once per fresh FR+BR pair
      if (sensorCycleComplete && now - lastControlTime >= CONTROL_INTERVAL) {
        float dt = (now - lastControlTime) / 1000.0;
        lastControlTime = now;
        sensorCycleComplete = false;
        wallFollowController(dt);
      }
      smoothAndDrive();
      break;

    case LEFT_PIVOT:
      if (now - stateStart >= LEFT_TURN_MS) enterState(SETTLE);
      break;

    case SETTLE:
      if (now - stateStart >= SETTLE_MS) enterState(FOLLOW);
      break;

    case RIGHT_CURVE:
      // Keep arcing right until the front-right sensor sees a wall again
      if (haveData[S_FR] && validStreak[S_FR] >= FR_REGAIN_COUNT) {
        if (DEBUG) {
          Serial.print("FR regained: ");
          Serial.println(filt[S_FR], 1);
        }
        enterState(FOLLOW);
      } else if (now - stateStart > CURVE_TIMEOUT_MS) {
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
