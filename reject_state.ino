// Tunables
const float REJECT_TURN_DEG = 180;            // tune on floor
const float REJECT_CM = 30;
const float REJECT_DETECT_CM = 50;
const unsigned long REJECT_IGNORE_MS = 300;   // ignore old readings after the turn

// Vars
bool rejectTurning = true;
unsigned long rejectTimer = 0;
unsigned long rejectTurnTime = 0;
unsigned long rejectDriveTime = 0;

void rejectStart() {
    float v, w;

    // time to turn: angle / angular speed
    float phiT = pwmToWheelSpeed(TURN_SPEED);
    wheelsToVelocity(-phiT, phiT, v, w);
    rejectTurnTime = (REJECT_TURN_DEG * PI / 180.0) / fabs(w) * 1000;

    // time to drive: distance / speed
    float phiD = pwmToWheelSpeed(DRIVE_SPEED);
    wheelsToVelocity(phiD, phiD, v, w);
    rejectDriveTime = (REJECT_CM / v) * 1000;

    rejectTurning = true;
    rejectTimer = millis();
    spinRight();
}

void printRejectReading(float dist) {
    Serial.print("Reject | ");
    Serial.print(rejectTurning ? "TURNING" : "FORWARD");
    Serial.print(" | Distance: ");
    Serial.println(dist);
}

void rejectUpdate(float distance, bool print) {
    unsigned long elapsed = millis() - rejectTimer;

    if (print) printRejectReading(distance);

    if (rejectTurning) {
        // turn done -> drive away
        if (elapsed >= rejectTurnTime) {
            rejectTurning = false;
            rejectTimer = millis();
            moveForward();
        }
    } else {
        // saw another box (after old readings have cleared) -> approach it
        if (elapsed >= REJECT_IGNORE_MS && distance > 0 && distance < REJECT_DETECT_CM) {
            changeState(APPROACH);
        }
        // drove far enough -> search again
        else if (elapsed >= rejectDriveTime) {
            changeState(SEARCH);
        }
    }
}