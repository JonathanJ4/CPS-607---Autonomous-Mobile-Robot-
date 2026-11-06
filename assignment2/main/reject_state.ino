const float REJECT_TURN_DEG = 300;            
const float REJECT_CM = 40;
const float REJECT_DETECT_CM = 50;
const unsigned long REJECT_IGNORE_MS = 300;   

bool rejectTurning = true;
unsigned long rejectTimer = 0;
unsigned long rejectTurnTime = 0;
unsigned long rejectDriveTime = 0;

void rejectStart() {
    float v, w;
    float phiTR = pwmToWheelSpeed(rightTurn());
    float phiTL = pwmToWheelSpeed(leftTurn());
    wheelsToVelocity(-phiTR, phiTL, v, w);
    rejectTurnTime = (REJECT_TURN_DEG * PI / 180.0) / fabs(w) * 1000;

    float phiDR = pwmToWheelSpeed(rightDrive());
    float phiDL = pwmToWheelSpeed(leftDrive());
    wheelsToVelocity(phiDR, phiDL, v, w);
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
        // turn done -> drive
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
        // after drive -> search again
        else if (elapsed >= rejectDriveTime) {
            changeState(SEARCH);
        }
    }
}