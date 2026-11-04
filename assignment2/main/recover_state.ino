// Tunables
const float RECOVER_CM = 20;       // distance to back up (cm)

// Vars
unsigned long recoverTime = 0;     // how long to drive back (ms), calculated
unsigned long recoverTimer = 0;

void recoverStart() {
    // drive speeds (PWM) -> wheel speed -> robot speed
    float phiR = pwmToWheelSpeed(rightDrive());
    float phiL = pwmToWheelSpeed(leftDrive());
    float v, w;
    wheelsToVelocity(phiR, phiL, v, w);

    // time = distance / speed
    recoverTime = (RECOVER_CM / v) * 1000;

    recoverTimer = millis();
    moveBackward();
}

void printRecoverReading(unsigned long elapsed) {
    Serial.print("Recover | ");
    Serial.print(elapsed);
    Serial.print("/");
    Serial.print(recoverTime);
    Serial.println(" ms");
}

void recoverUpdate(bool print) {
    unsigned long elapsed = millis() - recoverTimer;

    if (print) printRecoverReading(elapsed);

    if (elapsed >= recoverTime) {
        changeState(SEARCH);
    }
}