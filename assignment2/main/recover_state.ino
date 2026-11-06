const float RECOVER_CM = 20;       // distance to back up (cm)

unsigned long recoverTime = 0;     
unsigned long recoverTimer = 0;

void recoverStart() {
    float phiR = pwmToWheelSpeed(rightDrive());
    float phiL = pwmToWheelSpeed(leftDrive());
    float v, w;
    wheelsToVelocity(phiR, phiL, v, w);

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