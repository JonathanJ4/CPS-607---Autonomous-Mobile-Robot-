const float DONE_DISTANCE = 5;           // back off the line (cm)

bool doneBacking = false;
unsigned long doneTime = 0;
unsigned long doneTimer = 0;

void doneStart() {
    float phiR = pwmToWheelSpeed(rightDrive());
    float phiL = pwmToWheelSpeed(leftDrive());
    float v, w;
    wheelsToVelocity(phiR, phiL, v, w);

    doneTime = (DONE_DISTANCE / v) * 1000;
    doneBacking = true;
    doneTimer = millis();
    moveBackward();
}

void doneUpdate() {
    if (!doneBacking) return;

    if (millis() - doneTimer >= doneTime) {
        stopMove();
        doneBacking = false;
        Serial.println("DONE - box pushed out");
    }
}