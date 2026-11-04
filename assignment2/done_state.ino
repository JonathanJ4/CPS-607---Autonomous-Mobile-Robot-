// Tunables
extern const int DRIVE_SPEED;
const float DONE_CM = 5;           // back off the line (cm)

// Vars
bool doneBacking = false;
unsigned long doneTime = 0;
unsigned long doneTimer = 0;

void doneStart() {
    float phi = pwmToWheelSpeed(DRIVE_SPEED);
    float v, w;
    wheelsToVelocity(phi, phi, v, w);

    doneTime = (DONE_CM / v) * 1000;
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