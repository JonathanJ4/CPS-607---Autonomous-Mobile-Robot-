// Tunables
const float RELOCATE_CM = 15;      // distance to move (cm)

// Vars
unsigned long relocateTime = 0;    // how long to drive (ms), calculated
unsigned long relocateTimer = 0;

void relocateStart() {
    // DRIVE_SPEED (PWM) -> wheel speed -> robot speed
    float phi = pwmToWheelSpeed(DRIVE_SPEED);
    float v, w;
    wheelsToVelocity(phi, phi, v, w);

    // time = distance / speed
    relocateTime = (RELOCATE_CM / v) * 1000;

    relocateTimer = millis();
    moveForward();
}

void printRelocateReading(unsigned long elapsed) {
    Serial.print("Relocate | ");
    Serial.print(elapsed);
    Serial.print("/");
    Serial.print(relocateTime);
    Serial.println(" ms");
}

void relocateUpdate(bool print) {
    unsigned long elapsed = millis() - relocateTimer;

    if (print) printRelocateReading(elapsed);

    if (elapsed >= relocateTime) {
        changeState(SEARCH);
    }
}