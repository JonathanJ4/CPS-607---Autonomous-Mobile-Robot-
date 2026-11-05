// Tunables
const float RELOCATE_TURN_DEG = 45;           // spin before driving (deg), change later
const float RELOCATE_CM = 40;                 // distance to move (cm)
const float RELOCATE_DETECT_CM = 50;          // saw a box closer than this -> approach
const unsigned long RELOCATE_IGNORE_MS = 300; // ignore old readings after the turn

// Vars
bool relocateTurning = true;
unsigned long relocateTurnTime = 0;   // how long to turn (ms), calculated
unsigned long relocateDriveTime = 0;  // how long to drive (ms), calculated
unsigned long relocateTimer = 0;

void relocateStart() {
    float v, w;

    // time to turn: angle / angular speed
    float phiTR = pwmToWheelSpeed(rightTurn());
    float phiTL = pwmToWheelSpeed(leftTurn());
    wheelsToVelocity(-phiTR, phiTL, v, w);
    relocateTurnTime = (RELOCATE_TURN_DEG * PI / 180.0) / fabs(w) * 1000;

    // time to drive: distance / speed
    float phiDR = pwmToWheelSpeed(rightDrive());
    float phiDL = pwmToWheelSpeed(leftDrive());
    wheelsToVelocity(phiDR, phiDL, v, w);
    relocateDriveTime = (RELOCATE_CM / v) * 1000;

    relocateTurning = true;
    relocateTimer = millis();
    spinRight();
}

void printRelocateReading(unsigned long elapsed, float dist) {
    Serial.print("Relocate | ");
    Serial.print(relocateTurning ? "TURNING " : "FORWARD ");
    Serial.print(elapsed);
    Serial.print("/");
    Serial.print(relocateTurning ? relocateTurnTime : relocateDriveTime);
    Serial.print(" ms | Distance: ");
    Serial.println(dist);
}

void relocateUpdate(float distance, bool print) {
    unsigned long elapsed = millis() - relocateTimer;

    if (print) printRelocateReading(elapsed, distance);

    if (relocateTurning) {
        // saw a box while turning -> approach it
        if (distance > 0 && distance < RELOCATE_DETECT_CM) {
            changeState(APPROACH);
        }
        // turn done -> drive
        else if (elapsed >= relocateTurnTime) {
            relocateTurning = false;
            relocateTimer = millis();
            moveForward();
        }
    } else {
        // saw a box (after old readings have cleared) -> approach it
        if (elapsed >= RELOCATE_IGNORE_MS && distance > 0 && distance < RELOCATE_DETECT_CM) {
            changeState(APPROACH);
        }
        // drove far enough -> search again
        else if (elapsed >= relocateDriveTime) {
            changeState(SEARCH);
        }
    }
}