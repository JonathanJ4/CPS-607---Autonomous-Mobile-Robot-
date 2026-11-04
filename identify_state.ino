// Tunables
const unsigned long CORRECT_BURST = 60;     // small turn burst (ms), tune on floor
const unsigned long IDENTIFY_PAUSE = 250;   // settle time before reading (ms)
const int MAX_CORRECTIONS = 5;

// Vars
bool identifyTurning = false;
bool fromPush = false;
unsigned long identifyTimer = 0;
int corrections = 0;

void identifyStart(bool cameFromPush) {
    fromPush = cameFromPush;
    corrections = 0;
    identifyTurning = false;
    identifyTimer = millis();
}

void printIdentifyReading(bool left, bool right) {
    Serial.print("Identify | Left IR: ");
    Serial.print(left);
    Serial.print(" | Right IR: ");
    Serial.print(right);
    Serial.print(" | Corrections: ");
    Serial.print(corrections);
    Serial.print(" -> ");

    if (left && right) Serial.println("WHITE");
    else if (!left && !right && corrections == 0 && !fromPush) Serial.println("BLACK");
    else if (!left && !right) Serial.println("LOST");
    else Serial.println("OFF-ANGLE");
}

void identifyUpdate(int leftIr, int rightIr, bool print) {
    unsigned long wait = identifyTurning ? CORRECT_BURST : IDENTIFY_PAUSE;
    if (millis() - identifyTimer < wait) return;
    identifyTimer = millis();

    // burst finished -> stop, then wait IDENTIFY_PAUSE before reading
    if (identifyTurning) {
        stopMove();
        identifyTurning = false;
        return;
    }

    bool left = (leftIr == 1);
    bool right = (rightIr == 1);

    if (print) printIdentifyReading(left, right);

    if (left && right) {
        changeState(PUSH);                   // white box
    }
    else if (!left && !right) {
        // first read, not from a push -> black box. Otherwise we lost it.
        if (corrections == 0 && !fromPush) changeState(REJECT);
        else changeState(SEARCH);
    }
    else {
        // only one sees it -> turn toward that side
        if (corrections >= MAX_CORRECTIONS) {
            changeState(SEARCH);
            return;
        }
        corrections++;
        if (left) spinLeft();
        else spinRight();
        identifyTurning = true;
    }
}