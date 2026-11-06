const unsigned long CORRECT_BURST = 60;     // max small turn burst (ms)
const unsigned long IDENTIFY_PAUSE = 250;   // both IRs must stay off this long before deciding black / lost (ms)
const int MAX_CORRECTIONS = 5;

bool identifyTurning = false;
bool fromPush = false;
unsigned long identifyTimer = 0;   
unsigned long bothOffTimer = 0;    
bool bothOff = false;
int corrections = 0;

void identifyStart(bool cameFromPush) {
    fromPush = cameFromPush;
    corrections = 0;
    identifyTurning = false;
    bothOff = false;
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
    bool left = (leftIr == 1);
    bool right = (rightIr == 1);

    // both ON -> white 
    if (left && right) {
        if (print) printIdentifyReading(left, right);
        changeState(PUSH);
        return;
    }

    // mid-turn: keep turning until the burst runs out
    if (identifyTurning) {
        if (millis() - identifyTimer >= CORRECT_BURST) {
            stopMove();
            identifyTurning = false;
        }
        return;
    }

    if (!left && !right) {
        if (!bothOff) {
            bothOff = true;
            bothOffTimer = millis();
            return;
        }
        if (millis() - bothOffTimer < IDENTIFY_PAUSE) return;

        if (print) printIdentifyReading(left, right);

        // first read, not from a push -> black box. Otherwise we lost it.
        if (corrections == 0 && !fromPush) changeState(REJECT);
        else changeState(SEARCH);
        return;
    }

    bothOff = false;

    if (corrections >= MAX_CORRECTIONS) {
        changeState(SEARCH);
        return;
    }

    if (print) printIdentifyReading(left, right);

    corrections++;
    if (left) spinLeft();
    else spinRight();
    identifyTurning = true;
    identifyTimer = millis();
}