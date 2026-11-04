// Tunables
const unsigned long PUSH_TIMEOUT = 5000;   // give up if the line never shows (ms)
const unsigned long LOST_MS = 150;         // both IRs must stay not-both-on this long (ms)

// Vars
unsigned long pushTimer = 0;
unsigned long notBothTimer = 0;
bool notBoth = false;

void pushStart() {
    pushTimer = millis();
    notBoth = false;
    moveForward();
}

void printPushReading(bool left, bool right, int line) {
    Serial.print("Push | Left IR: ");
    Serial.print(left);
    Serial.print(" | Right IR: ");
    Serial.print(right);
    Serial.print(" | Line: ");
    Serial.println(line);
}

void pushUpdate(int leftIr, int rightIr, int line, bool print) {
    bool left = (leftIr == 1);
    bool right = (rightIr == 1);

    if (print) printPushReading(left, right, line);

    // reached the line -> box is across
    if (line == 1) {
        changeState(DONE);
        return;
    }

    // pushing for too long -> something is wrong
    if (millis() - pushTimer >= PUSH_TIMEOUT) {
        changeState(SEARCH);
        return;
    }

    // slipping: not both IRs on for LOST_MS -> re-align
    if (left && right) {
        notBoth = false;
    } else {
        if (!notBoth) {
            notBoth = true;
            notBothTimer = millis();
        } else if (millis() - notBothTimer >= LOST_MS) {
            changeState(IDENTIFY);
        }
    }
}