// Tunables
const unsigned long CHECK = 100;    // time between checks (ms)
const float STOP_CM = 15;           // close enough -> identify
const float MARGIN = 10;            // how much farther than last good = "bad" (cm)
const int MAX_BAD = 3;              // bad readings in a row -> lost the box

// Vars
unsigned long approachTimer = 0;
float lastGood = 0;
int badCount = 0;

void approachStart() {
    lastGood = distance;    // distance where search found it
    badCount = 0;
    approachTimer = millis();
    moveForward();
}

void printApproachReading(float dist, bool bad) {
    Serial.print("Approach | ");
    if (dist < 0) {
        Serial.print("No object");
    } else {
        Serial.print(dist);
        Serial.print(" cm");
    }
    Serial.print(" | last good ");
    Serial.print(lastGood);
    Serial.print(" | ");
    Serial.println(bad ? "BAD" : "OK");
}

void approachUpdate(float distance, bool print) {
    // only check once per CHECK so each check uses a fresh reading
    if (millis() - approachTimer < CHECK) return;
    approachTimer = millis();

    // close enough -> stop and identify
    if (distance > 0 && distance < STOP_CM) {
        if (print) printApproachReading(distance, false);
        changeState(IDENTIFY);
        return;
    }

    // bad = no echo, or farther than last good by more than the margin
    bool bad = (distance < 0) || (distance > lastGood + MARGIN);

    if (print) printApproachReading(distance, bad);

    if (bad) {
        badCount++;
        if (badCount >= MAX_BAD) {
            changeState(SEARCH);    // lost the box
        }
    } else {
        badCount = 0;
        lastGood = distance;
    }
}