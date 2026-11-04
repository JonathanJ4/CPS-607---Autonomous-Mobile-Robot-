// Tunables
const unsigned long BURST = 150;    // spin time per burst (ms)
const unsigned long PAUSE = 250;    // wait time before reading (ms)
const float THRESHOLD = 50;         // cm
const int MAX_BURSTS = 30;

// Vars
bool spinning = true;
unsigned long timer = 0;
int burstCount = 0;

void searchStart() {
    burstCount = 0;
    spinning = true;
    timer = millis();
    spinRight();
}

void printSearchReading(float dist) {
    Serial.print("Burst ");
    Serial.print(burstCount + 1);
    Serial.print("/");
    Serial.print(MAX_BURSTS);
    Serial.print(" | ");

    if (dist < 0) {
        Serial.println("No object -> FALSE");
    } else {
        Serial.print(dist);
        Serial.print(" < ");
        Serial.print(THRESHOLD);
        Serial.print(" -> ");
        Serial.println(dist < THRESHOLD ? "TRUE" : "FALSE");
    }
}

void searchUpdate(float distance, bool print) {
    if (spinning) {
        // burst done -> stop and wait
        if (millis() - timer >= BURST) {
            stopMove();
            spinning = false;
            timer = millis();
        }
    } else {
        // pause done -> check distance
        if (millis() - timer >= PAUSE) {
            if (print) printSearchReading(distance);

            if (distance > 0 && distance < THRESHOLD) {
                changeState(APPROACH);
            } else {
                burstCount++;
                if (burstCount >= MAX_BURSTS) {
                    changeState(RELOCATE);
                } else {
                    spinRight();
                    spinning = true;
                    timer = millis();
                }
            }
        }
    }
}