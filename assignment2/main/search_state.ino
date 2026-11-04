// Tunables
const unsigned long BURST = 150;    // spin time per burst (ms)
const unsigned long PAUSE = 200;    // settle time before confirming (ms), >= WINDOW_SIZE * PING_PERIOD_MS
const float THRESHOLD = 50;         // cm
const int MAX_BURSTS = 30;

// Vars
bool spinning = true;
bool suspected = false;             // saw something while spinning, waiting to confirm
unsigned long timer = 0;
int burstCount = 0;

void searchStart() {
    burstCount = 0;
    spinning = true;
    suspected = false;
    timer = millis();
    spinRight();
}

void printSearchReading(float dist) {
    Serial.print("Burst ");
    Serial.print(burstCount + 1);
    Serial.print("/");
    Serial.print(MAX_BURSTS);
    Serial.print(" | ");
    Serial.print(suspected ? "Confirming | " : "");

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
        // saw something mid-burst -> stop right away, then confirm after the pause
        if (distance > 0 && distance < THRESHOLD) {
            stopMove();
            spinning = false;
            suspected = true;
            timer = millis();
        }
        // burst done -> stop and wait
        else if (millis() - timer >= BURST) {
            stopMove();
            spinning = false;
            suspected = false;
            timer = millis();
        }
    } else {
        // pause done -> check distance on fresh readings
        if (millis() - timer >= PAUSE) {
            if (print) printSearchReading(distance);

            if (distance > 0 && distance < THRESHOLD) {
                changeState(APPROACH);
            } else {
                // nothing there (or false alarm) -> next burst
                burstCount++;
                if (burstCount >= MAX_BURSTS) {
                    changeState(RELOCATE);
                } else {
                    spinRight();
                    spinning = true;
                    suspected = false;
                    timer = millis();
                }
            }
        }
    }
}