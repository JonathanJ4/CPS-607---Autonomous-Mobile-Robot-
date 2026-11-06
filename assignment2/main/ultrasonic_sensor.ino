const int TRIG_PIN = 3;
const int ECHO_PIN = 4;

const unsigned long PING_PERIOD = 40;
const unsigned long ECHO_TIMEOUT = 6000;   
unsigned long lastPingTime = 0;

// Distance + sliding window
const int WINDOW_SIZE = 3;
float readings[WINDOW_SIZE];
int readingIndex = 0;
int readingsCount = 0;
float distance = -1.0;  

void ultrasonicSetup() {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

float filterDistance(float newDistance) {
    readings[readingIndex] = newDistance;
    readingIndex = (readingIndex + 1) % WINDOW_SIZE;
    if (readingsCount < WINDOW_SIZE) readingsCount++;

    float total = 0;
    for (int i = 0; i < readingsCount; i++) total += readings[i];
    return total / readingsCount;
}

void readDistance() {
    if (millis() - lastPingTime < PING_PERIOD) return;
    lastPingTime = millis();

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    unsigned long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT);

    if (duration == 0) {
        // No echo: reset the window 
        readingsCount = 0;
        readingIndex = 0;
        distance = -1.0;
        return;
    }

    distance = filterDistance(duration * 0.0343 / 2.0);
}

void printUltrasonicSensorReading() {
    if (distance < 0) {
        Serial.println("Distance: no object");
    } else {
        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.println(" cm");
    }
}