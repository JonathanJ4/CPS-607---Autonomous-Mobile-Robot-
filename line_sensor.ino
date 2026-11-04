// Pin
const int LINE_PIN = 2;

int line_sensor = HIGH;

void lineSensorSetup() {
    pinMode(LINE_PIN, INPUT);
}

void readLineSensor() {
    line_sensor = digitalRead(LINE_PIN);
}

void printLineSensorReading() {
    Serial.print("Line Sensor: ");
    Serial.println(line_sensor);
}