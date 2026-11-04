const int LEFT_IR_PIN = 8;
const int RIGHT_IR_PIN = 7;

int left_ir_sensor = LOW;
int right_ir_sensor = LOW;

void irSensorSetup() {
    pinMode(LEFT_IR_PIN, INPUT);
    pinMode(RIGHT_IR_PIN, INPUT);
}

void readIrSensor() {
    left_ir_sensor = !digitalRead(LEFT_IR_PIN);
    right_ir_sensor = !digitalRead(RIGHT_IR_PIN);
}

void printIrSensorReading() {
    Serial.print("Left IR: ");
    Serial.print(left_ir_sensor);
    Serial.print(" | Right IR: ");
    Serial.println(right_ir_sensor);
}