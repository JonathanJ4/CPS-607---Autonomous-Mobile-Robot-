// Pins
const int RIGHT_MOTOR_1B = 5;
const int RIGHT_MOTOR_1A = 6;
const int LEFT_MOTOR_1B = 9;
const int LEFT_MOTOR_1A = 10;

// Speeds
const int DRIVE_SPEED = 150;   // forward / backward
const int TURN_SPEED = 130;    // spinning in place

void motorSetup() {
    pinMode(RIGHT_MOTOR_1A, OUTPUT);
    pinMode(RIGHT_MOTOR_1B, OUTPUT);
    pinMode(LEFT_MOTOR_1A, OUTPUT);
    pinMode(LEFT_MOTOR_1B, OUTPUT);
}

void stopMove() {
    analogWrite(RIGHT_MOTOR_1A, 0);
    analogWrite(RIGHT_MOTOR_1B, 0);
    analogWrite(LEFT_MOTOR_1A, 0);
    analogWrite(LEFT_MOTOR_1B, 0);
}

void moveForward() {
    analogWrite(RIGHT_MOTOR_1A, DRIVE_SPEED);
    analogWrite(RIGHT_MOTOR_1B, 0);
    analogWrite(LEFT_MOTOR_1A, 0);
    analogWrite(LEFT_MOTOR_1B, DRIVE_SPEED);
}

void moveBackward() {
    analogWrite(RIGHT_MOTOR_1A, 0);
    analogWrite(RIGHT_MOTOR_1B, DRIVE_SPEED);
    analogWrite(LEFT_MOTOR_1A, DRIVE_SPEED);
    analogWrite(LEFT_MOTOR_1B, 0);
}

// Spin clockwise in place
void spinRight() {
    analogWrite(RIGHT_MOTOR_1A, 0);
    analogWrite(RIGHT_MOTOR_1B, TURN_SPEED);
    analogWrite(LEFT_MOTOR_1A, 0);
    analogWrite(LEFT_MOTOR_1B, TURN_SPEED);
}

// Spin counter-clockwise in place
void spinLeft() {
    analogWrite(RIGHT_MOTOR_1A, TURN_SPEED);
    analogWrite(RIGHT_MOTOR_1B, 0);
    analogWrite(LEFT_MOTOR_1A, TURN_SPEED);
    analogWrite(LEFT_MOTOR_1B, 0);
}