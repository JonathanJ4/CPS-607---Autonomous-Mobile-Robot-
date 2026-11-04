// Pins
const int RIGHT_MOTOR_1B = 5;
const int RIGHT_MOTOR_1A = 6;
const int LEFT_MOTOR_1B = 9;
const int LEFT_MOTOR_1A = 10;

// Base speeds (drive = forward/backward, turn = spinning in place)
const int RIGHT_DRIVE = 150;
const int LEFT_DRIVE  = 150;
const int RIGHT_TURN  = 130;
const int LEFT_TURN   = 130;

// Calibration adjustments (tweak these if one wheel is faster).
// Positive = speed that wheel up, negative = slow it down.
const int RIGHT_ADJ = 10;
const int LEFT_ADJ  = 0;

// Final speeds, clamped to valid PWM range
int rightDrive() { return constrain(RIGHT_DRIVE + RIGHT_ADJ, 0, 255); }
int leftDrive()  { return constrain(LEFT_DRIVE  + LEFT_ADJ,  0, 255); }
int rightTurn()  { return constrain(RIGHT_TURN  + RIGHT_ADJ, 0, 255); }
int leftTurn()   { return constrain(LEFT_TURN   + LEFT_ADJ,  0, 255); }

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
    analogWrite(RIGHT_MOTOR_1A, rightDrive());
    analogWrite(RIGHT_MOTOR_1B, 0);
    analogWrite(LEFT_MOTOR_1A, 0);
    analogWrite(LEFT_MOTOR_1B, leftDrive());
}

void moveBackward() {
    analogWrite(RIGHT_MOTOR_1A, 0);
    analogWrite(RIGHT_MOTOR_1B, rightDrive());
    analogWrite(LEFT_MOTOR_1A, leftDrive());
    analogWrite(LEFT_MOTOR_1B, 0);
}

// Spin clockwise in place
void spinRight() {
    analogWrite(RIGHT_MOTOR_1A, 0);
    analogWrite(RIGHT_MOTOR_1B, rightTurn());
    analogWrite(LEFT_MOTOR_1A, 0);
    analogWrite(LEFT_MOTOR_1B, leftTurn());
}

// Spin counter-clockwise in place
void spinLeft() {
    analogWrite(RIGHT_MOTOR_1A, rightTurn());
    analogWrite(RIGHT_MOTOR_1B, 0);
    analogWrite(LEFT_MOTOR_1A, leftTurn());
    analogWrite(LEFT_MOTOR_1B, 0);
}