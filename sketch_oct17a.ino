const int RIGHT_MOTOR_1B = 5;
const int RIGHT_MOTOR_1A = 6;
const int LEFT_MOTOR_1B = 9;
const int LEFT_MOTOR_1A = 10;

const double PWM_SPEED_50_PERCENT = 127.5;
const double PWM_SPEED_60_PERCENT = 153;
const double PWM_SPEED_70_PERCENT = 178.5;
const double PWM_SPEED_80_PERCENT = 204;
const double PWM_SPEED_90_PERCENT = 229.5;
const double PWM_SPEED_100_PERCENT = 255;


void setup() {
  pinMode(RIGHT_MOTOR_1B, OUTPUT);
  pinMode(RIGHT_MOTOR_1A, OUTPUT);
  pinMode(LEFT_MOTOR_1B, OUTPUT);
  pinMode(LEFT_MOTOR_1A, OUTPUT);
}

void loop() {
  moveForward(PWM_SPEED_100_PERCENT, PWM_SPEED_100_PERCENT);
  delay(5000);
  stopMove();
  delay(7000);
}

void moveForward(double rightSpeed, double leftSpeed) {
    analogWrite(RIGHT_MOTOR_1A, rightSpeed);
    analogWrite(LEFT_MOTOR_1B, leftSpeed);
    
    analogWrite(RIGHT_MOTOR_1B, 0);
    analogWrite(LEFT_MOTOR_1A, 0);
}




void turnRight() {
  digitalWrite(RIGHT_MOTOR_1B, LOW);
  digitalWrite(RIGHT_MOTOR_1A, LOW);
  digitalWrite(LEFT_MOTOR_1B, HIGH);
  digitalWrite(LEFT_MOTOR_1A, LOW);
}

void stopMove() {
  analogWrite(RIGHT_MOTOR_1B, 0);
  analogWrite(RIGHT_MOTOR_1A, 0);
  analogWrite(LEFT_MOTOR_1B, 0);
  analogWrite(LEFT_MOTOR_1A, 0);
}

