extern float distance;
extern int line_sensor;
extern int left_ir_sensor;
extern int right_ir_sensor;

enum State { SEARCH,
             APPROACH,
             IDENTIFY,
             REJECT,
             PUSH,
             DONE,
             RELOCATE,
             RECOVER };
State currentState = SEARCH;

// u = ultrasonic | i = IR | l = line | s = search | a = approach | d = identify | e = reject | p = push | r = relocate | c = recover | b = all | x = stop
char printMode = 'b';
unsigned long lastPrintTime = 0;
const unsigned long PRINT_PERIOD = 200;

void changeState(int newState) {
  State previous = currentState;
  currentState = (State)newState;
  stopMove();

  Serial.print("State -> ");
  if (newState == SEARCH) Serial.println("SEARCH");
  if (newState == APPROACH) Serial.println("APPROACH");
  if (newState == IDENTIFY) Serial.println("IDENTIFY");
  if (newState == REJECT) Serial.println("REJECT");
  if (newState == PUSH) Serial.println("PUSH");
  if (newState == DONE) Serial.println("DONE");
  if (newState == RELOCATE) Serial.println("RELOCATE");
  if (newState == RECOVER) Serial.println("RECOVER");

  if (newState == SEARCH) searchStart();
  if (newState == APPROACH) approachStart();
  if (newState == IDENTIFY) identifyStart(previous == PUSH);
  if (newState == REJECT) rejectStart();
  if (newState == PUSH) pushStart();
  if (newState == DONE) doneStart();
  if (newState == RELOCATE) relocateStart();
  if (newState == RECOVER) recoverStart();
}

void setup() {
  Serial.begin(9600);

  motorSetup();
  ultrasonicSetup();
  irSensorSetup();
  lineSensorSetup();

  Serial.println("Print modes: u = Ultrasonic | i = IR | l = Line | s = Search | a = Approach | d = Identify | e = Reject | p = Push | r = Relocate | c = Recover | b = All | x = Stop");

  delay(2000); 
  changeState(SEARCH);
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (strchr("uilsadeprcbx", c) != NULL) printMode = c;
  }

  // Read all sensors every loop
  readDistance();
  readIrSensor();
  readLineSensor();

  // Sensor printing 
  if (millis() - lastPrintTime >= PRINT_PERIOD) {
    lastPrintTime = millis();

    if (printMode == 'u' || printMode == 'b') printUltrasonicSensorReading();
    if (printMode == 'i' || printMode == 'b') printIrSensorReading();
    if (printMode == 'l' || printMode == 'b') printLineSensorReading();
  }

  // Boundary check 
  if (line_sensor == 1 && currentState != RECOVER && currentState != PUSH && currentState != DONE) {
    changeState(RECOVER);
  }

  // FSM
  switch (currentState) {
    case SEARCH:
      searchUpdate(distance, printMode == 's' || printMode == 'b');
      break;

    case APPROACH:
      approachUpdate(distance, printMode == 'a' || printMode == 'b');
      break;

    case IDENTIFY:
      identifyUpdate(left_ir_sensor, right_ir_sensor, printMode == 'd' || printMode == 'b');
      break;

    case REJECT:
      rejectUpdate(distance, printMode == 'e' || printMode == 'b');
      break;

    case PUSH:
      pushUpdate(left_ir_sensor, right_ir_sensor, line_sensor, printMode == 'p' || printMode == 'b');
      break;

    case DONE:
      doneUpdate();
      break;

    case RELOCATE:
      relocateUpdate(distance, printMode == 'r' || printMode == 'b');
      break;

    case RECOVER:
      recoverUpdate(printMode == 'c' || printMode == 'b');
      break;
  }
}