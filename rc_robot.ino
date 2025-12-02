#include <AFMotor.h>          // For L293D Motor Shield
#include <LiquidCrystal_I2C.h> // For LCD
#include <SoftwareSerial.h>   // For Bluetooth
#include <Servo.h>            // For Servo Motor

// Motor setup
AF_DCMotor motor1(1), motor2(2), motor3(3), motor4(4);

// LCD setup
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Bluetooth setup
SoftwareSerial bluetooth(2, 3); // RX, TX

// Ultrasonic setup
const int trigPin = 9;
const int echoPin = 10;

// Servo setup
Servo servo;
const int servoPin = 11;

// Constants
const int OBSTACLE_THRESHOLD = 60; // cm
const int CRITICAL_THRESHOLD = 20; // cm
const int NUM_READINGS = 5;
const int MIN_MOTOR_SPEED = 100;
const int MAX_MOTOR_SPEED = 255;
const int COMMAND_TIMEOUT = 5000; // ms
const int ACTION_DURATION = 2000; // ms
const int SERVO_MIN = 0;
const int SERVO_MAX = 180;
const int SCAN_ANGLES[] = {0, 45, 90, 135, 180}; // Full sweep
const int NUM_SCAN_ANGLES = 5;
const int TURN_DURATION = 600;    // ms
const int RAMP_DELAY = 50;        // ms
const int SERVO_DELAY = 200;      // ms

// Global Variables
int currentSpeed = MAX_MOTOR_SPEED;
bool isMoving = false;
unsigned long lastCommandTime = 0;
bool autonomousMode = true;
unsigned long lastBlinkTime = 0;
bool eyesOpen = true;
unsigned long actionTime = 0;
bool showingAction = false;
int servoAngle = 90;
bool manualOverride = false;

// Reduced path memory to 5 entries
struct Move {
  char action;
  unsigned long timestamp;
};
Move pathMemory[5];
int moveIndex = 0;

// Eye characters (2 instead of 8)
byte eyeOpen[8] = { B11111, B11111, B11111, B11111, B11111, B11111, B11111, B11111 };
byte eyeClosed[8] = { B11111, B11111, B11111, B00000, B00000, B11111, B11111, B11111 };

void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);

  lcd.init();
  lcd.backlight();
  lcd.print("Booting Up...");

  lcd.createChar(0, eyeOpen);
  lcd.createChar(1, eyeClosed);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  servo.attach(servoPin);
  servo.write(servoAngle);

  setMotorSpeeds(0);
  testMotors();

  lcd.clear();
  lcd.print("Robot Ready");
  Serial.println("Setup complete");
  bluetooth.println("Robot Ready");
  delay(1000);
}

void loop() {
  handleBluetoothCommands();

  if (!autonomousMode && !manualOverride && (millis() - lastCommandTime > COMMAND_TIMEOUT)) {
    autonomousMode = true;
    displayAction("Auto Mode");
    Serial.println("Reverting to autonomous mode");
    bluetooth.println("Auto Mode");
  }

  if (autonomousMode && !manualOverride) {
    long distance = getDistance();
    if (distance == -1) distance = OBSTACLE_THRESHOLD + 1;
    int adaptiveSpeed = map(distance, CRITICAL_THRESHOLD, OBSTACLE_THRESHOLD, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
    setMotorSpeeds(constrain(adaptiveSpeed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED));

    if (distance < CRITICAL_THRESHOLD) {
      stopMotors();
      displayAction("Too Close!");
      delay(500);
      smartAvoidance();
    } else if (distance < OBSTACLE_THRESHOLD) {
      stopMotors();
      displayAction("Obstacle!");
      smartAvoidance();
    } else {
      rampMoveForward();
      recordMove('F');
      displayStatus(distance);
    }
  }

  if (showingAction && (millis() - actionTime >= ACTION_DURATION)) {
    showingAction = false;
    lcd.clear();
  } else if (!showingAction) {
    updateEyes();
  }
}

// Motor Control
void setMotorSpeeds(int speed) {
  currentSpeed = constrain(speed, 0, MAX_MOTOR_SPEED);
  motor1.setSpeed(currentSpeed);
  motor2.setSpeed(currentSpeed);
  motor3.setSpeed(currentSpeed);
  motor4.setSpeed(currentSpeed);
}

void rampSpeed(int startSpeed, int endSpeed) {
  int step = (endSpeed > startSpeed) ? 10 : -10;
  for (int speed = startSpeed; speed != endSpeed + step; speed += step) {
    setMotorSpeeds(constrain(speed, 0, MAX_MOTOR_SPEED));
    delay(RAMP_DELAY);
  }
}

void rampMoveForward() {
  stopMotors();
  rampSpeed(0, currentSpeed);
  motor1.run(FORWARD); motor2.run(FORWARD);
  motor3.run(FORWARD); motor4.run(BACKWARD);
  isMoving = true;
}

void rampMoveBackward() {
  stopMotors();
  rampSpeed(0, currentSpeed);
  motor1.run(BACKWARD); motor2.run(BACKWARD);
  motor3.run(BACKWARD); motor4.run(FORWARD);
  isMoving = true;
}

void rampTurnLeft() {
  stopMotors();
  rampSpeed(0, currentSpeed);
  motor1.run(BACKWARD); motor2.run(BACKWARD);
  motor3.run(FORWARD); motor4.run(BACKWARD);
  isMoving = true;
  delay(TURN_DURATION);
  stopMotors();
}

void rampTurnRight() {
  stopMotors();
  rampSpeed(0, currentSpeed);
  motor1.run(FORWARD); motor2.run(FORWARD);
  motor3.run(BACKWARD); motor4.run(FORWARD);
  isMoving = true;
  delay(TURN_DURATION);
  stopMotors();
}

void stopMotors() {
  rampSpeed(currentSpeed, 0);
  motor1.run(RELEASE); motor2.run(RELEASE);
  motor3.run(RELEASE); motor4.run(RELEASE);
  isMoving = false;
}

// Bluetooth Commands
void handleBluetoothCommands() {
  if (bluetooth.available()) {
    char command = bluetooth.read();
    lastCommandTime = millis();
    Serial.print("Cmd: ");
    Serial.println(command);
    bluetooth.print("Echo: ");
    bluetooth.println(command);

    switch (command) {
      case 'F': autonomousMode = false; manualOverride = false; rampMoveForward(); displayAction("Forward"); break;
      case 'B': autonomousMode = false; manualOverride = false; rampMoveBackward(); displayAction("Backward"); break;
      case 'L': autonomousMode = false; manualOverride = false; rampTurnLeft(); displayAction("Left"); break;
      case 'R': autonomousMode = false; manualOverride = false; rampTurnRight(); displayAction("Right"); break;
      case 'S': stopMotors(); manualOverride = true; displayAction("Stopped"); break;
      case 'T': 
        autonomousMode = !autonomousMode; 
        manualOverride = false;
        if (!autonomousMode) stopMotors();
        displayAction(autonomousMode ? "Auto On" : "Auto Off");
        break;
      case '1'...'9': 
        autonomousMode = false; 
        manualOverride = false;
        setMotorSpeeds((command - '0') * MAX_MOTOR_SPEED / 10);
        char speedMsg[16];
        sprintf(speedMsg, "Speed: %d%%", (command - '0') * 10);
        displayAction(speedMsg);
        break;
    }
  }
}

// Autonomous Navigation
void smartAvoidance() {
  long distances[NUM_SCAN_ANGLES];
  for (int i = 0; i < NUM_SCAN_ANGLES; i++) {
    servo.write(SCAN_ANGLES[i]);
    delay(SERVO_DELAY);
    distances[i] = getDistance();
    if (distances[i] == -1) distances[i] = OBSTACLE_THRESHOLD + 1;
    Serial.print("Angle ");
    Serial.print(SCAN_ANGLES[i]);
    Serial.print(": ");
    Serial.println(distances[i]);
    bluetooth.print("A");
    bluetooth.print(SCAN_ANGLES[i]);
    bluetooth.print(":");
    bluetooth.println(distances[i]);
  }
  servo.write(90);
  delay(SERVO_DELAY);

  int bestIndex = 2; // Default to 90°
  long bestDistance = distances[bestIndex];
  for (int i = 0; i < NUM_SCAN_ANGLES; i++) {
    if (distances[i] > bestDistance) {
      bestDistance = distances[i];
      bestIndex = i;
    }
  }

  if (bestDistance < OBSTACLE_THRESHOLD) {
    rampMoveBackward();
    recordMove('B');
    displayAction("Backing Up");
    delay(500);
  } else if (bestIndex == 0 || bestIndex == 1) { // Left
    rampTurnLeft();
    recordMove('L');
    displayAction("Turn Left");
  } else if (bestIndex == 3 || bestIndex == 4) { // Right
    rampTurnRight();
    recordMove('R');
    displayAction("Turn Right");
  } else { // Straight
    rampMoveForward();
    recordMove('F');
    displayAction("Forward");
  }

  if (isStuck()) randomEscape();
}

void recordMove(char action) {
  pathMemory[moveIndex].action = action;
  pathMemory[moveIndex].timestamp = millis();
  moveIndex = (moveIndex + 1) % 5;
}

bool isStuck() {
  int forwardCount = 0, backCount = 0;
  for (int i = 0; i < 5; i++) {
    if (pathMemory[i].action == 'F') forwardCount++;
    if (pathMemory[i].action == 'B') backCount++;
  }
  return (forwardCount > 3 && backCount > 2);
}

void randomEscape() {
  if (random(2) == 0) {
    rampTurnLeft();
    recordMove('L');
    displayAction("Escape Left");
  } else {
    rampTurnRight();
    recordMove('R');
    displayAction("Escape Right");
  }
}

// Sensor and Display
long getDistance() {
  long readings[NUM_READINGS];
  int validCount = 0;
  for (int i = 0; i < NUM_READINGS; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH, 30000);
    readings[i] = (duration > 0) ? (duration * 0.034 / 2) : -1;
    if (readings[i] != -1) validCount++;
    delay(10);
  }
  if (validCount < NUM_READINGS / 2) return -1;
  for (int i = 0; i < NUM_READINGS - 1; i++) {
    for (int j = i + 1; j < NUM_READINGS; j++) {
      if (readings[i] > readings[j]) {
        long temp = readings[i];
        readings[i] = readings[j];
        readings[j] = temp;
      }
    }
  }
  return readings[NUM_READINGS / 2];
}

void displayStatus(long distance) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Dist: ");
  lcd.print(distance);
  lcd.print("cm");
  lcd.setCursor(0, 1);
  lcd.print("Spd: ");
  lcd.print(currentSpeed);
}

void displayAction(const char* message) {
  lcd.clear();
  lcd.print(message);
  showingAction = true;
  actionTime = millis();
}

void updateEyes() {
  if (millis() - lastBlinkTime >= 500) {
    lcd.clear();
    lcd.setCursor(2, 0); lcd.write(eyesOpen ? 0 : 1); lcd.write(eyesOpen ? 0 : 1);
    lcd.setCursor(2, 1); lcd.write(eyesOpen ? 0 : 1); lcd.write(eyesOpen ? 0 : 1);
    lcd.setCursor(12, 0); lcd.write(eyesOpen ? 0 : 1); lcd.write(eyesOpen ? 0 : 1);
    lcd.setCursor(12, 1); lcd.write(eyesOpen ? 0 : 1); lcd.write(eyesOpen ? 0 : 1);
    eyesOpen = !eyesOpen;
    lastBlinkTime = millis();
  }
}

void testMotors() {
  lcd.clear();
  lcd.print("Testing Motors");
  Serial.println("Motor Test Start");
  bluetooth.println("Motor Test Start");
  stopMotors();
  delay(500);

  lcd.clear(); lcd.print("M1 Left Back");
  motor1.setSpeed(MAX_MOTOR_SPEED); motor1.run(FORWARD);
  Serial.println("M1 Forward");
  bluetooth.println("M1 FWD");
  delay(1500);
  motor1.run(RELEASE);

  lcd.clear(); lcd.print("M2 Left Front");
  motor2.setSpeed(MAX_MOTOR_SPEED); motor2.run(FORWARD);
  Serial.println("M2 Forward");
  bluetooth.println("M2 FWD");
  delay(1500);
  motor2.run(RELEASE);

  lcd.clear(); lcd.print("M3 Right Back");
  motor3.setSpeed(MAX_MOTOR_SPEED); motor3.run(FORWARD);
  Serial.println("M3 Forward");
  bluetooth.println("M3 FWD");
  delay(1500);
  motor3.run(RELEASE);

  lcd.clear(); lcd.print("M4 Right Front");
  motor4.setSpeed(MAX_MOTOR_SPEED); motor4.run(BACKWARD);
  Serial.println("M4 Backward");
  bluetooth.println("M4 BWD");
  delay(1500);
  motor4.run(RELEASE);

  lcd.clear(); lcd.print("Test Done");
  Serial.println("Motor Test Complete");
  bluetooth.println("Motor Test Complete");
  delay(500);
}
