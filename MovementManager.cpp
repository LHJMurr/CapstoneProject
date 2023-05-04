#include "MovementManager.h"
#include "RobotState.h"
#include <Arduino.h>

MovementManager::MovementManager(int lP, int mP, int rP, int lC, int rC, int eA, int eB, int i1, int i2, int i3, int i4, int e1, int p1, int e2, int p2) {
  // Establish Sensor Pin Connections
  leftPin = lP;
  middlePin = mP;
  rightPin = rP;
  leftCorner = lC;
  rightCorner = rC;
  // Establish Motor Pin Connections
  enableA = eA;
  enableB = eB;
  input1 = i1;
  input2 = i2;
  input3 = i3;
  input4 = i4;
  // Establish Ultrasonic Sensor Connections
  echo1 = e1;
  ping1 = p1;
  echo2 = e2;
  ping2 = p2;
}

void MovementManager::initialize() const {
  Serial.println("BEGINNING MOVEMENT INITIALIZATION");
  
  // Establish Pin Modes
  pinMode(leftPin, INPUT);
  pinMode(middlePin, INPUT);
  pinMode(rightPin, INPUT);
  pinMode(leftCorner, INPUT);
  pinMode(rightCorner, INPUT);
  pinMode(enableA, OUTPUT);
  pinMode(enableB, OUTPUT);
  pinMode(input1, OUTPUT);
  pinMode(input2, OUTPUT);
  pinMode(input3, OUTPUT);
  pinMode(input4, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(ping1, OUTPUT);
  pinMode(echo2, INPUT);
  pinMode(ping2, OUTPUT);

  Serial.println("MOVEMENT INITIALIZATION COMPLETE");
}

void MovementManager::followLineForwards(int fullSpeed, int turnSpeed, RobotState* rs) const {
  /* Behavior for the state "FORWARDS". Entails line following and correction at set speeds (no motor control yet) */
  // Check if at corner
  if (MovementManager::atCorner()) {
    rs->changeState(RobotState::CORNER);
    return;  
  }

  // Check for motor action
  int leftRead = analogRead(leftPin);
  int rightRead = analogRead(rightPin);
  int turnThreshold = 150; // Lower means less difference will still cause turning. Needs tuning to venue/environment/sensors.
  // Turn Right
  if (rightRead - leftRead > turnThreshold) {
    this->motorControl(fullSpeed, turnSpeed, true, true);
  }
  // Turn Left
  else if (leftRead - rightRead > turnThreshold) {
    this->motorControl(turnSpeed, fullSpeed, true, true);
  }
  // Move Forwards
  else {
    this->motorControl(fullSpeed, fullSpeed, true, true);
  }
  return;  
}

bool MovementManager::turnRobot(int dir, int turnSpeed) const {
  /* Returns true when the robot returns to the line. The while loop format makes it so all other action stops during turns, but this SHOULD be ok. Restructure into millis() if needed */
  int blackThreshold = 400;
  bool offLine = false;
  unsigned int startTime = millis();
  while (millis() - startTime < 1) { // Control max turn time
    int irReading = analogRead(middlePin);
    // Check if we have left the original line
    if (!offLine && (irReading < blackThreshold)) {
      offLine = true;  
    }
    // Check to see if we are back on the line
    if (offLine && (irReading >= blackThreshold)) {
        return true; // Turn Complete
    }
    // Turn Right
    if (dir == 1) {
      this->motorControl(turnSpeed, turnSpeed, true, false);
    }
    // Turn Left
    else if (dir == -1) {
      this->motorControl(turnSpeed, turnSpeed, false, true);  
    }
    // No Turn
    else {
      return true;  
    }
  }
  return false;
}

void MovementManager::motorControl(int leftSpeed, int rightSpeed, bool dirLeft, bool dirRight) const {
  /* Sends signals to the L298N to control the motor speed and direction. leftSpeed and rightSpeed are PWM values from 0 to 255. IF both are zero, the motors stop. */
  // Speed Controls
  analogWrite(enableA, leftSpeed);
  analogWrite(enableB, rightSpeed);

  // Left Direction Controls
  if (dirLeft) {
    digitalWrite(input1, LOW);
    digitalWrite(input2, HIGH);
  }
  else {
    digitalWrite(input1, HIGH);
    digitalWrite(input2, LOW);  
  }

  // Right Direction Controls
  if (dirRight) {
    digitalWrite(input3, LOW);
    digitalWrite(input4, HIGH);
  }
  else {
    digitalWrite(input3, HIGH);
    digitalWrite(input4, LOW);  
  }
}

int MovementManager::pingDistance(bool grabberSensor) {
  /* Returns the distance in cm registered by the ultrasonic sensor in question. grabberSensor --> ping the sensor on the grabber. */
  int pingPin;
  int echoPin;
  if (grabberSensor) {
    pingPin = ping1;
    echoPin = echo1;
  }
  else {
    pingPin = ping2;
    echoPin = echo2;  
  }
  
  // Ping
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin, LOW);

  // Echo
  int duration = pulseIn(echoPin, HIGH);
  return duration / 29 / 2; 
}

bool MovementManager::atCorner() const {
  /*  
  blackThreshold = 100; // The threshold for the sensors to detect a black line. Will require tuning to be the right specificity.
  if ((analogRead(leftCorner) < blackThreshold) || (analogRead(rightCorner) < blackThreshold)) { // May want to change to checking some distance, if getting back on the line is problematic.
    return true;  
  }
  else if ((analogRead(left) > blackThreshold) && (analogRead(right) > blackThreshold) && (analogRead(middle) > blackThreshold) {
    return true;  
  }
  return false; 
  */
  return true; // For the sake of the state machine, delete when testing/tuning
}
