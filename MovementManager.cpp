
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

void MovementManager::followLineForwards(int fullSpeed, int turnSpeed, RobotState* rs) {
  /* Behavior for the state "FORWARDS". Entails line following and correction at set speeds (no motor control yet) 
    turnThreshold is the difference in sensor readings we need to determine we are at a line. */

  /*
  Serial.print("LEFT SENSOR READS: ");
  Serial.print(analogRead(leftPin));
  Serial.print("     MID SENSOR READS: ");
  Serial.print(analogRead(rightPin));
  Serial.print("     RIGHT SENSOR READS: ");
  Serial.println(analogRead(rightPin));
  */

  int turnThreshold = 150;
  static bool leftTurn = false; // Store last speed command
  static bool rightTurn = false;

  // Check if at corner
  if (this->atCorner()) {
    rs->changeState(RobotState::CORNER);
    return;  
  }

  // Check sensors
  int leftRead = analogRead(leftPin);
  int rightRead = analogRead(rightPin); 

  // Turn Right
  if (rightRead - leftRead > turnThreshold) {
    leftTurn = false;
    rightTurn = true;
    this->motorControl(fullSpeed, turnSpeed, true, true);
  }
  // Turn Left
  else if (leftRead - rightRead > turnThreshold) {
    leftTurn = true; 
    rightTurn = false; 
    this->motorControl(turnSpeed, fullSpeed, true, true);
  }
  /*
  // Completely off the line
  else if (leftRead < 200) { // Left is on white but right also reads white
    Serial.print("Repeating Last Command");
    if (!rightTurn && !leftTurn) { // Neither was turning last
      this->motorControl(fullSpeed, fullSpeed, true, true); 
    }
    else if (rightTurn) {
      this->motorControl(fullSpeed, fullSpeed, false, true);  
    }
    else {
      this->motorControl(fullSpeed, fullSpeed, true, false);  
    }
  }
  */
  // Move Forwards over black
  else {
    leftTurn = false; 
    rightTurn = true;
    this->motorControl(fullSpeed, fullSpeed, true, true);
  }
  return;  
}

bool MovementManager::turnRobot(int dir, int outSpeed, int inSpeed, int turnAdjust) {
  /* Returns true when the robot returns to the line. The while loop format makes it so all other action stops during turns, but this SHOULD be ok. Restructure into millis() if needed 
  
  outSpeed = 50, inSpeed = 55 seems to work, but we rotate around the front axis --> we may detect the same corner twice. That can be ok though, if we add a SKIP command. I'm quickly regretting
  using open loop control here...

  If we do EVEN inSpeed and outSpeed, it appears as if we turn about the breadboard. Can we move this rotation point away? If not, can we back up for a set time such that the breadboard is over the corner.

  Additionally, there's some overshoot everytime we try to stop the motor. Maybe we can code in something to reverse the overshoot? It wouldn't be a huge deal but it's causing issues with the corner detector
  (ie. detecting twice). 

  Also, if we ONLY read the middlePin, we will stop turning early. We want to stop when ALL line followers read black (so we're more aligned). Even still we need a slight delay. Since we don't have
  any sensors to record this position and implementing some sort of SLAM would be too complex, we can use open loop control and tune it to the venue.

  For some reason, the slip steering causes the robot to rotate about the front half of the car. So, we need to do a small static reverse to get that point over the corner before rotating.

  turnAdjust of 250 works for 90 degree turns. Larger for smaller angles (need to reverse more). 750 for the low angle first turn. ~550 for the next turn.
  
  */

  int blackThreshold = 300;

  this->motorControl(60, 60, false, false);
  delay(turnAdjust);
  this->motorControl(0, 0, true, true);
  delay(1000);
  
  bool offLine = false;
  unsigned int startTime = millis();
  while (millis() - startTime < 10000) { // Control max turn time
    int midReading = analogRead(middlePin);
    int leftReading = analogRead(leftPin);
    int rightReading = analogRead(rightPin);
    // Check if we have left the original line
    if (!offLine && (midReading < blackThreshold)) {
      offLine = true;  
    }
    // Check to see if we are back on the line
    if (offLine && (midReading >= blackThreshold) && (leftReading >= blackThreshold) && (rightReading >= blackThreshold)) {
      delay(100); // Final alignment
      this->motorControl(0, 0, true, true); // Stop motor before returning
      return true; // Turn Complete
    }
    // Turn Right
    if (dir == 1) {
      this->motorControl(outSpeed, inSpeed, true, false);
    }
    // Turn Left
    else if (dir == -1) {
      this->motorControl(inSpeed, outSpeed, false, true);  
    }
    // No Turn
    else {
      this->motorControl(0, 0, true, true); // Stop motor before returning
      return true;  
    }
  }
  this->motorControl(0, 0, true, true); // Stop motor before returning
  return false;
}

void MovementManager::motorControl(int leftSpeed, int rightSpeed, bool dirLeft, bool dirRight) {
  /* Sends signals to the L298N to control the motor speed and direction. leftSpeed and rightSpeed are percentages of maximum speed. IF both are zero, the motors stop.

  The minimum PWM to get motion is AROUND 60% of 218
  
  */
  
  // Safety Check
  if (leftSpeed > 100) {
    leftSpeed = 100;  
  }
  else if (leftSpeed < 0) {
    leftSpeed = 0;  
  }
  if (rightSpeed > 100) {
    rightSpeed = 100;  
  }
  else if (rightSpeed < 0) {
    rightSpeed = 0;  
  }

  // Left Direction Controls
  if (dirLeft) {
    digitalWrite(input1, HIGH);
    digitalWrite(input2, LOW);
  }
  else {
    digitalWrite(input1, LOW);
    digitalWrite(input2, HIGH);  
  }

  // Right Direction Controls
  if (dirRight) {
    digitalWrite(input3, HIGH);
    digitalWrite(input4, LOW);
  }
  else {
    digitalWrite(input3, LOW);
    digitalWrite(input4, HIGH);  
  }
  
  // Speed Controls
  analogWrite(enableA, convertPWM(leftSpeed));
  analogWrite(enableB, convertPWM(rightSpeed));
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
  /* Black Threshold = Number at which our sensors read "Black" instead of "White" */
  static bool lastRead = false; 
  // int blackThreshold = 300;
  // || ((analogRead(middlePin) < blackThreshold) && (analogRead(leftPin) < blackThreshold) && (analogRead(rightPin) < blackThreshold))
  if (digitalRead(leftCorner) || digitalRead(rightCorner)) {
    if (!lastRead) { // If just crossed onto a corner
      lastRead = true;
      return true; 
    }
    return false; // If haven't just crossed, return false to not get stuck on corners.
  }
  lastRead = false;
  return false;
  // return true; // For testing
}

int MovementManager::convertPWM(int inputPercent) {
  /* 100% --> 255 * (5/9) = 141.666 ~ 141, 0% --> 0 */
  return floor(inputPercent/100.0 * 218);
}
