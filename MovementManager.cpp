
#include "MovementManager.h"
#include "RobotState.h"
#include <Arduino.h>

MovementManager::MovementManager(int lP, int mP, int rP, int lC, int rC, int eA, int eB, int i1, int i2, int i3, int i4, int e1, int p1) {
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

  Serial.println("MOVEMENT INITIALIZATION COMPLETE");
}

void MovementManager::followLineForwards(int fullSpeed, int turnSpeed, RobotState* rs) {
  /* Behavior for the state "FORWARDS". Entails line following and correction at set speeds (no motor control yet) 
    turnThreshold is the difference in sensor readings we need to determine we are at a line. */

  int turnThreshold = 150;

  // Check if at corner
  if (atCorner()) {
    rs->changeState(RobotState::CORNER);
    return;  
  }

  // Check sensors
  int leftRead = analogRead(leftPin);
  int rightRead = analogRead(rightPin);

  // Turn Right
  if (rightRead - leftRead > turnThreshold) {
    this->motorControl(turnSpeed, 0, true, true);
  }
  // Turn Left
  else if (leftRead - rightRead > turnThreshold) {
    this->motorControl(0, turnSpeed, true, true);
  }
  // Move Forwards over black
  else {
    this->motorControl(fullSpeed, fullSpeed, true, true);
  }
  return;  
}

bool MovementManager::turnRobot(int dir, int outSpeed, int inSpeed, int turnTime) {
  /* Returns true when the robot returns to the line. The while loop format makes it so all other action stops during turns, but this SHOULD be ok. Restructure into millis() if needed 
  
  outSpeed = 50, inSpeed = 55 seems to work, but we rotate around the front axis --> we may detect the same corner twice. That can be ok though, if we add a SKIP command. I'm quickly regretting
  using open loop control here...

  If we do EVEN inSpeed and outSpeed, it appears as if we turn about the breadboard. Can we move this rotation point away? If not, can we back up for a set time such that the breadboard is over the corner.

  Additionally, there's some overshoot everytime we try to stop the motor. Maybe we can code in something to reverse the overshoot? It wouldn't be a huge deal but it's causing issues with the corner detector
  (ie. detecting twice). 

  Also, if we ONLY read the middlePin, we will stop turning early. We want to stop when ALL line followers read black (so we're more aligned). Even still we need a slight delay. Since we don't have
  any sensors to record this position and implementing some sort of SLAM would be too complex, we can use open loop control and tune it to the venue.

  For some reason, the slip steering causes the robot to rotate about the front half of the car. So, we need to do a small static reverse to get that point over the corner before rotating.

  turnAdjust of 250 works for 90 degree turns (290 @ low charge). Larger for smaller angles (need to reverse more). 750 for the low angle first turn. ~550 for the next turn.
  
  */
  
  unsigned int startTime = millis();
  while (millis() - startTime < turnTime) { // Control max turn time
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
  return true;
}

void MovementManager::motorControl(int leftSpeed, int rightSpeed, bool dirLeft, bool dirRight) {
  /* Sends signals to the L298N to control the motor speed and direction. leftSpeed and rightSpeed are percentages of maximum speed. IF both are zero, the motors stop.

  The minimum PWM to overcome static friction is around 70% of 218.
  The minimum PWM to overcome dynamic friction is around 35%, albeit this is probably to low (especially with the weight of the payload).
   
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

  /*
  // Jumpstart to overcome static friction
  else {
    static int sTime = millis();
    static int eTime = 0;
    if (millis() - eTime > 6000) {
      // Serial.println("WRITING HIGH");
      sTime = millis();
      while (millis() - sTime < 500) {
        analogWrite(enableA, convertPWM(70));
        analogWrite(enableB, convertPWM(70)); 
      }
      eTime = millis();
    }
    else {
      // Serial.println("WRITING LOW");
      analogWrite(enableA, convertPWM(leftSpeed));
      analogWrite(enableB, convertPWM(rightSpeed)); 
    }
  }
  */
}

int MovementManager::pingDistance() {
  /* Returns the distance in cm registered by the ultrasonic sensor in question. grabberSensor --> ping the sensor on the grabber. */
  
  // Ping
  digitalWrite(ping1, LOW);
  delayMicroseconds(2);
  digitalWrite(ping1, HIGH);
  delayMicroseconds(10);
  digitalWrite(ping1, LOW);

  // Echo
  int duration = pulseIn(echo1, HIGH);
  return duration / 29 / 2; 
}

bool MovementManager::atCorner() const {
  /* Black Threshold = Number at which our sensors read "Black" instead of "White" */
  static bool lastRead = false;
  // If on the line
  Serial.print(millis());
  Serial.print("   "); 
  Serial.print(digitalRead(leftCorner));
  Serial.print("  ");
  Serial.println(digitalRead(rightCorner));
  if (digitalRead(leftCorner) || digitalRead(rightCorner)) {
    if (!lastRead) { // If just crossed onto a corner
      lastRead = true;
      return true; 
    }
    return false; // If haven't just crossed, return false to not get stuck on corners.
  }
  // If not on the line
  lastRead = false; 
  return false;
  // return true; // For testing
}

int MovementManager::convertPWM(int inputPercent) {
  /* 100% --> 255 * (5/9) = 141.666 ~ 141, 0% --> 0 */
  return floor(inputPercent/100.0 * 255);
}

bool MovementManager::endOfLine() {
  // If all sensors read white, return true. 
  if ((analogRead(leftPin) < blackThreshold) && (analogRead(middlePin) < blackThreshold) && (analogRead(rightPin) < blackThreshold)) {
    return true;  
  }
  return false; 
}

int MovementManager::getMiddleSensor() {
  return analogRead(middlePin);  
}

void MovementManager::updateThreshold(int black, int white) {
  blackThreshold = 0.5 * (black + white);
  Serial.print("NEW THRESHOLD = ");
  Serial.println(blackThreshold);  
}
