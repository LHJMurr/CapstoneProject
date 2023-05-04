#include "StateBehavior.h"
#include "MovementManager.h"
#include "RobotState.h"
#include <Arduino.h>

int corner::commandIdx = 0; // Static integer initialization

void corner::main(int commands[], int numCommands, int turnSpeed, MovementManager* m, RobotState* rs) {
  if (corner::commandIdx >= numCommands) {
    Serial.println("COMMAND INDEX OUT OF BOUNDS");
    rs->changeState(RobotState::DONE);  
  }
  int dir = commands[corner::commandIdx];
  corner::commandIdx++;
  int state = commands[corner::commandIdx];
  corner::commandIdx++;
  // Turn
  m->turnRobot(dir, turnSpeed);
  // New State
  rs->changeState(state); 
}

void forwards::main(int fullSpeed, int turnSpeed, MovementManager* m, RobotState* rs) {
  m->followLineForwards(fullSpeed, turnSpeed, rs);  // Updates to to atCorner() internally
  return;
}

void pickup::main(int adjustSpeed, int turnSpeed, ArmManager* a, MovementManager* m, RobotState* rs) {
  // Raise the arm
  a->raiseArm();
  // Align the juicebox
  pickup::alignJuicebox(adjustSpeed, m);
  // Grab the juicebox
  a->grabJuicebox();
  // Lower the arm
  a->homePosition();
  // Turn around
  m->turnRobot(1, turnSpeed); // While loop
  rs->changeState(RobotState::FORWARDS);
}

void pickup::alignJuicebox(int adjustSpeed, MovementManager* m) {
  /* This function will control the motors to align the grabber with the juicebox using readings from the ultrasonic sensor
     It works by aiming left and right until a minimum distance value is found, noting the presense of the juicebox by large jumps in distance.
     Once aligned, we adjust the car forwards or backwards until the right distance is met. Then we return.

     For now, the function occurs "async" that is, it is called then operates completely. IF we need to do other stuff in the meantime, we can use millis() to desync.
  */   
  unsigned int startTime = millis();
  // int lastDistance = pingDistance(pingPin, echoPin);
  bool angleAligned = false;

  // Align Angle
  while (!angleAligned && millis() - startTime < 100000) { // Break Case is if we look for more than 100 seconds
    static bool leftEdge = false;
    static bool rightEdge = false;
    static int waitTime = 10000;
    // Try to look left
    unsigned int turnStart = millis();
    while(!leftEdge && (millis() - turnStart < waitTime)) {
      pickup::alignAngle(false, &leftEdge, adjustSpeed, m);
      if (leftEdge && rightEdge) {
        Serial.println("FINAL ALIGNMENT RIGHT");
        int turnTime = (millis() - turnStart) / 2;
        int finalAdjustment = millis();
        while (millis() - finalAdjustment < turnTime) {
          m->motorControl(adjustSpeed, adjustSpeed, true, false);  
        }
        Serial.println("ANGULARLY ALIGNED");
        angleAligned = true;
      }  
    }
    waitTime = waitTime + 1000;
    turnStart = millis();
    // Try to look right
    while(!rightEdge && (millis() - turnStart < waitTime)) {
      pickup::alignAngle(true, &rightEdge, adjustSpeed, m);
      if (leftEdge && rightEdge) {
        Serial.println("FINAL ALIGNMENT LEFT");
        int turnTime = (millis() - turnStart) / 2;
        int finalAdjustment = millis();
        while (millis() - finalAdjustment < turnTime) {
          m->motorControl(adjustSpeed, adjustSpeed, false, true);  
        }
        Serial.println("ANGULARLY ALIGNED");
        angleAligned = true;
      }  
    }
    waitTime = waitTime + 1000;
  }

  // Attempt to move linearly
  startTime = millis(); 
  bool linearlyAligned = false;
  while (!linearlyAligned & (millis() - startTime) < 10000) {
    if (m->pingDistance(true) > 10) {
      Serial.println("MOVING FORWARDS"); 
      m->motorControl(adjustSpeed, adjustSpeed, true, true);
    }  
    else if (m->pingDistance(true) < 10) {
      Serial.println("MOVING BACKWARDS");
      m->motorControl(adjustSpeed, adjustSpeed, false, false);  
    }
    else {
      Serial.println("LINEARLY ALIGNED");
      linearlyAligned = true; 
    }
  }
  return;
}

void pickup::alignAngle(bool checkRight, bool* cornerFound, int adjustSpeed, MovementManager* m) {
    static int lastDistance = m->pingDistance(true);
    int newDistance = m->pingDistance(true);
    if (checkRight) {
      Serial.println("TURNING RIGHT");
      m->motorControl(adjustSpeed, adjustSpeed, true, false);
      if (newDistance - lastDistance > 10) { // Distance Epsilon = 10cm. Will need to tune. We're specifically looking for the left EDGE of the box.
        lastDistance = newDistance;
        *cornerFound = true;
        Serial.println("RIGHT EDGE FOUND");
        
      }
    }
    else {
      Serial.println("TURNING LEFT");
      m->motorControl(adjustSpeed, adjustSpeed, false, true);
      if (newDistance - lastDistance > 10) { // Distance Epsilon = 10cm. Will need to tune. We're specifically looking for the left EDGE of the box.
        lastDistance = newDistance;
        *cornerFound = true;
        Serial.println("LEFT EDGE FOUND");
      }
    }
    lastDistance = newDistance;
    return;
}

void dropoff::main(int turnSpeed, ArmManager* a, MovementManager* m, RobotState* rs) {
  // Lower the arm
  a->lowerArm();
  // Release the juicebox
  a->releaseJuicebox();
  // Raise the arm
  a->homePosition();
  // Turn Around
  m->turnRobot(1, turnSpeed);
  rs->changeState(RobotState::FORWARDS);
}

void crawl::main(int crawlSpeed, int turnSpeed, MovementManager* m, RobotState* rs) {
  Serial.println("CRAWLING");
  rs->changeState(RobotState::FORWARDS);  
}

void done::main() {
  Serial.println("DONE");
  Serial.end();
}
