#include "StateBehavior.h"
#include "MovementManager.h"
#include "RobotState.h"
#include "UserInterface.h"
#include <Arduino.h>

int corner::commandIdx = 0; // Static integer initialization

void corner::main(int commands[], int turnTimings[], int numCommands, int outSpeed, int inSpeed, MovementManager* m, RobotState* rs) {
  
  m->motorControl(0, 0, true, true); // stop the motors at the corner
  delay(500); // Small delay to show corner is detected

  if (corner::commandIdx >= numCommands) {
    Serial.println("COMMAND INDEX OUT OF BOUNDS");
    rs->changeState(RobotState::DONE);  
  }
  int dir = commands[corner::commandIdx];
  corner::commandIdx++;
  int state = commands[corner::commandIdx];
  corner::commandIdx++;
  
  // Turn
  static int turnIndex = 0;
  int turnWait;
  if (abs(dir) == 1) {
      turnWait = turnTimings[turnIndex];
      turnIndex++; 
  }
  else {
    turnWait = 0;  
  }
  m->turnRobot(dir, outSpeed, inSpeed, turnWait);
  // New State
  rs->changeState(state); 
}

void forwards::main(int fullSpeed, int turnSpeed, MovementManager* m, RobotState* rs) {
  m->followLineForwards(fullSpeed, turnSpeed, rs);  // Updates to to atCorner() internally
  return;
}

void pickup::main(int adjustSpeed, int outSpeed, int inSpeed, ArmManager* a, MovementManager* m, RobotState* rs) {
  /*
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
  */
  Serial.println("PICKING UP");
  m->turnRobot(1, outSpeed, inSpeed, 0); // While loop
  rs->changeState(RobotState::FORWARDS);
}

void pickup::alignJuicebox(int adjustSpeed, MovementManager* m) {
  /* This function will control the motors to align the grabber with the juicebox using readings from the ultrasonic sensor
     It works by aiming left and right until a minimum distance value is found, noting the presense of the juicebox by large jumps in distance.
     Once aligned, we adjust the car forwards or backwards until the right distance is met. Then we return.

     For now, the function occurs "async" that is, it is called then operates completely. IF we need to do other stuff in the meantime, we can use millis() to desync.
  */ 

  int distanceEpsilon = 10;
  int targetDistance = 10;
  
  unsigned int startTime = millis();
  bool angleAligned = false;
  // Align Angle
  while (!angleAligned && millis() - startTime < 10000) { // Break Case is if we look for more than 10 seconds
    static bool leftEdge = false;
    static bool rightEdge = false;
    static int waitTime = 1000;
    // Try to look left
    unsigned int turnStart = millis();
    while(!leftEdge && (millis() - turnStart < waitTime)) {
      pickup::alignAngle(false, &leftEdge, adjustSpeed, m, distanceEpsilon);
      if (leftEdge && rightEdge) {
        int turnTime = (millis() - turnStart) / 2;
        int finalAdjustment = millis();
        while (millis() - finalAdjustment < turnTime) {
          m->motorControl(adjustSpeed, adjustSpeed, true, false);  
        }
        angleAligned = true;
      }  
    }
    waitTime = waitTime + 1000;
    turnStart = millis();
    // Try to look right
    while(!rightEdge && (millis() - turnStart < waitTime)) {
      pickup::alignAngle(true, &rightEdge, adjustSpeed, m, distanceEpsilon);
      if (leftEdge && rightEdge) {
        int turnTime = (millis() - turnStart) / 2;
        int finalAdjustment = millis();
        while (millis() - finalAdjustment < turnTime) {
          m->motorControl(adjustSpeed, adjustSpeed, false, true);  
        }
        angleAligned = true;
      }  
    }
    waitTime = waitTime + 1000;
  }

  // Attempt to move linearly
  startTime = millis(); 
  bool linearlyAligned = false;
  while (!linearlyAligned & (millis() - startTime) < 10000) {
    if (m->pingDistance(true) > targetDistance) { 
      m->motorControl(adjustSpeed, adjustSpeed, true, true);
    }  
    else if (m->pingDistance(true) < targetDistance) {
      m->motorControl(adjustSpeed, adjustSpeed, false, false);  
    }
    else {
      linearlyAligned = true; 
    }
  }
  return;
}

void pickup::alignAngle(bool checkRight, bool* cornerFound, int adjustSpeed, MovementManager* m, int distanceEpsilon) {
  static int lastDistance = m->pingDistance(true);
  int newDistance = m->pingDistance(true);
  if (checkRight) {
    m->motorControl(adjustSpeed, adjustSpeed, true, false);
    if (newDistance - lastDistance > distanceEpsilon) { // Distance Epsilon = 10cm. Will need to tune. We're specifically looking for the left EDGE of the box.
      lastDistance = newDistance;
      *cornerFound = true;
    }
  }
  else {
    m->motorControl(adjustSpeed, adjustSpeed, false, true);
    if (newDistance - lastDistance > distanceEpsilon) { // Distance Epsilon = 10cm. Will need to tune. We're specifically looking for the left EDGE of the box.
      lastDistance = newDistance;
      *cornerFound = true;
    }
  }
  lastDistance = newDistance;
  return;
}

void dropoff::main(int outSpeed, int inSpeed, ArmManager* a, MovementManager* m, RobotState* rs) {
  /*
  // Lower the arm
  a->lowerArm();
  // Release the juicebox
  a->releaseJuicebox();
  // Raise the arm
  a->homePosition();
  // Turn Around
  m->turnRobot(1, turnSpeed);
  */
  Serial.println("DROPPING OFF"); // For testing
  m->turnRobot(1, outSpeed, inSpeed, 0); // While loop
  rs->changeState(RobotState::FORWARDS);
}

void crawl::main(int crawlSpeed, int turnSpeed, MovementManager* m, RobotState* rs) {

  /*
  int safetyDistance = 10;
  
  m->followLineForwards(crawlSpeed, turnSpeed, rs);
  if (m->pingDistance(false) < safetyDistance) {
    rs->changeState(RobotState::FORWARDS); 
  }  
  */
  Serial.println("CRAWLING"); // For testing
  rs->changeState(RobotState::FORWARDS);
  return;
}

void done::main(InterfaceManager* ui, RobotState* rs) {
  /*
  int numPressed = 0;
  int timePressed = millis();
  ui->blinkDoneLight(true, 0); // Turn on DONE light
  while (true) { // This is the idle state. We stay here indefinitely until a break case is met
    if (ui->wasPressed()) {
        numPressed = numPressed + 1;
        timePressed = millis();        
    }
    if (numPressed > 0 && (millis() - timePressed > 1000)) {
      Serial.print("CHOSEN COMMAND STRING: ");
      Serial.println(numPressed);
      Serial.println("BEGINNING RUN");
      ui->blinkDoneLight(false, numPressed);
      rs->changeState(RobotState::FORWARDS);
      break;
    }
  }
  return;
  */

  Serial.println("IDLE STATE");
  Serial.end();
  
}
