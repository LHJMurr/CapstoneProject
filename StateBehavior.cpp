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

void pickup::main(int fullSpeed, ArmManager* a, MovementManager* m, RobotState* rs) {
  // Raise the arm
  a->raiseArm();
  
  // Move forwards until distance is reached
  int pingCount = 0;
  while(pingCount < 5) {
    Serial.print(m->pingDistance());
    Serial.print("    ");
    Serial.println(pingCount);
    if (m->pingDistance() < 10) {
      pingCount++;  
    }
    else {
      pingCount = 0;  
    }
    m->followLineForwards(fullSpeed, 0, rs); 
  }
  m->motorControl(0, 0, true, true); // Stop motors
  
  // Grab the juicebox
  a->grabJuicebox();
  
  //Reverse to clear the wall 
  delay(500);
  m->motorControl(fullSpeed, fullSpeed, false, false);
  delay(1000);
  m->motorControl(0, 0, true, true);
  
  // Lower the arm
  a->homePosition(true);
  a->releaseOffset(40, 15); // Offset = 15
  
  // Set state to corner (turn and move forwards)
  rs->changeState(RobotState::CORNER);
}

void dropoff::main(int fullSpeed, ArmManager* a, MovementManager* m, RobotState* rs) {
  // Follow the line until the end is reached
  while(!m->endOfLine()) {
      Serial.println("MovingForwards");
      m->followLineForwards(fullSpeed, 0, rs);
  }
  m->motorControl(0, 0, true, true);

  // Lower Arm
  a->lowerArm(); // Right now it just returns, but if we need more space to turn we can adjust it. 
  
  // Drop the juicebox
  Serial.println("Dropping off juicebox");
  a->releaseJuicebox();

  
  // Set state to corner (turn and move forwards)
  Serial.println("AT 'CORNER'");
  rs->changeState(RobotState::CORNER);
  return;
  
}

void crawl::main(int fullSpeed, int turnSpeed, MovementManager* m, RobotState* rs) {
  // Wait for obstacle to be moved.
  // TO DO
  
  // Full speed through the rough terrain
  m->followLineForwards(fullSpeed, turnSpeed, rs);
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
