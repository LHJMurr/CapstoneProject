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

  m->motorControl(70, 70, false, false);
  delay(turnWait);
  m->motorControl(0, 0, true, true);
  delay(1000);
  
  m->turnRobot(dir, outSpeed, inSpeed);
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
  delay(1000);

  // Move forwards until distance is reached
  int pingCount = 0;
  while(pingCount < 5) {
    Serial.print(m->pingDistance());
    Serial.print("    ");
    Serial.println(pingCount);
    if (m->pingDistance() < 12) {
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
  delay(1000);
  
  //Reverse until at corner
  while(!m->atCorner()) {
     m->motorControl(fullSpeed, fullSpeed, false, false); 
  }
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
  while(m->pingDistance() < 10) {Serial.println("OBSTACLE DETECTED");}
  
  // Full speed through the rough terrain
  m->followLineForwards(fullSpeed, turnSpeed, rs);
  return;
}

void done::main(InterfaceManager* ui, MovementManager* m, RobotState* rs, Instructions* inst) {
  digitalWrite(ui->stateLight, HIGH);
  ui->blinkDoneLight(false, 0); // Turn OFF done light
  while (true) { // This is the idle state. We stay here indefinitely until a break case is met
    // CALIBRATE
    int blackVal = 600;
    while (true) {
      if (ui->wasPressed()) {
        blackVal =  m->getMiddleSensor();
        Serial.print("Black Value is: ");
        Serial.println(blackVal);
        ui->blinkDoneLight(false, 1); 
        break;        
      }
    }
    int whiteVal = 0;
    while (true) {
      if (ui->wasPressed()) {
        whiteVal =  m->getMiddleSensor();
        Serial.print("White Value is: ");
        Serial.println(whiteVal);
        ui->blinkDoneLight(false, 1);       
        break;  
      }
    }
    m->updateThreshold(blackVal, whiteVal);
    
    // GET ROLL 1
    int roll1 = 0;
    int timePressed;
    while (true) {
      if (ui->wasPressed()) {
          roll1++;
          timePressed = millis();        
      }
      if (roll1 > 0 && (millis() - timePressed > 1000)) {
        Serial.print("ROLL 1: ");
        Serial.println(roll1);
        ui->blinkDoneLight(false, roll1);
        break;
      }
    }
    
    // GET ROLL 2
    int roll2 = 0;
    while (true) {
      if (ui->wasPressed()) {
          roll2++;
          timePressed = millis();        
      }
      if (roll2 > 0 && (millis() - timePressed > 1000)) {
        Serial.print("ROLL 2: ");
        Serial.println(roll2);
        ui->blinkDoneLight(false, roll2);
        break;
      }
    }
    
    // SET COMMAND STRING
    inst->setCommands(roll1, roll2); // CHANGE corner.main to call FROM instructions!
    rs->changeState(RobotState::FORWARDS);
    break;
  }
  // ui->blinkDoneLight(false, 0);
  digitalWrite(ui->stateLight, LOW);
  return; 
}
