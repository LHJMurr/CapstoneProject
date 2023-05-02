// Includes
#include <Arduino.h>
#include "RobotState.h"
#include "MovementManager.h"
#include "ArmManager.h"

// Function Declarations
void StateMachine();

// Global Variables
int commandInstructions[18] = {0, 1, -1, 1, 0, 2, 1, 1, 0, 1, -1, 3, -1, 1, 1, 1, 0, 5}; // skip, left, pickup, right, skip, crawl, left, right, done. Each two integers is one "corner action". Direction -> state.
MovementManager movement(A0, A1, A2, A3, A4, 9, 8, 7, 6, 5, 4);
ArmManager arm(13, 12, 11, 10);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // Initialize classes
  movement.initialize();
  arm.initialize();
  // Input dice roll here
  Serial.println("Beginning run. Moving forwards..."); 
}

void loop() {
  // put your main code here, to run repeatedly:
  StateMachine();
}

void StateMachine() {

  /* This function controls the decision logic of the robot and is called every loop().
     If we need to run multiple functions in "parallel", consider using the millis() function to do so every t seconds.
     https://www.youtube.com/watch?v=TH__MdQD_GA&t=123s (A resource I utilized when making this)
  */

  // Track the current command index
  static int commandIndex = 0;
  // Track the current state
  static RobotState currentState;
  
  switch (currentState.getState()) {

    // CornerState
    case RobotState::CORNER:
      {
        Serial.println("AT CORNER");
        int dir = commandInstructions[commandIndex];
        commandIndex++;
        int state = commandInstructions[commandIndex];
        commandIndex++;
        // Turn
        movement.turnRobot(dir, 90);
        // New State
        currentState.changeState(state);
        break;
      }
    
    // Forwards State
    case (RobotState::FORWARDS):
      {
        movement.followLineForwards(100, 100, &currentState); // checks atCorner() internally, although we may want another break case.
        break;
      }

    // Pickup State
    case RobotState::PICKUP:
      {
        arm.raiseArm();
        Serial.println("Juicebox Grabbed");
        arm.homePosition();
        Serial.println("Arm Retracted");
        movement.turnRobot(1, 90); // Turn 180 degrees then change state
        currentState.changeState(RobotState::FORWARDS);
        break;
      }

     // Dropoff State
    case RobotState::DROPOFF:
      {
        arm.lowerArm();
        Serial.println("Juicebox Released");
        arm.homePosition();
        Serial.println("Arm Retracted");
        movement.turnRobot(1, 90); // Turn 180 degrees then change state
        currentState.changeState(RobotState::FORWARDS);
        break;
      } 

    // Crawl State
    case RobotState::CRAWL:
      {
        movement.followLineForwards(50, 50, &currentState);
        if (currentState.getState() != RobotState::CRAWL) { // IF we switch to a corner state, break. Also, we may want to switch the order of logic here.
          break;  
        }
        // if not, if its safe to pass the obstacle, pick up the speed. If not, keep crawling.
        Serial.println("Obstacle Detected");
        Serial.println("Safety Determined");
        currentState.changeState(RobotState::FORWARDS);
        break;
      }

    // Done State
    case RobotState::DONE:
      {
        Serial.println("Finished!");
        Serial.end();
        break;
      }
  }
}
