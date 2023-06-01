// Includes
#include <Arduino.h>
#include "RobotState.h"
#include "MovementManager.h"
#include "ArmManager.h"
#include "StateBehavior.h"
#include "UserInterface.h"

// Function Declarations
void StateMachine();

// Global Variables
int commandInstructions[22] = {0, 1, 0, 1, 0, 1, -1, 1, -1, 1, -1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1};
int turnAdjustments[4] = {770, 570, 290, 290}; // Array for the static reverse times @ each turn
MovementManager movement(A5, A1, A7, 23, 13 , 4, 9, 5, 6, 7, 8, 53, 39);
ArmManager arm(11, 2, 3, 12, A5);
InterfaceManager ui(52, 51, 1000);
RobotState currentState;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // Initialize classes
  movement.initialize();
  arm.initialize();
  ui.initialize();
}

void loop() {
  // put your main code here, to run repeatedly:

  /* General Notes:
    It may be that because lower charge -> lower voltage -> less force, the turnAdjustments may be a function of charge. Make sure to test on FULL charge before the final presentation.
  */

  Serial.println(movement.pingDistance());

  /*
  pickup::main(61, 0, 80, &arm, &movement, &currentState); // Third value is the speed to turn around
  delay(20000);
  arm.releaseJuicebox();
  delay(20000);
  */

  // StateMachine();
  
}

void StateMachine() {

  /* This function controls the decision logic of the robot and is called every loop().
     If we need to run multiple functions in "parallel", consider using the millis() function to do so every t seconds.
     https://www.youtube.com/watch?v=TH__MdQD_GA&t=123s (A resource I utilized when making this)
  */
  
  switch (currentState.getState()) {

    // CornerState
    case RobotState::CORNER:
      {
        corner::main(commandInstructions, turnAdjustments, 22, 70, 70, &movement, &currentState);
        break;
      }
    
    // Forwards State
    case (RobotState::FORWARDS):
      {
        forwards::main(90, 0, &movement, &currentState);
        break;
      }

    // Pickup State
    case RobotState::PICKUP:
      {
        pickup::main(40, 70, &arm, &movement, &currentState);
        break;
      }

     // Dropoff State
    case RobotState::DROPOFF:
      {
        dropoff::main(40, 70, &arm, &movement, &currentState);
        break;
      } 

    // Crawl State
    case RobotState::CRAWL:
      {
        crawl::main(60, 30, &movement, &currentState);
        break;
      }

    // Done State
    case RobotState::DONE:
      {
        done::main(&ui, &currentState);
        break;
      }
  }
}
