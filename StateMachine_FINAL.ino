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
int commandInstructions[8] = {0, 1, 0, 1, -1, 1, 0, 1};
int turnAdjustments[1] = {250}; // Array for the static reverse times @ each turn
MovementManager movement(A0, A1, A2, 28, 29 , 9, 4, 8, 7, 6, 5, 22, 23, 24, 25);
ArmManager arm(13, 12, 11, 10, A5);
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

  StateMachine();
  
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
        corner::main(commandInstructions, turnAdjustments, 8, 70, 70, &movement, &currentState);
        break;
      }
    
    // Forwards State
    case (RobotState::FORWARDS):
      {
        forwards::main(70, 0, &movement, &currentState);
        break;
      }

    // Pickup State
    case RobotState::PICKUP:
      {
        pickup::main(10, 50, 55, &arm, &movement, &currentState);
        break;
      }

     // Dropoff State
    case RobotState::DROPOFF:
      {
        dropoff::main(50, 55, &arm, &movement, &currentState);
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
