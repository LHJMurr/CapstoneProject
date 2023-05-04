// Includes
#include <Arduino.h>
#include "RobotState.h"
#include "MovementManager.h"
#include "ArmManager.h"
#include "StateBehavior.h"

// Function Declarations
void StateMachine();

// Global Variables
int commandInstructions[22] = {0, 1, -1, 1, 0, 2, -1, 1, 0, 1, -1, 4, -1, 1, 1, 1, 0, 3, 1, 0, 0, 5}; // skip, left, pickup, right, skip, crawl, left, right, done. Each two integers is one "corner action". Direction -> state.
MovementManager movement(A0, A1, A2, A3, A4, 9, 8, 7, 6, 5, 4, 22, 23, 24, 25);
ArmManager arm(13, 12, 11, 10);
RobotState currentState;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // Initialize classes
  movement.initialize();
  arm.initialize();
  // Input dice roll here
  Serial.println("Beginning run"); 
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
        corner::main(commandInstructions, 22, 60, &movement, &currentState);
        break;
      }
    
    // Forwards State
    case (RobotState::FORWARDS):
      {
        forwards::main(150, 100, &movement, &currentState);
        break;
      }

    // Pickup State
    case RobotState::PICKUP:
      {
        pickup::main(10, 60, &arm, &movement, &currentState);
        break;
      }

     // Dropoff State
    case RobotState::DROPOFF:
      {
        dropoff::main(60, &arm, &movement, &currentState);
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
        done::main();
        break;
      }
  }
}
