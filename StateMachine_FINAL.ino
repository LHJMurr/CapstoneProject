// Includes
#include <Arduino.h>
#include "RobotState.h"
#include "MovementManager.h"
#include "ArmManager.h"
#include "StateBehavior.h"
#include "UserInterface.h"

/* 
GENERAL NOTES:
  MOVEMENT:
    There may be issues with detecting the corner after the rough terrain. This is because we have to move fullspeed to pass the terrain and may miss the corner detection.
      Potential Solutions: Move slower. 100% speed may be faster than needed.
      Use open loop control and just time the stop. Kind of a rough idea, but may work.
      Make some speed function that STARTS high and SLOWS after a certain amount of time.
      
    We don't have a ton of sensors for feedback, so a LOT of our model is Open-Loop. So its REALLY important to get the timings of the delays right, especially the turnAdjustments array.
    Good turnAdjustments --> accurate turns --> less variance in runs.

    Full charge and half charge result in different voltages. Tuning should be done around FULL CHARGE so we have a standard value.

    In MovementManager.cpp, there's a function called ConvertPWM. In that function, we give a percentage and it returns the PWM signal to the motors. IF we need more power, change that 235 to something 
    UP TO 255. Note though that this is more voltage than out motors should recieve, so do this sparingly and only if needed.

    We still need to tune obstacle detection in the StateBehavior::crawl::main() function. Just make it something like "if ultrasonic sensor reads > SAFE_VAL, go. Else, wait."
  ARM:
    We may want to up the arm servo voltages to 6V, but 5V seems to suffice. It just looks really sketchy.

    We need to tune the ultrasonic sensor readings. The dropoff isn't too bad but the pickup needs work.
  
*/

// Function Declarations
void StateMachine();

// Global Variables
int commandInstructions[12] = {0, 1, 0, 1, 0, 1, -1, 1, -1, 4, -1, 1};
int turnAdjustments[3] = {710, 525, 350}; // Array for the static reverse times @ each turn. 70 speed @ 235 max PWM. 300 for 90 degrees, 710 for large-angle. 525 for the small angle. 350-ish for 90 degree afte crawl.
MovementManager movement(A0, A1, A2, 23, 13 , 4, 9, 5, 6, 7, 8, 53, 39);
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

  arm.raiseArm();
  delay(2000);
  arm.homePosition(true);
  arm.releaseOffset(40, 15);
  delay(2000); 

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
        corner::main(commandInstructions, turnAdjustments, 4, 100, 100, &movement, &currentState);
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
        pickup::main(40, &arm, &movement, &currentState);
        break;
      }

     // Dropoff State
    case RobotState::DROPOFF:
      {
        dropoff::main(40, &arm, &movement, &currentState);
        break;
      } 

    // Crawl State
    case RobotState::CRAWL:
      {
        crawl::main(100, 0, &movement, &currentState);
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
