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

    There aren't a lot of safety features in the code/robot. We don't default to a done state and it's possible to get all sorts of strange inputs. I don't have time to fix them, so we have to make sure
    we operate the robot in a standard way
  
*/

// Function Declarations
void StateMachine();

// Global Variables
int commandInstructions[10] = {0, 1, -1, 2, 1, 1, 0, 1, 0, 5};
int turnAdjustments[2] = {75, 150}; // Array for the static reverse times @ each turn. 70 speed @ 235 max PWM. 250 for 90 degrees, 710 for large-angle. 525 for the small angle. 350-ish for 90 degree afte crawl.
MovementManager movement(A0, A1, A2, 23, 13 , 4, 9, 5, 6, 7, 8, 47, 39);
ArmManager arm(11, 2, 3, 12, A5);
InterfaceManager ui(52, 53, 35, 1000);
Instructions instructions; // manages the instructions given by a button press. TO DO.
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
        // corner::main(instructions.commandInstructions, turnAdjustments, 10, 100, 100, &movement, &currentState);
        corner::main(commandInstructions, turnAdjustments, 10, 100, 100, &movement, &currentState);
        break;
      }
    
    // Forwards State
    case (RobotState::FORWARDS):
      {
        forwards::main(60, 100, &movement, &currentState);
        break;
      }

    // Pickup State
    case RobotState::PICKUP:
      {
        pickup::main(70, &arm, &movement, &currentState);
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
        done::main(&ui, &movement, &currentState, &instructions);
        break;
      }
  }
}
