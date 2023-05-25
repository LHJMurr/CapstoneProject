#include "RobotState.h"
#include "UserInterface.h"
#include <Arduino.h>

RobotState::RobotState() {
  currentState = 1; // Default to FORWARD (Default to 5 (done) for final product. When done, wait for input.) 
}

void RobotState::changeState(int newState) {
  this->currentState = newState;
}

int RobotState::getState() const {
  return this->currentState;  
}
