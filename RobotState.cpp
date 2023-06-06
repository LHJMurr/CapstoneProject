#include "RobotState.h"
#include "UserInterface.h"
#include <Arduino.h>

RobotState::RobotState() {
  currentState = 1; // Default to FORWARDS
}

void RobotState::changeState(int newState) {
  this->currentState = newState;
}

int RobotState::getState() const {
  return this->currentState;  
}
