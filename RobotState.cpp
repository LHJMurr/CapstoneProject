#include "RobotState.h"
#include <Arduino.h>

RobotState::RobotState() {
  currentState = 1; // Default to forwards  
}

void RobotState::changeState(int newState) {
  this->currentState = newState;
}

int RobotState::getState() const {
  return this->currentState;  
}
