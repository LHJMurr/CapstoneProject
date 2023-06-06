#include "UserInterface.h"
#include <Arduino.h>

InterfaceManager::InterfaceManager(int dL, int sL, int iB, int bD) {
  doneLight = dL;
  interfaceButton = iB;
  blinkDelay = bD;
  stateLight = sL; 
}

void InterfaceManager::initialize() {
  Serial.println("INITIALIZING INTERFACE MANAGER");
  
  // Establish Pin Modes
  pinMode(doneLight, OUTPUT);
  pinMode(interfaceButton, INPUT);
  pinMode(stateLight, OUTPUT);

  Serial.println("INTERFACE MANAGER INITIALIZATION COMPLETE");
}

void InterfaceManager::blinkDoneLight(bool state, int numTimes) {
  // Turn on/off the light
  digitalWrite(doneLight, state);
  // Blink numTimes times
  for (int i = 0; i < numTimes; i++) {
    delay(blinkDelay);
    digitalWrite(doneLight, !digitalRead(doneLight));
    delay(blinkDelay);
    digitalWrite(doneLight, !digitalRead(doneLight));
  }
  return;
}

bool InterfaceManager::wasPressed() {
  static bool lastRead = false;
  if (digitalRead(interfaceButton)) {
    if (lastRead == false) {
      lastRead = true;
      return lastRead; 
    }
    lastRead = true;
    return false;
  }
  lastRead = false;
  return false;
}

Instructions::Instructions() {
  commandInstructions[0] = 0;
  commandInstructions[1] = 5; // Default to idle state
}

void Instructions::setCommands(int roll_1_result, int roll_2_result) {
  int cidx = 0;
  for (int i = 0; i < numCommandsRoll_1; i++) {
    commandInstructions[cidx] = roll_1[roll_1_result - 1][i];
    cidx++;
  }  
  for (int i = 0; i < numCommandsRoll_1; i++) {
    commandInstructions[cidx] = roll_2[roll_2_result - 1][i];
    cidx++;
  } 
}

int Instructions::getNumCommands() const {
  return numCommandsRoll_1 + numCommandsRoll_2;
}
