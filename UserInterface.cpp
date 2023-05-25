#include "UserInterface.h"
#include <Arduino.h>

InterfaceManager::InterfaceManager(int dL, int iB, int bD) {
  doneLight = dL;
  interfaceButton = iB;
  blinkDelay = bD;
}

void InterfaceManager::initialize() {
  Serial.println("INITIALIZING INTERFACE MANAGER");
  
  // Establish Pin Modes
  pinMode(doneLight, OUTPUT);
  pinMode(interfaceButton, INPUT);

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
