#ifndef MOVEMENT_MANAGER_H
#define MOVEMENT_MANAGER_H

#include "RobotState.h"

// General Classes. Store static functions for use in state behavior and help facilitate the initialization.
class MovementManager {
  public:
    void followLineForwards(int fullSpeed, int turnSpeed, RobotState* rs) const;
    bool turnRobot(int dir, int turnSpeed) const;
    MovementManager(int lP, int mP, int rP, int lC, int rC, int eA, int eB, int i1, int i2, int i3, int i4);
    void initialize() const;
    void motorControl(int leftSpeed, int rightSpeed, bool dirLeft, bool dirRight) const;
  private:
    // Private Functions
    bool atCorner() const;

    // Movement Sensor Pins
    int leftPin;
    int middlePin;
    int rightPin;
    int leftCorner;
    int rightCorner;

    // Motor pins
    int enableA;
    int enableB;
    int input1;
    int input2;
    int input3;
    int input4;
};

#endif
