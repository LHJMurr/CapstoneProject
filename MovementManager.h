#ifndef MOVEMENT_MANAGER_H
#define MOVEMENT_MANAGER_H

#include "RobotState.h"

// General Classes. Store static functions for use in state behavior and help facilitate the initialization.
class MovementManager {
  public:
    MovementManager(int lP, int mP, int rP, int lC, int rC, int eA, int eB, int i1, int i2, int i3, int i4, int e1, int p1);
    void initialize() const;
    void followLineForwards(int fullSpeed, int turnSpeed, RobotState* rs); // *
    bool turnRobot(int dir, int outSpeed, int inSpeed); // *
    void motorControl(int leftSpeed, int rightSpeed, bool dirLeft, bool dirRight);
    int pingDistance();
    bool atCorner() const;
    bool endOfLine();
    int getMiddleSensor();
    void updateThreshold(int black, int white);
  private:
    // Private Functions
    int convertPWM(int inputPercent); // * 

    // Movement Sensor Pins
    int leftPin;
    int middlePin;
    int rightPin;
    int leftCorner;
    int rightCorner;
    
    int blackThreshold = 300;

    // Motor pins
    int enableA;
    int enableB;
    int input1;
    int input2;
    int input3;
    int input4;

    // Ultrasonic Sensor Pins
    int echo1;
    int ping1;
};

#endif
