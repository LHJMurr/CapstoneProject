#ifndef ARM_MANAGER_H
#define ARM_MANAGER_H

#include <Servo.h>

class ArmManager {
  public:
    ArmManager(int s1, int s2, int s3, int grabber);
    void initialize();
    void raiseArm();
    void lowerArm();
    void homePosition();
  private:
    // Private Functions
    bool moveArm(int targetTheta, int targetAlpha, int restTime);
    void moveServo(int targetAngle, int restTime, Servo s);

    // Private Members
    Servo jointOne;
    Servo jointTwo;
    Servo jointThree;
    Servo jointGrabber;
    int jointOneInitialAngle = 30;
    int jointTwoInitialAngle = 0;
    int jointThreeInitialAngle = 90;
    int s1Pin;
    int s2Pin;
    int s3Pin;
    int grabberPin;
};

#endif
