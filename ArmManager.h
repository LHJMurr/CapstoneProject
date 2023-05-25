#ifndef ARM_MANAGER_H
#define ARM_MANAGER_H

#include <Servo.h>

class ArmManager {
  public:
    ArmManager(int s1, int s2, int s3, int grabber, int ps);
    void initialize();
    void raiseArm();
    void lowerArm();
    void homePosition();
    void grabJuicebox();
    void releaseJuicebox();
    bool moveArm(int targetTheta, int targetPhi, int restTime);
  private:
    // Private Functions
    void moveServo(int targetAngle, int restTime, Servo s);
    static int translateAngle(int desiredAngle, bool toWrite);

    // Private Members
    Servo jointOne;
    Servo jointTwo;
    Servo jointThree;
    Servo jointGrabber;
    int jointOneInitialAngle = 22; // The initial angles are VERY important as they calibrate the arm balancing logic. Each is measured with respect to the last link
    int jointTwoInitialAngle = 158; // by right-hand rule convention
    int jointThreeInitialAngle = -90; 
    int s1Pin;
    int s2Pin;
    int s3Pin;
    int grabberPin;

    int pressureSensor;
};

#endif
