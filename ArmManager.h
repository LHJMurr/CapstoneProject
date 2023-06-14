#ifndef ARM_MANAGER_H
#define ARM_MANAGER_H

#include <Servo.h>

class ArmManager {
  public:
    ArmManager(int s1, int s2, int s3, int grabber, int ps);
    void initialize();
    void raiseArm();
    void lowerArm();
    void homePosition(bool offset);
    void grabJuicebox();
    void releaseJuicebox();
    bool moveArm(int targetTheta, int targetPhi, int restTime, int offset); 
    void releaseOffset(int restTime, int offset); 
      Servo jointOne;
    Servo jointTwo;
    Servo jointThree;
    Servo jointGrabber; // 0 is open, 180 is closed. May need to scale up to 7V if too weak to grab.
  private:
    
    // Private Functions
    static int translateAngle(int desiredAngle, bool toWrite);
    bool moveServo(int targetAngle, int restTime, Servo s); 

    // Private Members
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
