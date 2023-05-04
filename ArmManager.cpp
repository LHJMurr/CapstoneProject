#include "ArmManager.h"
#include <Servo.h>
#include <Arduino.h>

ArmManager::ArmManager(int s1, int s2, int s3, int grabber) {
  // Establish Servo Pin Numbers
  s1Pin = s1;
  s2Pin = s2;
  s3Pin = s3;
  grabberPin = grabber;  
}

void ArmManager::initialize() {
  Serial.println("BEGINNING ARM INITIALIZATION");
  // Establish Pin Modes
  pinMode(s1Pin, OUTPUT);
  pinMode(s2Pin, OUTPUT);
  pinMode(s3Pin, OUTPUT);
  pinMode(grabberPin, OUTPUT);
  
  // Attach pins to servos
  jointOne.attach(s1Pin);
  jointTwo.attach(s2Pin);
  jointThree.attach(s3Pin);
  jointGrabber.attach(grabberPin);
  
  // Write to initial positions
  this->homePosition();

  Serial.println("ARM INITIALIZATION COMPLETE");
}

void ArmManager::raiseArm() {
  Serial.println("RAISING ARM");
  return;  
}

void ArmManager::lowerArm() {
  Serial.println("LOWERING ARM");
  return;  
}

void ArmManager::homePosition() {
  Serial.println("RETURNING ARM TO HOME");
  jointOne.write(0);
  jointTwo.write(0);
  jointThree.write(0);
  return;  
}

void ArmManager::grabJuicebox() {
  Serial.println("GRABBING JUICEBOX");
  return;
}

void ArmManager::releaseJuicebox() {
  Serial.println("RELEASEING JUICEBOX");
  return;
}

bool ArmManager::moveArm(int targetTheta, int targetPhi, int restTime) {
  /* Moves the three servos of the arm in tandem with eachother. s3 will adjust to keep the grabber parallel to the ground.
  targetTheta is the angle of joint 1, targetPhi is the angle of joint 2, s1 is the servo object for joint 1, "" for 2 and 3.

  The return is true if the desired angles have been reached, false otherwise. This CAN create issues if we try to write to impossible angles
  */

  // Move the first servo
  int j1_write = 2*(targetTheta - jointOneInitialAngle);
  if (j1_write > 180 || j1_write < 0) {
    Serial.println("JOINT ONE GIVEN IMPROPER RANGE");
    return false; //   
  }
  this->moveServo(j1_write, restTime, this->jointOne);

  // Move the second servo
  int j2_write = (jointTwoInitialAngle - targetPhi);
  if (j2_write > 180 || j1_write < 0) {
    Serial.println("JOINT TWO GIVEN IMPROPER RANGE");
    return false;  
  }
  this->moveServo(j2_write, restTime, this->jointTwo);

  // Balance the third servo
  int theta = jointOneInitialAngle + (this->jointOne.read() / 2);
  int alpha = jointTwoInitialAngle - this->jointTwo.read();
  if(cos((theta + alpha) * M_PI/180) <= 0) {
    this->jointThree.write(jointThreeInitialAngle - (180 - (theta + alpha))); 
  }
  else {
    this->jointThree.write(jointThreeInitialAngle + (theta + alpha));
  }
  if (this->jointOne.read() == j1_write && this->jointTwo.read() == j2_write) {
    return true; // Final turn of the servos  
  }
  return false; // Not the final turn, keep moving
}

void ArmManager::moveServo(int targetAngle, int restTime, Servo s) {
  /* Moves the servo s one degree at a time with restTime milliseconds between each degree.*/

  /* To move the arm to a given angle, we read the current angle, then adjust it in the direction we need to move into. IF we're giving the max PWM signal and can't get the angle, we return
  and print a warning to the serial monitor that the angle cannot be reached. */

  int currentAngle = s.read();
  delay(restTime);
    
  // static unsigned int startTime = millis(); The static unsigned int was having a rough time when we were calling it for multiple servos. We COULD do it this way, but we'd need one unified function
  // It may be worth doing this though IF we need to do stuff in "parallel" since delays can be problematic as they freeze the stack. Come back if its an issue.
  if (true) {
    // startTime = millis();
    // Move the servo
    if (currentAngle > targetAngle) {
        currentAngle = currentAngle - 1; 
    }
    else if (currentAngle < targetAngle) {
      currentAngle = currentAngle + 1;  
    }
    else {
      return; // target angle reached, no need to move again
    }
    // Serial.print("MOVING SERVO TO "); Verbose is good for debugging, but slows b/c println takes some time.
    // Serial.print(currentAngle);
    // Serial.println(" DEGREES");
    s.write(currentAngle);
  }
  return; // Not the right timing to move  
}
