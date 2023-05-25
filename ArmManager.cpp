#include "ArmManager.h"
#include <Servo.h>
#include <Arduino.h>

ArmManager::ArmManager(int s1, int s2, int s3, int grabber, int ps) {
  // Establish Servo Pin Numbers
  s1Pin = s1;
  s2Pin = s2;
  s3Pin = s3;
  grabberPin = grabber;
  pressureSensor = ps;  
}

void ArmManager::initialize() {
  Serial.println("BEGINNING ARM INITIALIZATION");
  // Establish Pin Modes
  pinMode(s1Pin, OUTPUT);
  pinMode(s2Pin, OUTPUT);
  pinMode(s3Pin, OUTPUT);
  pinMode(grabberPin, OUTPUT);
  pinMode(pressureSensor, INPUT);
  
  // Attach pins to servos
  jointOne.attach(s1Pin);
  jointTwo.attach(s2Pin);
  jointThree.attach(s3Pin);
  jointGrabber.attach(grabberPin);
  
  // Write to initial positions
  homePosition();

  Serial.println("ARM INITIALIZATION COMPLETE");
}

void ArmManager::raiseArm() {
  int startTime = millis();
  while(!moveArm(40, 60, 5) && (millis() - startTime < 10000)) {}
  return;  
}

void ArmManager::lowerArm() {
  int startTime = millis();
  while(!moveArm(120, 60, 5) && (millis() - startTime < 10000)) {}
  return;
}

void ArmManager::homePosition() {
  int startTime = millis();
  Serial.println("TEST");
  while(!moveArm(jointOneInitialAngle, jointTwoInitialAngle, 5) && (millis() - startTime < 10000)) {Serial.println("MOVING HOME");}
  return;  
}

void ArmManager::grabJuicebox() {

  int targetPressure = 10; 
  
  int startTime = millis();
  while(analogRead(pressureSensor) < targetPressure && (millis() - startTime < 10000)) {
    moveServo(180, 2, jointGrabber);  
  }
  return;
}

void ArmManager::releaseJuicebox() {
  jointGrabber.write(0); // Release the servo. We can do this at full speed.
  return;
}

bool ArmManager::moveArm(int targetTheta, int targetAlpha, int restTime) {
  /* Moves the three servos of the arm in tandem with eachother. s3 will adjust to keep the grabber parallel to the ground.
  targetTheta is the angle of joint 1, targetPhi is the angle of joint 2, s1 is the servo object for joint 1, "" for 2 and 3.

  The return is true if the desired angles have been reached, false otherwise. This CAN create issues if we try to write to impossible angles
  */

  // Move the first servo
  int j1_write = 2*(targetTheta - jointOneInitialAngle); // Angle that j1 should achieve
  if (j1_write > 180 || j1_write < 0) {
    Serial.println("JOINT ONE GIVEN IMPROPER RANGE");
    return false; //   
  }
  moveServo(translateAngle(j1_write, true), restTime, jointOne);

  // Move the second servo
  int j2_write = (jointTwoInitialAngle - targetAlpha); // Angle that j2 should achieve
  if (j2_write > 180 || j1_write < 0) {
    Serial.println("JOINT TWO GIVEN IMPROPER RANGE");
    return false;  
  }
  moveServo(translateAngle(j2_write, true), restTime, jointTwo);

  // Balance the third servo
  int theta = jointOneInitialAngle + (translateAngle(jointOne.read(), false) / 2);
  int alpha = jointTwoInitialAngle - translateAngle(jointTwo.read(), false);
  int phi;
  if(cos((theta + alpha) * M_PI/180) <= 0) { // Point FORWARDS
    phi = (180 - (theta + alpha)); 
  }
  else { // Point BACKWARDS
    phi = (0 - (theta + alpha));
  }
  // Check if safe
  if (phi > 90) {
    phi = 90;  
  }
  else if (phi < jointThreeInitialAngle) {
    phi = jointThreeInitialAngle;  
  }
  int jointThreeToWrite = phi - jointThreeInitialAngle;
  jointThree.write(jointThreeToWrite);

  // Check if done
  if (abs(translateAngle(jointOne.read(), false) - j1_write) <= 1 && abs(translateAngle(jointTwo.read(), false) - j2_write) <= 1) { // +- 1 degree
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

int ArmManager::translateAngle(int translate, bool toWrite) {
  /* Transllates the DESIRED angle to the angle TO WRITE to the large servos, which have a larger range. Can also translate back */
  if (toWrite) {
    float turnPercent = translate / 180.0f; 
    return round(130 * turnPercent); 
  }
  else {
    float turnPercent = translate / 130.0f;
    return round(180 * turnPercent);
  }
}
