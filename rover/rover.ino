/*
 This code runs the rover INSERT WHAT ROVER DOES.

 The code is supposed to run on the Arduino UNO and is built for
 SOFT561 module as a project.

 VERSION: v0.1.1
 */

#include "movement.h"

/*Sets up motor pins in the main setup function
 * goes through direction and brake pins of motor
 * A and B and initiates them as output pins to 
 * send signals to the motor.
 * 
 * The speed pin does not needs to be initiated as
 * its a dedicated output PWM pin.
 */

void setUpMotorPins() {
  
  pinMode(motorA_dir_pin, OUTPUT); //set motor A direction pin to output
  pinMode(motorA_brake_pin, OUTPUT); //set motor A brake pin to output

  pinMode(motorB_dir_pin, OUTPUT); //set motor B direction pin to output
  pinMode(motorB_brake_pin, OUTPUT); //set motor B brake pin to output
  
}

void setup() {

  setUpMotorPins(); //sets up all the motor pins

}

void loop() {
  
  // moves the robot forward
  moveForward(200);
  delay(1000);
  
  moveBackward(200);
  delay(1000);
  
  turnLeft(255, 1000);

  turnLeftBack(255, 1000);
  
  turnRight(255, 1000);
  
  turnRightBack(255, 1000);
  
  stopRoverMotors();
  delay(50000);
  
}
