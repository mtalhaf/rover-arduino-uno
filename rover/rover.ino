/*
 This code runs the rover INSERT WHAT ROVER DOES.

 The code is supposed to run on the Arduino UNO and is built for
 SOFT561 module as a project.

 VERSION: v0.1.2
 */

#include "obstacle.h"

/*
 * Sets up motor pins in the main setup function
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

/*
 * Sets up the pins for the ultrasonic rangefinder.
 * the trigger pin is used as an output and the echo
 * pin is to read the input from the rangefinder.
 */

void setUpUltraSonicRangeFinderPins() {
  
  pinMode(ultrasonic_trigger_pin, OUTPUT); //set ultrasonic trigger pin to output
  pinMode(ultrasonic_echo_pin, INPUT); //set ultrasonic echo pin to output
  
}

/*
 * Sets up LED pins for debugging purposes, obstacle found, not moving etc etc
 */

void setUpLedPins(){
  pinMode(led_pin, OUTPUT);
}

void setup() {

  setUpMotorPins(); //sets up all the motor pins
  setUpUltraSonicRangeFinderPins(); //sets up all ultra sonic pins
  setUpLedPins(); //sets up led pin

}

/*
 * moves the rover around the room and changes direction if an obstacle is hit
 */
void moveRoverAround(){

  moveForward(ROVER_SPEED);
  
  if (detectObstacles()) //if obstacles are detected in front of the rover
    avoidObstacle(); //avoid the obstacles
}

/*
 * moves the rover in all directions
 */

void moveInAllDirections(){
   // moves the robot forward
  moveForward(ROVER_SPEED);
  delay(1000);
  
  moveBackward(ROVER_SPEED);
  delay(1000);
  
  turnLeft(ROVER_SPEED, 1000);

  turnLeftBack(ROVER_SPEED, 1000);
  
  turnRight(ROVER_SPEED, 1000);
  
  turnRightBack(ROVER_SPEED, 1000);
  
  stopRoverMotors();
}

void loop() {
  moveRoverAround();
}
