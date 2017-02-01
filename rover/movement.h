/*
 * Basic rover movement functions are defined here
 */

#include "motor.h"

/*
 * Moves the rover in the specified direction
 * 
 * Moves the rover by speeding up motor A and motor B in
 * two opposite directions, this is needed because the motors have 
 * been placed such that moving these 2 motors in opposite direction
 * moves the rover in 1 direction: backward or forward.
 */
void moveRover(int roverSpeed, int roverDirection){
  
  switch(roverDirection){
    case ROVER_FORWARD_DIRECTION:
      roverMotorsForward();
    break;
    case ROVER_BACKWARD_DIRECTION:
      roverMotorsBackward();
    break;
  }

  startRoverMotors();

  setRoverSpeed(roverSpeed);
}

/*
 * Moves the rover forward, takes the rover speed
 * as the input
 */
void moveForward(int roverSpeed){
  moveRover(roverSpeed, ROVER_FORWARD_DIRECTION);
}

/*
 * Moves the rover backward, takes the rover speed
 * as the input
 */
void moveBackward(int roverSpeed){
  moveRover(roverSpeed, ROVER_BACKWARD_DIRECTION);
}

/*
 * Turns the rover in the specified direction
 * 
 * The rover turns left by speeding up motorA
 * and braking/ stopping motorB. This give a 
 * turning effect which in turn the rover.
 * After the delay motorA is also stopped.
 */
void turnLeft(int roverSpeed, int turnDelay){

  digitalWrite(motorB_brake_pin, HIGH);

  digitalWrite(motorA_dir_pin, LOW);
  digitalWrite(motorA_brake_pin, LOW);
  analogWrite(motorA_speed_pin, roverSpeed);

  delay(turnDelay);

  digitalWrite(motorA_brake_pin, HIGH);
  
}

/*
 * Turn the rover right, takes the rover speed
 * as the input and the turnDelay
 */
void turnRight(int roverSpeed, int turnDelay){

  digitalWrite(motorA_brake_pin, HIGH);

  digitalWrite(motorB_dir_pin, HIGH);
  digitalWrite(motorB_brake_pin, LOW);
  analogWrite(motorB_speed_pin, roverSpeed);

  delay(turnDelay);

  digitalWrite(motorB_brake_pin, HIGH);
  
}

/*
 * Turn the rover left backwards, takes the rover speed
 * as the input and the turnDelay
 */
void turnLeftBack(int roverSpeed, int turnDelay){

  digitalWrite(motorB_brake_pin, HIGH);

  digitalWrite(motorA_dir_pin, HIGH);
  digitalWrite(motorA_brake_pin, LOW);
  analogWrite(motorA_speed_pin, roverSpeed);

  delay(turnDelay);

  digitalWrite(motorA_brake_pin, HIGH);
  
}

/*
 * Turn the rover right backwards, takes the rover speed
 * as the input and the turnDelay
 */
void turnRightBack(int roverSpeed, int turnDelay){

  digitalWrite(motorA_brake_pin, HIGH);

  digitalWrite(motorB_dir_pin, LOW);
  digitalWrite(motorB_brake_pin, LOW);
  analogWrite(motorB_speed_pin, roverSpeed);

  delay(turnDelay);

  digitalWrite(motorB_brake_pin, HIGH);
  
}

