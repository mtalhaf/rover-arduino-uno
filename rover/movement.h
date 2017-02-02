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

  // Switches the rover direction to know which direction to move the motors in
  switch(roverDirection){
    case ROVER_FORWARD_DIRECTION:
      roverMotorsForward(); // Moves the rover forward
    break;
    case ROVER_BACKWARD_DIRECTION:
      roverMotorsBackward();  // Moves the rover backward
    break;
  }

  disengageRoverBrakes(); // disengages the rover brakes so it can move

  setRoverSpeed(roverSpeed); // Set the rover speed and starts moving it
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
 * The rover turns by speeding up 1 motor
 * and braking/ stopping the other. This gives a 
 * turning effect which turns the rover.
 * 
 * After the delay the motors are also stopped.
 */
void turnRover(int roverSpeed, int roverDirection, int roverTurn, int turnDelay){

  // switchs which way to turn the rover in
  switch(roverTurn){    
    case ROVER_TURN_LEFT:
      motorB_Stop(); // stops motor B
      roverDirection == ROVER_FORWARD_DIRECTION ? motorA_Forward() : motorA_Backward(); // uses inline if else to decide to move the motor forward or backward based on the parameter
      motorA_DisengageBrakes(); // disengages motor A brakes so it can move
      motorA_Speed(roverSpeed); // sets motor A speed and starts it
    break;
    
    case ROVER_TURN_RIGHT:
      motorA_Stop(); // stops motor A
      roverDirection == ROVER_FORWARD_DIRECTION ? motorB_Forward() : motorB_Backward(); // uses inline if else to decide to move the motor forward or backward based on the parameter
      motorB_DisengageBrakes(); // disengages motor B brakes so it can move
      motorB_Speed(roverSpeed); // sets motor B speed and starts it
    break;
  }

  delay(turnDelay); // delays the turn of the rover by the turn delay, this is needed to make sure that the rover stops after turning
  stopRoverMotors(); // stops the rover from moving
  
}


 /*
  * Turn the rover left, takes the rover speed
  * as the input and the turnDelay
  */
void turnLeft(int roverSpeed, int turnDelay){
  turnRover(roverSpeed, ROVER_FORWARD_DIRECTION, ROVER_TURN_LEFT, turnDelay);
}

/*
 * Turn the rover right, takes the rover speed
 * as the input and the turnDelay
 */
void turnRight(int roverSpeed, int turnDelay){
  turnRover(roverSpeed, ROVER_FORWARD_DIRECTION, ROVER_TURN_RIGHT, turnDelay);
}

/*
 * Turn the rover left backwards, takes the rover speed
 * as the input and the turnDelay
 */
void turnLeftBack(int roverSpeed, int turnDelay){
  turnRover(roverSpeed, ROVER_BACKWARD_DIRECTION, ROVER_TURN_LEFT, turnDelay);
}

/*
 * Turn the rover right backwards, takes the rover speed
 * as the input and the turnDelay
 */
void turnRightBack(int roverSpeed, int turnDelay){
  turnRover(roverSpeed, ROVER_BACKWARD_DIRECTION, ROVER_TURN_RIGHT, turnDelay);
}

