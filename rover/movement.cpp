/*
 * Movement class implementation
 */

#include "movement.h"

/*
 * Start of movement class
 */

// Constructor implementation
Movement::Movement(Motor* motorA, Motor* motorB) : motorA(motorA), motorB(motorB){}

/*
 * Moves the rover in the specified direction
 * 
 * Moves the rover by speeding up motor A and motor B in
 * two opposite directions, this is needed because the motors have 
 * been placed such that moving these 2 motors in opposite direction
 * moves the rover in 1 direction: backward or forward.
 */
void Movement::moveRover(int roverSpeed, int roverDirection){

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
void Movement::moveForward(int roverSpeed){
  moveRover(roverSpeed, ROVER_FORWARD_DIRECTION);
}

/*
 * Moves the rover backward, takes the rover speed
 * as the input
 */
void Movement::moveBackward(int roverSpeed){
  moveRover(roverSpeed, ROVER_BACKWARD_DIRECTION);
}

/*
 * Turns the rover in the specified direction
 * for a specific amount of time
 * 
 * The rover turns by speeding up 1 motor
 * and braking/ stopping the other. This gives a 
 * turning effect which turns the rover.
 * 
 * After the delay the motors are also stopped.
 */
void Movement::turnRover(int roverSpeed, int roverDirection, int roverTurn, int turnDelay){

  // switchs which way to turn the rover in
  switch(roverTurn){    
    case ROVER_TURN_LEFT:
      motorB->stopMotor(); // stops motor B
      roverDirection == ROVER_FORWARD_DIRECTION ? motorA->moveForward() : motorA->moveBackward(); // uses inline if else to decide to move the motor forward or backward based on the parameter
      motorA->disengageBrake(); // disengages motor A brakes so it can move
      motorA->motorSpeed(roverSpeed); // sets motor A speed and starts it
    break;
    
    case ROVER_TURN_RIGHT:
      motorA->stopMotor(); // stops motor A
      roverDirection == ROVER_FORWARD_DIRECTION ? motorB->moveForward() : motorB->moveBackward(); // uses inline if else to decide to move the motor forward or backward based on the parameter
      motorB->disengageBrake(); // disengages motor B brakes so it can move
      motorB->motorSpeed(roverSpeed); // sets motor B speed and starts it
    break;
  }

  delay(turnDelay); // delays the turn of the rover by the turn delay, this is needed to make sure that the rover stops after turning
  
}

/*
 * Turns the rover in the specified direction
 * 
 * The rover turns by speeding up 1 motor
 * and braking/ stopping the other. This gives a 
 * turning effect which turns the rover.
 * 
 */
void Movement::turnRover(int roverSpeed, int roverDirection, int roverTurn){
  turnRover(roverSpeed, roverDirection, roverTurn, 0);
}

/*
 * Turns the rover in the specified turn
 * 
 * The rover turns by speeding up 1 motor
 * and the other in a seperate direction. This gives a 
 * turning effect which turns the rover.
 * 
 */
void Movement::turnRoverWithoutMovement(int roverSpeed, int roverTurn, int turnDelay){
  
  // switchs which way to turn the rover in
  switch(roverTurn){    
    case ROVER_TURN_LEFT:
      //motorB->stopMotor(); // stops motor B
      motorA->moveForward(); 
      motorB->moveBackward();
      disengageRoverBrakes(); // disengages motor brakes so it can move
      setRoverSpeed(roverSpeed); // sets motor speed and starts it
    break;
    
    case ROVER_TURN_RIGHT:
      //motorA->stopMotor(); // stops motor A
      motorB->moveForward(); 
      motorA->moveBackward();
      disengageRoverBrakes(); // disengages motor brakes so it can move
      setRoverSpeed(roverSpeed); // sets motor speed and starts it
    break;
  }

  delay(turnDelay); // delays the turn of the rover by the turn delay, this is needed to make sure that the rover stops after turning
}

/*
 * Turns the rover in the specified turn
 * 
 * The rover turns by speeding up 1 motor
 * and the other in a seperate direction. This gives a 
 * turning effect which turns the rover.
 * 
 */
void Movement::turnRoverWithoutMovement(int roverSpeed, int roverTurn){
  turnRoverWithoutMovement(roverSpeed, roverTurn, 0);
}


 /*
  * Turn the rover left, takes the rover speed
  * as the input and the turnDelay
  */
void Movement::turnLeftForward(int roverSpeed, int turnDelay){
  turnRover(roverSpeed, ROVER_FORWARD_DIRECTION, ROVER_TURN_LEFT, turnDelay);
}

/*
 * Turn the rover right, takes the rover speed
 * as the input and the turnDelay
 */
void Movement::turnRightForward(int roverSpeed, int turnDelay){
  turnRover(roverSpeed, ROVER_FORWARD_DIRECTION, ROVER_TURN_RIGHT, turnDelay);
}

/*
 * Turn the rover left backwards, takes the rover speed
 * as the input and the turnDelay
 */
void Movement::turnLeftBack(int roverSpeed, int turnDelay){
  turnRover(roverSpeed, ROVER_BACKWARD_DIRECTION, ROVER_TURN_LEFT, turnDelay);
}

/*
 * Turn the rover right backwards, takes the rover speed
 * as the input and the turnDelay
 */
void Movement::turnRightBack(int roverSpeed, int turnDelay){
  turnRover(roverSpeed, ROVER_BACKWARD_DIRECTION, ROVER_TURN_RIGHT, turnDelay);
}

// moves the rover forward
void Movement::roverMotorsForward(){
  motorA->moveForward();
  motorB->moveForward();
}

// moves the rover backward
void Movement::roverMotorsBackward(){
  motorA->moveBackward();
  motorB->moveBackward();
}

// starts both rover motors to move the rover
void Movement::disengageRoverBrakes(){
  motorA->disengageBrake();
  motorB->disengageBrake();
}

// stops both rover motors
void Movement::stopRoverMotors(){
  motorA->stopMotor();
  motorB->stopMotor();
}

// sets the rover speed
void Movement::setRoverSpeed(int roverSpeed){
  motorA->motorSpeed(roverSpeed);
  motorB->motorSpeed(roverSpeed);
}

