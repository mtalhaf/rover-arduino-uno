/*
 * Motor A and B pin outputs are set here as easy functions
 */

#include "constants.h"

/*
 * Functions for Motor A
 */
void motorA_Forward(){
    digitalWrite(motorA_dir_pin, LOW);// this is set to LOW because this makes the rover goes forward
}

void motorA_Stop(){
    digitalWrite(motorA_brake_pin, HIGH); // stops the motor/ applys the brake
}

void motorA_Backward(){
    digitalWrite(motorA_dir_pin, HIGH);// this is set to HIGH because this makes the rover goes backwards
}

void motorA_Start(){
    digitalWrite(motorA_brake_pin, LOW); // starts the motor/ disengages the brakes
}

void motorA_Speed(int motorSpeed){
  analogWrite(motorA_speed_pin, motorSpeed); // sets motor A speed
}


/*
 * Functions for Motor B
 */
void motorB_Forward(){
    digitalWrite(motorB_dir_pin, HIGH);// this is set to HIGH because this makes the rover goes forward
}

void motorB_Stop(){
    digitalWrite(motorB_brake_pin, HIGH); // stops the motor/ applys the brake
}

void motorB_Backward(){
    digitalWrite(motorB_dir_pin, LOW);// this is set to LOW because this makes the rover goes backwards
}

void motorB_Start(){
    digitalWrite(motorB_brake_pin, LOW); // starts the motor/ disengages the brakes
}

void motorB_Speed(int motorSpeed){
  analogWrite(motorB_speed_pin, motorSpeed); // sets motorB speed
}

/*
 * Functions for Motor A & Motor B both
 */

// moves the rover forward
void roverMotorsForward(){
  motorA_Forward();
  motorB_Forward();
}

// moves the rover backward
void roverMotorsBackward(){
  motorA_Backward();
  motorB_Backward();
}

// starts both rover motors to move the rover
void startRoverMotors(){
  motorA_Start();
  motorB_Start();
}

// stops both rover motors
void stopRoverMotors(){
  motorA_Stop();
  motorB_Stop();
}

// sets the rover speed
void setRoverSpeed(int roverSpeed){
  motorA_Speed(roverSpeed);
  motorB_Speed(roverSpeed);
}

