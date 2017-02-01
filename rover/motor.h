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

/*
 * Functions for Motor A & Motor B both
 */

void roverMotorsForward(){
  motorA_Forward();
  motorB_Forward();
}

void roverMotorsBackward(){
  motorA_Backward();
  motorB_Backward();
}

void startRoverMotors(){
  motorA_Start();
  motorB_Start();
}

void stopRoverMotors(){
  motorA_Stop();
  motorB_Stop();
}

