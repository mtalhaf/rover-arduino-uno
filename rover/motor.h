/*
 * Motor A and B pin outputs are set here as easy functions
 */

#include "constants.h"

/*
 * Motor A
 */
void motorA_DirectionForward(){
    digitalWrite(motorA_dir_pin, LOW);// this is set to LOW because this makes the rover goes forward
}

void motorA_Stop(){
    digitalWrite(motorA_brake_pin, HIGH); // stops the motor/ applys the brake
}

void motorA_DirectionBackward(){
    digitalWrite(motorA_dir_pin, HIGH);// this is set to HIGH because this makes the rover goes backwards
}

void motorA_Start(){
    digitalWrite(motorA_brake_pin, LOW); // starts the motor/ disengages the brakes
}


/*
 * Motor B
 */
void motorB_DirectionForward(){
    digitalWrite(motorB_dir_pin, HIGH);// this is set to HIGH because this makes the rover goes forward
}

void motorB_Stop(){
    digitalWrite(motorB_brake_pin, HIGH); // stops the motor/ applys the brake
}

void motorB_DirectionBackward(){
    digitalWrite(motorB_dir_pin, LOW);// this is set to LOW because this makes the rover goes backwards
}

void motorB_Start(){
    digitalWrite(motorB_brake_pin, LOW); // starts the motor/ disengages the brakes
}

