/*
 * Basic rover movement functions are defined here
 */

#include "constants.h"

/*
 * Moves the rover forward, takes the rover speed
 * as the input
 * 
 * Moves the rover by speeding up motor A and motor B in
 * two opposite directions, this is needed because the motors have 
 * been placed such that moving these 2 motors in opposite direction
 * moves the rover in 1 direction: backward or forward.
 */
void moveForward(int roverSpeed){
  
  digitalWrite(motorA_dir_pin, LOW);
  digitalWrite(motorA_brake_pin, LOW);
  analogWrite(motorA_speed_pin, roverSpeed);

  digitalWrite(motorB_dir_pin, HIGH);
  digitalWrite(motorB_brake_pin, LOW);
  analogWrite(motorB_speed_pin, roverSpeed);
  
}

/*
 * Moves the rover backward, takes the rover speed
 * as the input
 */
void moveBackward(int roverSpeed){
  
  digitalWrite(motorA_dir_pin, HIGH);
  digitalWrite(motorA_brake_pin, LOW);
  analogWrite(motorA_speed_pin, roverSpeed);

  digitalWrite(motorB_dir_pin, LOW);
  digitalWrite(motorB_brake_pin, LOW);
  analogWrite(motorB_speed_pin, roverSpeed);

}
  digitalWrite(motorB_dir_pin, HIGH);
  digitalWrite(motorB_brake_pin, LOW);
  analogWrite(motorB_speed_pin, roverSpeed);
}
