/*
 * Motor class implementation
 */

#include "motor.h"

/*
 * Start of motor class
 */

// Constructor implementation
Motor::Motor(uint8_t motorDirPin, uint8_t motorBrakePin, uint8_t motorSpeedPin){
  motor_dir_pin = motorDirPin;
  motor_brake_pin = motorBrakePin;
  motor_speed_pin = motorSpeedPin;
}

void Motor::setUp(){
  pinMode(motor_dir_pin, OUTPUT); //set motor A direction pin to output
  pinMode(motor_brake_pin, OUTPUT); //set motor A brake pin to output

  pinMode(motor_dir_pin, OUTPUT); //set motor B direction pin to output
  pinMode(motor_brake_pin, OUTPUT); //set motor B brake pin to output
}

// Set forward direction for motor
void Motor::setForwardDirection(){
  digitalWrite(motor_dir_pin, HIGH);
}

// Set backward direction for motor
void Motor::setBackwardDirection(){
    digitalWrite(motor_dir_pin, LOW);
}

// Disengages the brakes on the motor
void Motor::disengageBrake(){
  digitalWrite(motor_brake_pin, LOW);
}

// Stops motor
void Motor::stopMotor(){
  digitalWrite(motor_brake_pin, HIGH);
}

// Moves the motor with the given speed
void Motor::moveMotor(uint8_t motorSpeed){
  analogWrite(motor_speed_pin, motorSpeed);
}
