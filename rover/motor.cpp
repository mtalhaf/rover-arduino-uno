/*
 * Motor class implementation
 */

#include "motor.h"

/*
 * Start of motor class
 */

// Constructor implementation
Motor::Motor(int motorDirPin, int motorBrakePin, int motorSpeedPin){
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

// Move motor forward
void Motor::moveForward(){
  digitalWrite(motor_dir_pin, HIGH);
}

// Moves motor backward
void Motor::moveBackward(){
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

// Sets the speed of the motor
void Motor::motorSpeed(int motorSpeed){
  analogWrite(motor_speed_pin, motorSpeed);
}
