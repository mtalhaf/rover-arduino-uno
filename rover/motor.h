/*
 * Motor class definition
 */

#ifndef motor_h
#define motor_h

#include "Arduino.h"

/*
 * Start of motor class
 */

class Motor{

  // private motor variables
  private:
    int motor_dir_pin;   //motor direction pin
    int motor_brake_pin; //motor brake pin
    int motor_speed_pin; //motor speed pin

  // public methods for motor
  public:
    Motor(int motorDirPin, int motorBrakePin, int motorSpeedPin); // initialises the motor
    void setUp(); // sets up the motor
    void moveForward();  // moves the motor forward
    void moveBackward(); // moves the motor backward
    void disengageBrake(); // disengages the brakes
    void stopMotor(); // stops the motor
    void motorSpeed(int motorSpeed); // sets the motor speed
};
#endif
