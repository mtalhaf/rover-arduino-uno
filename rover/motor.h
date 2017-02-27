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
    uint8_t motor_dir_pin;   //motor direction pin
    uint8_t motor_brake_pin; //motor brake pin
    uint8_t motor_speed_pin; //motor speed pin

  // public methods for motor
  public:
    Motor(uint8_t motorDirPin, uint8_t motorBrakePin, uint8_t motorSpeedPin); // initialises the motor
    void setUp(); // sets up the motor
    void setForwardDirection();  // sets the motor to rotate in the forward direction
    void setBackwardDirection(); // sets the motor to rotate in the backward direction
    void disengageBrake(); // disengages the brakes
    void stopMotor(); // stops the motor
    void moveMotor(uint8_t motorSpeed); // starts moving the motor with the given motor speed
};
#endif
