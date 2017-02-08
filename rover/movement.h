/*
 * Movement class definition
 */

#ifndef movement_h
#define movement_h

#include "Arduino.h"
#include "constants.h"
#include "motor.h"


/*
 * Start of movement class
 */
class Movement{
  
  // private movement variables
  private:
    Motor* motorA;   // motorA definition
    Motor* motorB; // motorB definition

  // public methods for movement
  public:
    Movement(Motor* motorA, Motor* motorB); // initialises the momovement object
    
    void roverMotorsForward(); // moves the rover forward
    void roverMotorsBackward(); // moves the rover backward
    void disengageRoverBrakes(); // disengages the rover brakes
    void stopRoverMotors(); // stops the rover
    void setRoverSpeed(int roverSpeed); // sets the rover speed
    
    void moveRover(int roverSpeed, int roverDirection); // moves the rover in a specific direction
    void moveForward(int roverSpeed); // moves the rover forward
    void moveBackward(int roverSpeed); // moves the rover backward
    
    void turnRover(int roverSpeed, int roverDirection, int roverTurn, int turnDelay); // turns the rover in a specific direction and turn
    void turnLeftForward(int roverSpeed, int turnDelay); // turns the rover left forward
    void turnRightForward(int roverSpeed, int turnDelay); // turns the rover right forward
    void turnLeftBack(int roverSpeed, int turnDelay); // turns the rover left backwards
    void turnRightBack(int roverSpeed, int turnDelay); // turns the rover right backwards
};

#endif
