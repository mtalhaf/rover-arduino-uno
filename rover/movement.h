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
    void moveRover(uint8_t roverSpeed); // sets the rover speed
    
    void moveRover(uint8_t roverSpeed, uint8_t roverDirection); // moves the rover in a specific direction
    void moveForward(uint8_t roverSpeed); // moves the rover forward
    void moveBackward(uint8_t roverSpeed); // moves the rover backward
    
    void turnRover(uint8_t roverSpeed, uint8_t roverDirection, uint8_t roverTurn, uint8_t turnDelay); // turns the rover in a specific direction and turn for a specific amount of time
    void turnRover(uint8_t roverSpeed, uint8_t roverDirection, uint8_t roverTurn); // keeps turning the rover in a specific direction and turn 
    void turnRoverWithoutMovement(uint8_t roverSpeed, uint8_t roverTurn, uint8_t turnDelay); // turns the rover in a specific turn 
    void turnRoverWithoutMovement(uint8_t roverSpeed, uint8_t roverTurn); // keeps turning the rover in a specific turn 
    void turnLeftForward(uint8_t roverSpeed, uint8_t turnDelay); // turns the rover left forward
    void turnRightForward(uint8_t roverSpeed, uint8_t turnDelay); // turns the rover right forward
    void turnLeftBack(uint8_t roverSpeed, uint8_t turnDelay); // turns the rover left backwards
    void turnRightBack(uint8_t roverSpeed, uint8_t turnDelay); // turns the rover right backwards
};

#endif
