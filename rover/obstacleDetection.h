/*
 * obstacle detection class definition
 */

#ifndef obstacleDetection_h
#define obstacleDetection_h

#include "Arduino.h"
#include "constants.h"
#include "movement.h"
#include "motor.h"
#include "lcd.h"
#include "ultrasonic.h"

/*
 * Start of obstacle detection class
 */
class ObstacleDetection{

  // private movement variables
  private:
    Movement movement; // movement object
    Ultrasonic ultrasonic; // ultrasonic object

  // public methods for movement
  public:
    ObstacleDetection(Movement* movement, Ultrasonic* ultrasonic); // initialises Obstacle detection class
    boolean detectObstacles(); // detects obstacles
    void avoidObstacle(); // avoids obstacles

};


#endif
