/*
 * obstacle detection class definition
 */

#ifndef obstacleDetection_h
#define obstacleDetection_h

#include "Arduino.h"
#include "constants.h"
#include "movement.h"
#include "motor.h"
#include "ultrasonic.h"
#include "Wire.h"
#include "LiquidCrystal_I2C.h" // Library included seperately to control the LCD display


/*
 * Start of obstacle detection class
 */
class ObstacleDetection{

  // private movement variables
  private:
    Movement* movement; // movement object
    Ultrasonic* ultrasonic; // ultrasonic object
    LiquidCrystal_I2C* lcd; // lcd object
    boolean displayOnLcd;
    int distanceThreshold;
    int thresholdBounds;
    boolean forObstacleDetection;

  // public methods for movement
  public:
    ObstacleDetection(Movement* movement, Ultrasonic* ultrasonic, LiquidCrystal_I2C* lcd, boolean displayOnLcd, int distanceThreshold); // initialises Obstacle detection class
    ObstacleDetection(Movement* movement, Ultrasonic* ultrasonic, LiquidCrystal_I2C* lcd, boolean displayOnLcd, int distanceThreshold, int thresholdBounds); // initialises Obstacle detection class with bounds for the threshold
    boolean detectObstacles(); // detects obstacles
    void avoidObstacle(); // avoids obstacles

};


#endif
