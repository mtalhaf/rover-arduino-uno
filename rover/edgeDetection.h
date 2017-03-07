/*
 * edge detection class definition
 */

#ifndef edgeDetection_h
#define edgeDetection_h

#include "Arduino.h"
#include "constants.h"
#include "movement.h"
#include "motor.h"
#include "ultrasonic.h"
#include "Wire.h"
#include "LiquidCrystal_I2C.h" // Library included seperately to control the LCD display

/*
 * Start of edge detection class
 */
class EdgeDetection{

  // private movement variables
  private:
    Movement* movement; // movement object
    Ultrasonic* ultrasonic; // ultrasonic object
    LiquidCrystal_I2C* lcd; // lcd object
    boolean displayOnLcd;
    uint8_t distanceThreshold;
    uint8_t thresholdBounds;

  // public methods for edge detection
  public:
    EdgeDetection(Movement* movement, Ultrasonic* ultrasonic, LiquidCrystal_I2C* lcd, boolean displayOnLcd, uint8_t distanceThreshold, uint8_t thresholdBounds); // initialises edge detection class with bounds for the threshold
    boolean detectEdges(); // detects edges
    void avoidEdge(); // avoids edge

};

#endif
