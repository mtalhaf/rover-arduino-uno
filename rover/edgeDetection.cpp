/*
 * Edge Detection class implementation
 */
 
#include "edgeDetection.h"

/*
 * Start of Edge Detection class
 */

EdgeDetection::EdgeDetection(Movement* movement, Ultrasonic* ultrasonic, LiquidCrystal_I2C* lcd, boolean displayOnLcd, int distanceThreshold, int thresholdBounds) : movement(movement), ultrasonic(ultrasonic), lcd(lcd){
  this->displayOnLcd = displayOnLcd;
  this->distanceThreshold = distanceThreshold;
  this->thresholdBounds = thresholdBounds;
}
 
/*
 * detects edges in the rovers path 
 */

boolean EdgeDetection::detectEdges(){

  long distanceToObject; // distance to the detected object
  long maxThreshold = distanceThreshold + thresholdBounds; // maximum ThresholdBound
  long minThreshold = distanceThreshold - thresholdBounds; // minimum THresholdBound
  
  ultrasonic->echoUltraSonic(); //echos the ultra sonic to check for obstacles
  distanceToObject = ultrasonic->getDistance();

  /*
  * prints out the distance to the nearest object
  */
  if (displayOnLcd){
    lcd->clear();
    lcd->setCursor(0,0);
    lcd->print("distance:");
    lcd->setCursor(0,1);
    lcd->print(distanceToObject, DEC);
  }

  if (distanceToObject >= 0 && distanceToObject >= minThreshold && distanceToObject >= maxThreshold)
    return true;
    
  return false;
 
}

void EdgeDetection::avoidEdge(){

  int turnDirection = random(1,3); // initialise random turn to take left/ right
  int roverDirection = random(1,3); // initialise random direction to take forward/ back
  boolean edge = detectEdges(); // variable to see if obstacle is still in place
  boolean avoidedEdge = false; // variable t check if an obstacle was avoided

  //moves the rover backward imediately if an edge is detected
  if (edge){
    movement->roverMotorsBackward(); // turn robot direction backwards
    movement->moveRover(255); // set the rover speed to full to make sure it goes back faster
    delay(100); // move back for 100 milli seconds
    movement->stopRoverMotors(); // stop the rover
  }
  // if obstacle is still there keep turning the rover
  while(edge){

    if (displayOnLcd){
      lcd->clear();
      lcd->setCursor(0,0);
      lcd->print("Obstacle ahead");
      lcd->setCursor(0,1);
      lcd->print("Avoiding");
    }
    movement->turnRoverWithoutMovement(ROVER_SPEED, turnDirection); // turns the rover at full speed in the random turn
    edge = detectEdges();
    avoidedEdge = true;
  }

  if (avoidedEdge)
    movement->turnRoverWithoutMovement(ROVER_SPEED, turnDirection, 1000); // keeps the rover turning for 1 second to completely avoid the obstacle
}

