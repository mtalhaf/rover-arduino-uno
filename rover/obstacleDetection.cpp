/*
 * Obstacle Detection class implementation
 */
 
#include "obstacleDetection.h"

/*
 * Start of Obstacle Detection class
 */

ObstacleDetection::ObstacleDetection(Movement* movement, Ultrasonic* ultrasonic, LiquidCrystal_I2C* lcd, boolean displayOnLcd, uint8_t distanceThreshold) : movement(movement), ultrasonic(ultrasonic), lcd(lcd){
  this->displayOnLcd = displayOnLcd;
  this->distanceThreshold = distanceThreshold;
}
 
/*
 * detects obstacles in the rovers path 
 */

boolean ObstacleDetection::detectObstacles(){

  int distanceToObject; // distance to the detected object
  
  ultrasonic->echoUltraSonic(); //echos the ultra sonic to check for obstacles
  distanceToObject = ultrasonic->getDistance();

  /*
  * prints out the distance to the nearest object
  */
  if (displayOnLcd){
    lcd->clear();
    lcd->setCursor(0,0);
    lcd->print(F("distance:"));
    lcd->setCursor(0,1);
    lcd->print(distanceToObject, DEC);
  }

  if (distanceToObject > 0 && distanceToObject <= distanceThreshold)
    return true;
 

  return false;
 
}

void ObstacleDetection::avoidObstacle(){

  uint8_t turnDirection = random(1,3); // initialise random turn to take left/ right
  uint8_t roverDirection = random(1,3); // initialise random direction to take forward/ back
  boolean obstacle = detectObstacles(); // variable to see if obstacle is still in place
  boolean avoidedObtacle = false; // variable t check if an obstacle was avoided

  // if obstacle is still there keep turning the rover
  while(obstacle){

    if (displayOnLcd){
      lcd->clear();
      lcd->setCursor(0,0);
      lcd->print(F("Obstacle ahead"));
      lcd->setCursor(0,1);
      lcd->print(F("Avoiding"));
    }
    
    movement->turnRoverWithoutMovement(ROVER_SPEED, turnDirection); // turns the rover at full speed in the random turn
    obstacle = detectObstacles();
    avoidedObtacle = true;
  }

  if (avoidedObtacle)
    movement->turnRoverWithoutMovement(ROVER_SPEED, turnDirection, 1000); // keeps the rover turning for 1 second to completely avoid the obstacle
}

