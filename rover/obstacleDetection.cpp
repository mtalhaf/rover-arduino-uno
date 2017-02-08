/*
 * Obstacle Detection class implementation
 */
 
#include "obstacleDetection.h"

/*
 * Start of Obstacle Detection class
 */

ObstacleDetection::ObstacleDetection(Movement* movement, Ultrasonic* ultrasonic) : movement(movement), ultrasonic(ultrasonic){}
 
/*
 * detects obstacles in the rovers path 
 */

boolean ObstacleDetection::detectObstacles(){

  long distanceToObject; // distance to the detected object
  
  ultrasonic->echoUltraSonic(); //echos the ultra sonic to check for obstacles
  distanceToObject = ultrasonic->getDistance();

  /*
   * prints out the distance to the nearest object
   */
  
  lcd.clear(); 
  lcd.setCursor(0,0);
  lcd.print("distance:");
  lcd.setCursor(0,1);
  lcd.print(distanceToObject, DEC);

  if (distanceToObject > 0 && distanceToObject <= OBSTACLE_DETECTION_DISTANCE){
    return true;
  }else{
    return false;
  }
    
}

void ObstacleDetection::avoidObstacle(){

  int turnDirection = random(1,3); // initialise random turn to take left/ right
  int roverDirection = random(1,3); // initialise random direction to take forward/ back
  boolean obstacle = true; // variable to see if obstacle is still in place
  
  // if obstacle is still there keep turning the rover
  while(obstacle){
    movement->turnRover(ROVER_SPEED, roverDirection, turnDirection, 2000); // turns the rover at full speed in the random turn and direction for 2 seconds
    obstacle = detectObstacles();
  }
  
}

