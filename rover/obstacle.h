/*
 * obstacle avoidance detection and aviodance funtions 
 */

#include "ultrasonic.h"
#include "movement.h"
#include "led.h"

/*
 * detects obstacles in the rovers path 
 */

boolean detectObstacles(){

  long distanceToObject; // distance to the detected object
  
  echoUltraSonic(); //echos the ultra sonic to check for obstacles
  distanceToObject = getDistance();

  if (distanceToObject <= OBSTACLE_DETECTION_DISTANCE)
    return true;
  else
    return false;
    
}

void avoidObstacle(){

  int turnDirection = random(1,3); // initialise random turn to take left/ right
  int roverDirection = random(1,3); // initialise random direction to take forward/ back
  boolean obstacle = true; // variable to see if obstacle is still in place
  
  // if obstacle is still there keep turning the rover
  while(obstacle){
    turnLedOn();
    turnRover(ROVER_SPEED, roverDirection, turnDirection, 2000); // turns the rover at full speed in the random turn and direction for 2 seconds
    obstacle = detectObstacles();
  }

  turnLedOff();
  
}

