/*
 * obstacle avoidance detection and aviodance funtions 
 */

#include "ultrasonic.h"

/*
 * detects obstacles in the rovers path 
 */

boolean detectObstacles(long detectionDistance){

  long distanceToObject; // distance to the detected object
  
  echoUltraSonic(); //echos the ultra sonic to check for obstacles
  distanceToObject = getDistance();

  if (distanceToObject <= detectionDistance)
    return true;
  else
    return false;
    
}

