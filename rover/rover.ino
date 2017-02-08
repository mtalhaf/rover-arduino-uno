/*
 This code runs the rover INSERT WHAT ROVER DOES.

 The code is supposed to run on the Arduino UNO and is built for
 SOFT561 module as a project.

 VERSION: v0.1.4
 */

#include "init.h"
#include "obstacleDetection.h"
#include "movement.h"

void setup() {
  initialise(); // from init.h
}

/*
 * moves the rover around the room and changes direction if an obstacle is hit
 */
void moveRoverAround(){

  movement->moveForward(ROVER_SPEED);
  
  if (obstacleDetection->detectObstacles()) //if obstacles are detected in front of the rover
    obstacleDetection->avoidObstacle(); //avoid the obstacles
}

/*
 * moves the rover in all directions
 */

void moveInAllDirections(){
   // moves the robot forward
  movement->moveForward(ROVER_SPEED);
  delay(1000);
  
  movement->moveBackward(ROVER_SPEED);
  delay(1000);
  
  movement->turnLeftForward(ROVER_SPEED, 1000);

  movement->turnLeftBack(ROVER_SPEED, 1000);
  
  movement->turnRightForward(ROVER_SPEED, 1000);
  
  movement->turnRightBack(ROVER_SPEED, 1000);
  
  movement->stopRoverMotors();
}

void loop() {
  moveRoverAround();
}
