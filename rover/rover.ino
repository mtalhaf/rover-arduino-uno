/*
 This code runs the rover INSERT WHAT ROVER DOES.

 The code is supposed to run on the Arduino UNO and is built for
 SOFT561 module as a project.

 VERSION: v0.2.0
 */

#include "init.h"
#include "obstacleDetection.h"
#include "movement.h"
#include "rosCallbackChecks.h"

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
    
  if (frontEdgeObstacleDetection->detectEdges()) //if edge are detected in front of the rover
    frontEdgeObstacleDetection->avoidEdge(); //avoid the edge
}

/*
 * moves the rover around its place and changes direction if an obstacle is presented
 */
void shyRover(){
  
  if (obstacleDetection->detectObstacles()) //if obstacles are detected in front of the rover
    obstacleDetection->avoidObstacle(); //avoid the obstacles
  else
    movement->stopRoverMotors();
    
  //frontEdgeObstacleDetection->detectObstacles();
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

void rosCommands(){
  if (stopRoverMovement){
    movement->stopRoverMotors();
  }

  if (moveForward){
    movement->moveForward(ROVER_SPEED);
  }

  if (moveBackward){
     movement->moveBackward(ROVER_SPEED);
  }

  if (turnLeft){
    movement->turnRoverWithoutMovement(ROVER_SPEED, ROVER_TURN_LEFT);
  }

  if (turnRight){
    movement->turnRoverWithoutMovement(ROVER_SPEED, ROVER_TURN_RIGHT);
  }
}

void loop() {
  //moveRoverAround();
  rosCommands();
  rosLoop();
  //shyRover();
}
