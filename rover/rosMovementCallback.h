/*
 * Ros Movement callbacks to handle movement topics
 */

#ifndef rosMovementCallback_h
#define rosMovementCallback_h

#include "rosCallbackChecks.h"
#include "rosConstants.h"
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>

 
/*
 * Moves rover forward
 */

void moveRoverForward(){
  moveBackward = false;
  moveForward = true;
  stopRoverMovement = false;
}

/*
 * Moves rover backward
 */

void moveRoverBackward(){
  moveBackward = true;
  moveForward = false;
  stopRoverMovement = false;
}

/*
 * turns rover left
 */

void turnRoverLeft(){
  turnLeft = true;
  turnRight = false;
  stopRoverMovement = false;
}


/*
 * Turns rover right
 */

void turnRoverRight(){
  turnLeft = false;
  turnRight = true;
  stopRoverMovement = false;
}

void moveRover(const std_msgs::UInt8& moveDirection){
  switch(moveDirection.data){
    case 1:
      moveRoverForward();
    break;
    case 2:
      moveRoverBackward();
    break;
    case 3:
      turnRoverLeft();
    break;
    case 4:
      turnRoverRight();
    break;
  }
}



/*
 * Ros subscriber for moving the rover forward
 */
ros::Subscriber<std_msgs::UInt8> movementMoveRoverSubscriber(TOPIC_MOVEMENT_MOVE, &moveRover );



/*
 * Stops the rover from moving
 */

void stopRover(const std_msgs::Empty& message){
  stopRoverMovement = true;
  moveForward = false;
  moveBackward = false;
  turnLeft = false;
  turnRight = false;
}

/*
 * Ros subscriber for stopping the rover
 */
ros::Subscriber<std_msgs::Empty> movementStopRoverSubscriber(TOPIC_MOVEMENT_STOP_ROVER, &stopRover );

#endif
