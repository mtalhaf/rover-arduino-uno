/*
 * Ros Movement callbacks to handle movement topics
 */

#ifndef rosMovementCallback_h
#define rosMovementCallback_h

#include "rosCallbackChecks.h"
#include "rosConstants.h"
#include <ros.h>
#include <std_msgs/Empty.h>

 
/*
 * Moves rover forward
 */

void moveRoverForward(const std_msgs::Empty& message){
  moveBackward = false;
  moveForward = true;
  stopRoverMovement = false;
}

/*
 * Ros subscriber for moving the rover forward
 */
ros::Subscriber<std_msgs::Empty> movementMoveForwardSubscriber(TOPIC_MOVEMENT_MOVE_FORWARD, &moveRoverForward );

/*
 * Moves rover backward
 */

void moveRoverBackward(const std_msgs::Empty& message){
  moveBackward = true;
  moveForward = false;
  stopRoverMovement = false;
}

/*
 * Ros subscriber for moving the rover backward
 */
ros::Subscriber<std_msgs::Empty> movementMoveBackwardSubscriber(TOPIC_MOVEMENT_MOVE_BACKWARD, &moveRoverBackward );

/*
 * turns rover left
 */

void turnRoverLeft(const std_msgs::Empty& message){
  turnLeft = true;
  turnRight = false;
  stopRoverMovement = false;
}

/*
 * Ros subscriber for turning the rover left
 */
ros::Subscriber<std_msgs::Empty> movementTurnRoverLeftSubscriber(TOPIC_MOVEMENT_TURN_LEFT, &turnRoverLeft );

/*
 * Turns rover right
 */

void turnRoverRight(const std_msgs::Empty& message){
  turnLeft = false;
  turnRight = true;
  stopRoverMovement = false;
}

/*
 * Ros subscriber for turning the rover right
 */
ros::Subscriber<std_msgs::Empty> movementTurnRoverRightSubscriber(TOPIC_MOVEMENT_TURN_RIGHT, &turnRoverRight );

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
