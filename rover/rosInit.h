/*
 * Ros Initialisation file to initialise all the ros components and include all the packages to run the ros commands
 */
 
#ifndef rosInit_h
#define rosInit_h

#include "rosLcdCallback.h"
#include "rosMovementCallback.h"


//get the node handler instance
ros::NodeHandle rosNodeHandle;

void rosSetup()
{ 
  //initialises node
  rosNodeHandle.initNode();
  
  //subscribes to the topic being published
  rosNodeHandle.subscribe(lcdDisplaySubscriber);
  rosNodeHandle.subscribe(lcdClearDisplaySubscriber);

  rosNodeHandle.subscribe(movementMoveForwardSubscriber);
  rosNodeHandle.subscribe(movementMoveBackwardSubscriber);
  rosNodeHandle.subscribe(movementTurnRoverLeftSubscriber);
  rosNodeHandle.subscribe(movementTurnRoverRightSubscriber);
  rosNodeHandle.subscribe(movementStopRoverSubscriber);
}

void rosLoop()
{ 
  //spin once handles all the callbacks and ros related features 
  rosNodeHandle.spinOnce();
  delay(1);
}

#endif
