/*
 * Ros Initialisation file to initialise all the ros components and include all the packages to run the ros commands
 */
 
#ifndef rosInit_h
#define rosInit_h

#include "rosLcdCallback.h"


ros::NodeHandle rosNodeHandle;

void rosSetup()
{ 
  rosNodeHandle.initNode();
  rosNodeHandle.subscribe(lcdDisplaySubscriber);
}

void rosLoop()
{  
  rosNodeHandle.spinOnce();
  delay(1);
}

#endif
