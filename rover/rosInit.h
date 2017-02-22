/*
 * Ros Initialisation file to initialise all the ros components and include all the packages to run the ros commands
 */
 
#ifndef rosinit_h
#define rosinit_h

#include <ros.h>
#include <std_msgs/String.h>
#include "rosConstants.h"
#include "rosCallBacks.h"
#include "LiquidCrystal_I2C.h"

ros::NodeHandle rosNodeHandle;

void rosSetup()
{ 
  rosNodeHandle.initNode();
  rosNodeHandle.subscribe(lcdSubscriber);
}

void rosLoop()
{  
  rosNodeHandle.spinOnce();
  delay(1);
}

#endif
