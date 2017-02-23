/*
 * Ros Initialisation file to initialise all the ros components and include all the packages to run the ros commands
 */
 
#ifndef rosinit_h
#define rosinit_h

#include <ros.h>
#include <std_msgs/String.h>
#include "rosConstants.h"
#include "LiquidCrystal_I2C.h"

ros::NodeHandle rosNodeHandle;
LiquidCrystal_I2C* lcd = new LiquidCrystal_I2C(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

std_msgs::String str_msg;
char hello[13] = "hello world!";


ros::Publisher chatter("chatter", &str_msg);


void displayOnLcd(const std_msgs::String message){
  lcd->clear();
  lcd->setCursor(0,0);
  lcd->print(message.data);   // display on lcd

  str_msg.data = hello;
  chatter.publish( &str_msg );

  str_msg.data = message.data;
  chatter.publish( &str_msg );
}




ros::Subscriber<std_msgs::String> lcdSubscriber("topicDisplayOnLcd", &displayOnLcd );


void rosSetup()
{ 
  rosNodeHandle.initNode();
  rosNodeHandle.subscribe(lcdSubscriber);
  rosNodeHandle.advertise(chatter);
}

void rosLoop()
{  
  rosNodeHandle.spinOnce();
  delay(1);
}

#endif
