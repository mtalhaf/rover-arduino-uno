/*
 * Ros Lcd callbacks to handle LCD topics
 */

#ifndef rosLcdCallback_h
#define rosLcdCallback_h

#include "LiquidCrystal_I2C.h"
#include "Wire.h"
#include "rosConstants.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
 
LiquidCrystal_I2C* lcd = new LiquidCrystal_I2C(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);


/*
 * Displays message on LCD
 */

void displayOnLcd(const std_msgs::String message){
  lcd->clear();
  lcd->setCursor(0,0);
  lcd->print(message.data);   // display on lcd
}

/*
 * Ros subscriber for the lcd display
 */
ros::Subscriber<std_msgs::String> lcdDisplaySubscriber(TOPIC_LCD_PRINT_MESSAGE, &displayOnLcd );

/*
 * Clear displays on LCD
 */

void clearDisplayOnLcd(const std_msgs::Empty& message){
  lcd->clear();
}

/*
 * Ros subscriber for the lcd display
 */
ros::Subscriber<std_msgs::Empty> lcdClearDisplaySubscriber(TOPIC_MOVEMENT_STOP_ROVER, &clearDisplayOnLcd );

#endif

