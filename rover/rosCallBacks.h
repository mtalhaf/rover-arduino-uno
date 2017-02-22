/*
 * ros callbacks class definition
 */

#ifndef roscallbacks_h
#define roscallbacks_h


#include <ros.h>
#include <std_msgs/String.h>
#include "rosConstants.h"
#include "LiquidCrystal_I2C.h" // Library included seperately to control the LCD display


/*
 * Start of ros callbacks class
 */
class RosCallbacks{

  // private variables
  private:
    LiquidCrystal_I2C* lcd; // lcd object

  // public ros callback methods
  public:
    RosCallbacks(LiquidCrystal_I2C* lcd); // initialises ros callbacks class
    void displayMessageOnLCD(const std_msgs::String message); // displays message on an LCD screen
    ros::Subscriber<std_msgs::String> lcdSubscriber(DISPLAY_ON_LCD, &displayMessageOnLCD );
};


#endif
