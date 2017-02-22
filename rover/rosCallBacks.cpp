/*
 * ros callbacks class implementation
 */
 
#include "rosCallBacks.h"

/*
 * Start of ros callbacks class
 */

void displayMessageOnLCD(const std_msgs::String message){
  lcd->clear();
  lcd->setCursor(0,0);
  lcd->print(message);
}


