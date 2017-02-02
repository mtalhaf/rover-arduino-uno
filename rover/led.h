/*
 * LED pin outputs are set here as easy functions
 */

#include "constants.h"

void turnLedOn(){
  digitalWrite(led_pin, HIGH);// turns the LED on
}

void turnLedOff(){
  digitalWrite(led_pin, LOW);// turns the LED off
}


