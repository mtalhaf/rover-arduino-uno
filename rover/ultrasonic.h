/*
 * Ultrasonic pin output and inputs are set here for easy access
 */

#include "constants.h"

 void echoOn(){
    digitalWrite(ultrasonic_trigger_pin, HIGH); //turns the echo on
 }

  void echoOff(){
    digitalWrite(ultrasonic_trigger_pin, LOW); //turns the echo off
 }

 void getEchoPulse(){
   duration = pulseIn(ultrasonic_echo_pin, HIGH);
 }

/*
 * Turns the echo on and off in order to
 * send ultrasonic waves and recieve them
*/

void echoUltraSonic(){

  //turns the ultrasonic off for 2 microseconds
  echoOff(); 
  delayMicroseconds(2); 

  //echos for 10 microseconds
  echoOn();
  delayMicroseconds(10); 

  //turns the ultrasonic off again
  echoOff();
  
}

/*
 * gets the distance in cm determined by the ultrasonic
 */

long getDistance(){

  long distance, duration; //initialises the duration and distance variable
  
  duration = getEchoPulse(); // get the duration it took for the ultrasonic waves to return
  distance = duration/58.2; // calculates the distance based on speed of the waves

  return distance; // returns the distance in cm
  
}

