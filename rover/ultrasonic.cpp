/*
 * Ultrasonic class definition
 */

#include "ultrasonic.h"

/*
 * Start of ultrasonic class
 */

// Constructor implementation
Ultrasonic::Ultrasonic(uint8_t ultrasonicTriggerPin, uint8_t ultrasonicEchoPin){
  ultrasonic_trigger_pin = ultrasonicTriggerPin;
  ultrasonic_echo_pin = ultrasonicEchoPin;
}

void Ultrasonic::echoOn(){
  digitalWrite(ultrasonic_trigger_pin, HIGH); //turns the echo on
}

void Ultrasonic::echoOff(){
  digitalWrite(ultrasonic_trigger_pin, LOW); //turns the echo off
}

int Ultrasonic::getEchoPulse(){
  return pulseIn(ultrasonic_echo_pin, HIGH);
}


void Ultrasonic::setUp(){
  pinMode(ultrasonic_trigger_pin, OUTPUT); //set ultrasonic trigger pin to output
  pinMode(ultrasonic_echo_pin, INPUT); //set ultrasonic echo pin to output
}

/*
 * Turns the echo on and off in order to
 * send ultrasonic waves and recieve them
*/

void Ultrasonic::echoUltraSonic(){

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

int Ultrasonic::getDistance(){

  int distance, duration; //initialises the duration and distance variable
  
  duration = getEchoPulse(); // get the duration it took for the ultrasonic waves to return
  distance = duration/58.2; // calculates the distance based on speed of the waves

  return distance; // returns the distance in cm
  
}

