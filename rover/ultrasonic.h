/*
 * Ultrasonic class definition
 */

#ifndef ultrasonic_h
#define ultrasonic_h

#include "Arduino.h"
#include "constants.h"
#include "movement.h"
#include "motor.h"

/*
 * Start of ultrasonic class
 */
class Ultrasonic{

  // private movement variables
  private:
    int ultrasonic_trigger_pin; // ultrasonic trigger pin
    int ultrasonic_echo_pin; // ultrasonic echo pin
    void echoOn(); // starts echo ultrasonic waves
    void echoOff(); // stops echo ultrasonic waves
    long getEchoPulse(); // reads the echo pulse from the ultrasonic sensor

  // public methods for movement
  public:
    Ultrasonic(int ultrasonicTriggerPin, int ultrasonicEchoPin); // initialises the ultrasonic sensor
    void setUp(); // sets Up the ultrasonic sensor
    void echoUltraSonic(); // echos the ultrasonic for a short time
    long getDistance(); // gets the distance from the object infront of the sensor

};
 
#endif

