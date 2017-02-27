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
    uint8_t ultrasonic_trigger_pin; // ultrasonic trigger pin
    uint8_t ultrasonic_echo_pin; // ultrasonic echo pin
    void echoOn(); // starts echo ultrasonic waves
    void echoOff(); // stops echo ultrasonic waves
    int getEchoPulse(); // reads the echo pulse from the ultrasonic sensor

  // public methods for movement
  public:
    Ultrasonic(uint8_t ultrasonicTriggerPin, uint8_t ultrasonicEchoPin); // initialises the ultrasonic sensor
    void setUp(); // sets Up the ultrasonic sensor
    void echoUltraSonic(); // echos the ultrasonic for a short time
    int getDistance(); // gets the distance from the object infront of the sensor

};
 
#endif

