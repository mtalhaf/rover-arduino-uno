/*
 * Init file to include all the Initialisation required by the main program
 */

#ifndef init_h
#define init_h


#include "constants.h"
#include "Motor.h"
#include "Movement.h"
#include "Ultrasonic.h"
#include "ObstacleDetection.h"
#include "lcd.h"


Motor* motorA;
Motor* motorB;
Ultrasonic* ultrasonic;
Movement* movement;
ObstacleDetection* obstacleDetection;



/*
 * Sets up all the required sensor objects
 */

void setUpSensorObjects(){
  motorA = new Motor(motorA_dir_pin, motorA_brake_pin, motorA_speed_pin);
  motorB = new Motor(motorB_dir_pin, motorB_brake_pin, motorB_speed_pin);
  ultrasonic = new Ultrasonic(ultrasonic_front_trigger_pin, ultrasonic_front_echo_pin);
}


/*
 * Sets up all the required functional objects
 */

void setUpFunctionalObjects(){
  movement = new Movement(motorA, motorB);
  obstacleDetection = new ObstacleDetection(movement, ultrasonic);
}

/*
 * Sets up motor pins in the main setup function
 * goes through direction and brake pins of motor
 * A and B and initiates them as output pins to 
 * send signals to the motor.
 * 
 * The speed pin does not needs to be initiated as
 * its a dedicated output PWM pin.
 */

void setUpMotorPins() {
  motorA->setUp(); // sets up motor A pins
  motorB->setUp(); // sets up motor B pins
}


/*
 * Sets up the pins for the ultrasonic rangefinder.
 * the trigger pin is used as an output and the echo
 * pin is to read the input from the rangefinder.
 */

void setUpUltraSonicRangeFinderPins() {
  ultrasonic->setUp();
}

/*
 * Sets up LCD for printing
 */

void setUpLCD() {
  
  lcd.begin(16,2);   // initialize the lcd for 16 chars 2 lines, turn on backlight
  lcd.setCursor(0,0); //Start at character 0 on line 0
}

void init(){
  setUpMotorPins(); //sets up all the motor pins
  setUpUltraSonicRangeFinderPins(); //sets up all ultra sonic pins
  setUpLCD(); //sets up the LCD
}
#endif
