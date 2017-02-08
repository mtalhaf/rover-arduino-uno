/*
 This code runs the rover INSERT WHAT ROVER DOES.

 The code is supposed to run on the Arduino UNO and is built for
 SOFT561 module as a project.

 VERSION: v0.1.3
 */

#include "obstacleDetection.h"
#include "motor.h"
#include "movement.h"
#include "ultrasonic.h"

/*
 * Sets up motor pins in the main setup function
 * goes through direction and brake pins of motor
 * A and B and initiates them as output pins to 
 * send signals to the motor.
 * 
 * The speed pin does not needs to be initiated as
 * its a dedicated output PWM pin.
 */

Motor motorA(motorA_dir_pin, motorA_brake_pin, motorA_speed_pin);
Motor motorB(motorB_dir_pin, motorB_brake_pin, motorB_speed_pin);
//Ultrasonic ultrasonic(ultrasonic_trigger_pin, ultrasonic_echo_pin);

//Movement movement;

void setUpMotorPins() {
  
  //pinMode(motorA_dir_pin, OUTPUT); //set motor A direction pin to output
  //pinMode(motorA_brake_pin, OUTPUT); //set motor A brake pin to output

  //pinMode(motorB_dir_pin, OUTPUT); //set motor B direction pin to output
  //pinMode(motorB_brake_pin, OUTPUT); //set motor B brake pin to output

  motorA.setUp(); // sets up motor A pins
  motorB.setUp(); // sets up motor B pins
}

/*
 * Sets up the pins for the ultrasonic rangefinder.
 * the trigger pin is used as an output and the echo
 * pin is to read the input from the rangefinder.
 */

void setUpUltraSonicRangeFinderPins() {
  //pinMode(ultrasonic_trigger_pin, OUTPUT); //set ultrasonic trigger pin to output
  // pinMode(ultrasonic_echo_pin, INPUT); //set ultrasonic echo pin to output

  ultrasonic.setUp();
  
}

/*
 * Sets up LCD for printing, prints out welcome message
 */

void setUpLCD() {
  
  lcd.begin(16,2);   // initialize the lcd for 16 chars 2 lines, turn on backlight

  lcd.setCursor(0,0); //Start at character 0 on line 0
  lcd.print("Hello!");
  lcd.setCursor(0,1);
  lcd.print("I am your rover");
  lcd.setCursor(0,0); //Start at character 0 on line 0
  
}

void setup() {

  setUpMotorPins(); //sets up all the motor pins
  setUpUltraSonicRangeFinderPins(); //sets up all ultra sonic pins
  setUpLCD(); //sets up the LCD
  
}

/*
 * moves the rover around the room and changes direction if an obstacle is hit
 */
void moveRoverAround(){

  movement.moveForward(ROVER_SPEED);
  
  if (detectObstacles()) //if obstacles are detected in front of the rover
    avoidObstacle(); //avoid the obstacles
}

/*
 * moves the rover in all directions
 */

void moveInAllDirections(){
   // moves the robot forward
  movement.moveForward(ROVER_SPEED);
  delay(1000);
  
  movement.moveBackward(ROVER_SPEED);
  delay(1000);
  
  movement.turnLeftForward(ROVER_SPEED, 1000);

  movement.turnLeftBack(ROVER_SPEED, 1000);
  
  movement.turnRightForward(ROVER_SPEED, 1000);
  
  movement.turnRightBack(ROVER_SPEED, 1000);
  
  movement.stopRoverMotors();
}

void loop() {
  moveRoverAround();
}
