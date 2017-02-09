/*
 * Constants file to include all the constants required by the main program
 */

#ifndef constants_h
#define constants_h

#define motorA_dir_pin 12 //Direction pin number for motor A
#define motorA_speed_pin 3 //Speed pin number for motor A, the pin is analog PWM 
#define motorA_brake_pin 9 //Brake pin number for motor A

#define motorB_dir_pin 13 //Direction pin number for motor B
#define motorB_speed_pin 11 //Speed pin number for motor B, the pin is analog PWM
#define motorB_brake_pin 8 //Brake pin number for motor B

#define ultrasonic_front_trigger_pin 6 //Trigger pin for the ultra sonic range finder in fron of the rover
#define ultrasonic_front_echo_pin 7 //Echo pin for the ultra sonic range finder in fron of the rover

#define ultrasonic_front_edge_trigger_pin 4 //Trigger pin for the ultra sonic range finder in fron of the rover
#define ultrasonic_front_edge_echo_pin 5 //Echo pin for the ultra sonic range finder in fron of the rover

#define ROVER_FORWARD_DIRECTION 1 //rover forward direction
#define ROVER_BACKWARD_DIRECTION 2 //rover backward direction

#define ROVER_TURN_LEFT 1 //rover forward direction
#define ROVER_TURN_RIGHT 2 //rover backward direction

#define ROVER_SPEED 130 //rover speed settings

#define OBSTACLE_DETECTION_DISTANCE 25 //maximum distance between rover and an obstacle in cm
#define EDGE_DETECTION_DISTANCE 7 //maximum distance between rover and and obstacle in cm
#define EDGE_DETECTION_DISTANCE_THRESHOLD 4 //the threshold for maximum distance between rover and and obstacle in cm, this value is added and deducted from the EDGE_DETECTION_DISTANCE to create min, max values for edge detection.

#endif
