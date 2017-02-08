/*
 * This file is used to initialise LCD for printing
 */

#ifndef lcd_h
#define lcd_h

#include "Wire.h"
#include "LiquidCrystal_I2C.h" // Library included seperately to control the LCD display

LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

#endif


