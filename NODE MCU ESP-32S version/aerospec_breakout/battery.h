#ifndef BAT_H
#define BAT_H
#include <Arduino.h>
#include <math.h>
#define BAT_OUT 34
#define CHARGE 36 //VP
#define VSPI_SS  13


// Returns the avg of the past 10 readings
float batteryLevel();

// Takes the latest reading and puts it in an array of past 10 readings
// Returns the average of the array
float getAvgBat(float percentage);

#endif
