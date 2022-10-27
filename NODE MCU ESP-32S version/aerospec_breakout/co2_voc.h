#ifndef CO2_H
#define CO2_H

#include "Adafruit_CCS811.h"
#include <Wire.h>
#define VOC_INT 17

// init the ccs811 module, returns true if successful
bool co2_init();

// returns a pair containing the co2 then voc value
std::pair<float, float> getCo2Voc();

#endif
