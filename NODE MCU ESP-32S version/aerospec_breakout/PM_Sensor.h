#ifndef __PM_SENSOR_H__
#define __PM_SENSOR_H__

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PM25AQI.h>

namespace pmConsts{
  constexpr int PM_SDA {21};
  constexpr int PM_SCL {22};
  constexpr int PM_RST {4};
  constexpr int PM_SET {15};
  constexpr int PM_I2C_FREQ {100000};
}

bool setUpPMSensor();

PM25_AQI_Data pm25();

void reset();
#endif
