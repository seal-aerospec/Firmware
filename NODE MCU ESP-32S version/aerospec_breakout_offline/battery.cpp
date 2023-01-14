#include "battery.h"

short pointer = 0;
char avgBat[10];

float batteryLevel() {
  pinMode(BAT_OUT, INPUT);
  float adcRead = analogRead(BAT_OUT);
  float espVoltage = (adcRead / 4095) * 3.3; // 4095 is 3.3V for esp32 adc
  float batVoltage = espVoltage / 0.7833; // convert to battery voltage, which uses voltage divided of ratio 0.783
  //do conversion from voltage to percentage
  //Serial.print("batVoltage");
  //Serial.println(batVoltage);
  float percentage = 123 - (123 / (pow(pow(batVoltage / 3.7, 80) + 1, 0.165)));
  //Serial.print("percentage");
  Serial.println(percentage);
  //return getAvgBat(percentage);
  return percentage;
}

float getAvgBat(float percentage) {
  avgBat[pointer] = percentage;
  if(pointer < 9) {
    pointer++;
  }
  else {
    pointer = 0;
  }
  int total = 0;
  int numMeasurements = 0;
  for(int i = 0; i < 10; i++) {
    if(avgBat[i] != 0) {
      total += avgBat[i];
      numMeasurements++;
    }
  }
  return (int)total/numMeasurements;
}
