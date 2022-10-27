#include "PM_Sensor.h"

Adafruit_PM25AQI pmSensor = Adafruit_PM25AQI();
PM25_AQI_Data pmData;

bool setUpPMSensor(){
  
  pinMode(pmConsts::PM_RST, OUTPUT);
  digitalWrite(pmConsts::PM_RST, HIGH);
  delay(1000);
  if (! pmSensor.begin_I2C()) {      // connect to the sensor over I2C
    Serial.println("Could not find PM 2.5 sensor!");
    return false;
  }
  return true;
}

PM25_AQI_Data pm25(){
  if (!pmSensor.read(&pmData)) {
    Serial.println("Could not read from AQI");
    delay(500);  // try again in a bit!
  }
  Serial.println("AQI reading success");

  return pmData;
}

void reset() {
  digitalWrite(pmConsts::PM_RST, LOW);
  delay(10);
  digitalWrite(pmConsts::PM_RST, HIGH);
}
