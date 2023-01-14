#include "tmpHumiditySensor.h"

Adafruit_SHT31 sht31 = Adafruit_SHT31();

bool setUpEnvSensor(){
  pinMode(consts::TMP_HUM_RST, OUTPUT);
  digitalWrite(consts::TMP_HUM_RST, HIGH);
  digitalWrite(consts::TMP_HUM_RST, HIGH);
  pinMode(consts::TMP_HUM_ALR, INPUT);

  //0x44 is default mem address for env sensor, 0x45 can alternatively be used
  if (!sht31.begin(0x44)) {
    Serial.println("Could not find temp and humidity sensor!");
    return false;
  }
  return true;
}

float readTemperature(void) {
  return sht31.readTemperature();
}

float readHumidity(void) {
  return sht31.readHumidity();
}

void altAdr() {
  pinMode(consts::TMP_HUM_ADR, OUTPUT);
  digitalWrite(consts::TMP_HUM_ADR, HIGH);
}

void hardwareRst() {
  digitalWrite(consts::TMP_HUM_RST, LOW);
  delay(10);
  digitalWrite(consts::TMP_HUM_RST, HIGH);
}
