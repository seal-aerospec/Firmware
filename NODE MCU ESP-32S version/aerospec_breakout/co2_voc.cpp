 #include "co2_voc.h"

Adafruit_CCS811 ccs;

bool co2_init() {
  if(!ccs.begin()){
    Serial.println("Failed to start sensor! Please check your wiring.");
    return false;
  }
  return true;
}

std::pair<float, float> getCo2Voc() {
  std::pair<float, float> ccsRead;
  if(ccs.available()){
    if(!ccs.readData()){
      Serial.print("CO2: ");
      float co2 = ccs.geteCO2();
      ccsRead.first = co2;
      Serial.print(co2);
      Serial.print("ppm, TVOC: ");
      float voc = ccs.getTVOC();
      ccsRead.second = voc;
      Serial.println(voc);
      return ccsRead;
    }
    else{
      Serial.println("ERROR!");
    }
  }
  return ccsRead;
}
