# Particle-Board-to-ESP32
Particle board code that needs to be changed to work with ESP32
When uploading mvp-beta-edit to ESP32, ensure the Parition scheme setting under tools is set to Minimal SPIFFS.

# Folder titled Particle Board to ESP32
This folder contains a working platformIO port for the esp32. This should be compatible with a freshly installed platformIO plugin on any editor of your choice. This firmware is not ready for experimental use, still several bugs to work out for reliability.

#### Note: This file will not be changed. We are using it as a guide for the ESP32 code we will be writing. Anything to do with the GPS can be ignored.

Dependencies:
"Adafruit_PM25AQI.h": Adafruit @V 1.0.6
"BME280.h": Tyler Glenn @V 3.0.0
"Adafruit_EPD.h": Adafruit @V 4.3.3
"SdFat.h": Bill Greiman @V 2.0.5
"RTClib.h": Adafruit @V 1.12.5
"NTPClient":Fabrice Weingberg @V 3.2.0
