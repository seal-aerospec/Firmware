#include "PM_Sensor.h"
#include "awsCredentials.h"
#include "wifiCredential.h"
#include "battery.h"
#include "co2_voc.h"
#include "oled.h"
#include "rtc.h"
#include "sdCard.h"
#include "tmpHumiditySensor.h"

#include <EEPROM.h>
#include <WiFiClientSecure.h>
#include <MQTTClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"

#define MAIN_LOOP 5000  // how long between sensor reads
#define OUTPUTFILE "/Data.txt" // file sensor data is logged to
#define ID_BUFF_SIZE 20
#define NAME_ADDR 10
#define LOGO_WIDTH 40
#define LOGO_HEIGHT 35

// Aerospec Logo bitmap
const unsigned char myBitmapBitmap [] PROGMEM = {
// 'AeroSpec, 40x35px
0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 
0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 
0xe3, 0x00, 0x00, 0x00, 0x00, 0xc3, 0x00, 0x00, 0x00, 0x01, 0xc7, 0x80, 0x00, 0x00, 0x01, 0x86, 
0x80, 0x00, 0x00, 0x03, 0x8c, 0xc0, 0x00, 0x00, 0x03, 0x08, 0x60, 0x00, 0x00, 0x07, 0x18, 0x60, 
0x00, 0x00, 0x0e, 0x30, 0x30, 0x00, 0x00, 0x0c, 0x20, 0x10, 0x00, 0x00, 0x1c, 0x61, 0x18, 0x00, 
0x00, 0x18, 0x43, 0x08, 0x00, 0x00, 0x38, 0xc3, 0x8c, 0x00, 0x00, 0x31, 0x87, 0x86, 0x00, 0x00, 
0x71, 0x8f, 0xc6, 0x00, 0x00, 0xe3, 0x0c, 0xc3, 0x00, 0x00, 0xc3, 0x00, 0xe1, 0x00, 0x01, 0xc6, 
0x00, 0x71, 0x80, 0x01, 0x86, 0x00, 0x30, 0x80, 0x03, 0x8f, 0xfe, 0x38, 0xc0, 0x03, 0x00, 0x00, 
0x18, 0x60, 0x07, 0x00, 0x00, 0x1c, 0x60, 0x0e, 0x00, 0x00, 0x0c, 0x30, 0x0f, 0xff, 0xff, 0xfe, 
0x10, 0x1f, 0xff, 0xff, 0xff, 0x18, 0x10, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x0c, 
0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x06, 0x7f, 0xff, 0xff, 0xff, 0xff
 };

// error variables
bool ERROR_OLED = false;
bool ERROR_WIFI = false;
bool ERROR_CO2 = false;
bool ERROR_PM = false;
bool ERROR_RTC = false;
bool ERROR_ENV = false;
bool ERROR_SD = false;
bool ERROR_BATT = false;

// For SPI
SPIClass * vspi = new SPIClass(VSPI);

// oled display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
vspi, OLED_DC, OLED_RESET, OLED_CS);

// The MQTT topics that this device should publish/subscribe
#define AWS_IOT_PUBLISH_TOPIC   "esp32/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"

WiFiClientSecure net = WiFiClientSecure();
MQTTClient client = MQTTClient(256);

char ID[ID_BUFF_SIZE];

void setup() {
    Serial.begin(115200);
    
    //initDeviceID(ID);
    //vspi->begin(OLED_CLK,19, OLED_MOSI, VSPI_SS); // Start SPI Bus
    vspi->begin();
    // DISPLAY INIT
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    
    if(!display.begin(SSD1306_SWITCHCAPVCC)) {
        Serial.println(F("SSD1306 allocation failed"));
        ERROR_OLED = true;
    }
    else {
        //display.display();
        delay(1000);
        display.clearDisplay();
        display.setTextSize(1);      // Normal 1:1 pixel scale
        display.setTextColor(SSD1306_WHITE); // Draw white text
        display.setCursor(0, 0);     // Start at top-left corner
        display.cp437(true);         // Use full 256 char 'Code Page 437' font
        display.drawBitmap(
        (display.width()  - 20 ) / 2,
        (display.height() - 28) / 2,
        myBitmapBitmap, LOGO_WIDTH, LOGO_HEIGHT, 1);
        display.display();
    }
    /*
    // WIFI INIT
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(1000);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD); // connect to Wifi
    Serial.println("Connecting to Wi-Fi");
    // wait until wifi connected
    for (int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++) {
      delay(1000);
    }
    if (WiFi.status() != WL_CONNECTED) {
      ERROR_WIFI = true;
    }
    delay(1000);
    Serial.print("Connected, IP Address: ");  // print ip address
    Serial.println(WiFi.localIP());
    */
    delay(1000); // delay to make sure everything is initialized
    
    // AWS INIT
    // Not fully functional yet
    // awsConnect();
    
    // VOC_CO2 INIT
    ERROR_CO2 = !co2_init();
    delay(1000);
    
    // PM SENSOR INIT
    ERROR_PM = !setUpPMSensor();
    delay(1000);
    
    // RTC INIT
    ERROR_RTC = !initRtc();
    delay(1000);
    
    // ENV INIT
    ERROR_ENV = !setUpEnvSensor();
    delay(1000);
    
    // SD CARD INIT
    // Choose chip select pin
    pinMode(VSPI_SS, OUTPUT); //VSPI SS
    // initialize SD Card
    if(!SD.begin(VSPI_SS,*vspi)){
        Serial.println("Card Mount Failed");
        ERROR_SD = true;
    }
    delay(1000);
    // Writes header to text file on SD card if the file doesn't exist. 
    if (!SD.exists(OUTPUTFILE)) {
      char label[260];
      sprintf(label, "%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s",
      "Date","Time", "Battery","Dp>0.3","Dp>0.5","Dp>1.0","Dp>2.5","Dp>5.0","Dp>10.0",
      "PM1_Std","PM2.5_Std","PM10_Std","PM1_Env","PM2.5_Env","PM10_Env","Temp(C)","RH(%)","CO2(ppm)","TVOC(ppb)");
      appendFile(SD, OUTPUTFILE,ID);
      delay(10);
      appendFile(SD, OUTPUTFILE, label);
      Serial.println("Writes header to text file on SD card if the file doesn't exist. ");
    }
    
    // if error, then break program
    if(errorCheck()) {
      while(1) {
        delay(1000);  
      }
    }
    
    Serial.println("done");
}


void loop() {
  //delay(5000);
  // if error, then break program
  if(errorCheck()) {
    while(1) {
      delay(10);  
    }
  }
  
  //CO2,TVOC READ
  Serial.println("CO2 Reading");
  std::pair<float, float> ccsRead = getCo2Voc();
  
  
  //PM2.5 READ
  char pmData[150];
  PM25_AQI_Data data = readPM(pmData);
  
  //RTC READ
  Serial.println("RTC Reading");
  DateTime now = getDateTime();
  
  //BATTERY READ
  Serial.print("Battery Level: ");
  float batLevel = batteryLevel();
  Serial.println("");
  
  
  //ENV(SHT31) READ
  Serial.println("Env Reading");
  Serial.print("Temp = ");
  float temp = readTemperature();
  Serial.print(temp);
  Serial.println(" C");
  Serial.print("Humidity = ");
  float humidity = readHumidity();
  Serial.print(humidity);
  Serial.println(" %");
  Serial.println("\n");
  
  // update display
  //float batLevel = 30;
  displayPrint(now, temp, humidity, data, batLevel, ccsRead);
  
  // write to sd card
  sdPrint(now, temp, humidity, pmData, batLevel, ccsRead);
  //publishMessage(now,temp, humidity, pmData, batLevel, ccsRead);
  client.loop();
  delay(MAIN_LOOP);
  /**/
    
}

// initializes device id from mac address
// takes a char* and saves ID to it and the EEPROM
char* initDeviceID(char* ID) {
    Serial.print("ESP32 Board MAC Address:  ");
    Serial.println(WiFi.macAddress());
    byte mac[6]; 
    WiFi.macAddress(mac);
    sprintf(ID, "%2X%2X%2X%2X%2X%2X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);    
    EEPROM.begin(512);
    delay(2000); 
    EEPROM.put(NAME_ADDR, ID);
    EEPROM.commit();
}

// Saves the ID to a user inputted char array
// Currently not needed, as ID is saved as a global variable
char* getDeviceID(char* ID) {
  EEPROM.get(NAME_ADDR,ID);
}

// Checks all error flags and displays on screen if there is an error
// Returns true if any flags are raised
bool errorCheck() {
  bool isError = false;
  if(ERROR_WIFI) {
    Serial.println("WIFI ERROR");
    display.clearDisplay();
    display.setCursor(0, 0); 
    display.write("ERROR 1: \nCould not connect to WiFi");
    display.display();
    isError = true;
  }
  if(ERROR_SD) {
    Serial.println("SD ERROR");
    display.clearDisplay();
    display.setCursor(0, 0); 
    display.write("ERROR 2: \nCould not find SD \ncard");
    display.display(); 
    isError = true;
  }
  if(ERROR_PM) {
    Serial.println("PM ERROR");
    display.clearDisplay();
    display.setCursor(0, 0); 
    display.write("ERROR 3: \nPM Sensor error");
    display.display(); 
    isError = true;
  }
  if(ERROR_RTC) {
    Serial.println("RTC ERROR");
    display.clearDisplay();
    display.setCursor(0, 0); 
    display.write("ERROR 4: \nRTC error");
    display.display(); 
    isError = true;
  }
  if(ERROR_ENV) {
    Serial.println("ENV ERROR");
    display.clearDisplay();
    display.setCursor(0, 0); 
    display.write("ERROR 5: \nEnvironmental Sensor error");
    display.display(); 
    isError = true;
  }
  if(ERROR_CO2) {
    Serial.println("CO2 ERROR");
    display.clearDisplay();
    display.setCursor(0, 0); 
    display.write("ERROR 6: \nC02/VOC sensor error");
    display.display(); 
    isError = true;
  }
  if(ERROR_BATT) {
    Serial.println("BAT ERROR");
    display.clearDisplay();
    display.setCursor(0, 0); 
    display.write("ERROR 7: \nBattery read error");
    display.display(); 
    isError = true;
  }
  if(ERROR_OLED) {
    // No need to display error if display doesn't work
    Serial.println("OLED ERROR");
    isError = true;
  }
  if(isError) {
    Serial.println("Error Found");
  }
  else {
    Serial.println("No errors");
  }
  return isError;
}

// connects to aws, returns false if failed to connect
bool awsConnect() {
    // Configure WiFiClientSecure to use the AWS IoT device credentials
    net.setCACert(AWS_CERT_CA);
    net.setCertificate(AWS_CERT_CRT);
    net.setPrivateKey(AWS_CERT_PRIVATE);
    
    // Connect to the MQTT broker on the AWS endpoint we defined earlier
    client.begin(AWS_IOT_ENDPOINT, 8883, net);
    
    // Create a message handler  
    client.onMessage(messageHandler);
    
    Serial.println("Connecting to AWS IOT");

    int timeout = 20;
    int count = 0;
    
    // while not connected to aws
    while (!client.connect(THINGNAME) && count < timeout) {
      count++;
      if(WiFi.status()== WL_CONNECTED) Serial.println("Connected to WiFi, Connecting to AWS");
      delay(500);
    }
    
    if(!client.connected()){
      Serial.println("AWS IoT Timeout!");
      return false;
    }
    
    // Subscribe to a topic
    client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
    
    Serial.println("AWS IoT Connected!");
    return true;
}

// Publishes data to AWS
void publishMessage(DateTime now_time, float temp, float humidity, char pmData[], float batLevel, std::pair<float, float> ccsRead) {
  if(client.connected()){
    char timeData[20]; 
    char dateData[20]; char batData[10];
    char tempData[10], rhData[10];
    char co2Data[10];
    char tvocData[10];

    sprintf(timeData, "%u:%u:%u", now_time.hour(),now_time.minute(),now_time.second()); // String for Time
    sprintf(dateData, "%u/%u/%u", now_time.year(),now_time.month(),now_time.day()); // String for date
    sprintf(tempData, "%f", temp);
    sprintf(rhData, "%f", humidity);
    sprintf(batData,"%f", batLevel);
    sprintf(co2Data, "%d", (int)ccsRead.first);
    sprintf(tvocData, "%d", (int)ccsRead.second);
    
    StaticJsonDocument<300> doc;
    doc["DeviceID"] = 12;
    String timestamp = String(millis());
    doc["timestamp"] = timeData;
    doc["sensor_data"] = 12;
    char jsonBuffer[512];
    serializeJson(doc, jsonBuffer); // print to client
    Serial.println("Data published to AWS");
    client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
  }
  else {
    Serial.println("Disconnected from AWS Client");
  }
}

// print incoming messages to serial monitor
void messageHandler(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);

//  StaticJsonDocument<200> doc;
//  deserializeJson(doc, payload);
//  const char* message = doc["message"];
}


// Takes in sensor data and saves it to the sdcard on the output file
void sdPrint(DateTime now_time, float temp, float humidity, char pmData[], float batLevel, std::pair<float, float> ccsRead) {
    char data[270]; // should be same size or larger than sum of smaller buffers. should be dynamic later. 
    char timeData[20]; 
    char dateData[20]; char batData[10];
    char tempData[10], rhData[10];
    char co2Data[10];
    char tvocData[10];

    sprintf(timeData, "%u:%u:%u", now_time.hour(),now_time.minute(),now_time.second()); // String for Time
    sprintf(dateData, "%u/%u/%u", now_time.year(),now_time.month(),now_time.day()); // String for date
    sprintf(tempData, "%f", temp);
    sprintf(rhData, "%f", humidity);
    sprintf(batData,"%f", batLevel);
    sprintf(co2Data, "%d", (int)ccsRead.first);
    sprintf(tvocData, "%d", (int)ccsRead.second);
    sprintf(data, "%10s,%10s,%10s,%10s,%10s,%5s,%10s,%10s", dateData, timeData, batData, pmData, tempData, rhData, co2Data, tvocData);
    if(sdCardInserted()) {

      appendFile(SD, OUTPUTFILE, data);
    }
}

// Takes in sensor data and prints in to the display
void displayPrint(DateTime now, float temp, float humidity, PM25_AQI_Data data, float batLevel, std::pair<float, float>ccsRead) {
    char timeDisp[20];
    char tempDisp[15];
    char pmDisp[30];
    char batDisp[10];
    char co2Disp[20];
    char wifiDisp[20];
    
    display.clearDisplay();
    display.setCursor(0, 0); 
    Serial.println("here");
    if(WiFi.status() == WL_CONNECTED) {
      sprintf(wifiDisp, "WiFi: Connected");
    }
    else {
      sprintf(wifiDisp, "WiFi: Disconnected");
    }
    if(now.minute() < 10) {
      sprintf(timeDisp, "%d/%d %d:0%d", now.month(), now.day(), now.hour(), now.minute());
    }
    else {
      sprintf(timeDisp, "%d/%d %d:%d", now.month(), now.day(), now.hour(), now.minute());
    }
    sprintf(tempDisp, "%.2fC", temp);
    sprintf(pmDisp, "P>0.3um/0.1L air:%d", data.particles_03um);
    sprintf(co2Disp, "CO2:%d ppm", (int)ccsRead.first);
    sprintf(batDisp, "Bat:%d%%", (int)batLevel);

    // Write to display
    display.write(timeDisp);
    display.write("   ");
    display.write(batDisp);
    display.setCursor(0, 8);
    display.write(wifiDisp);
    display.setCursor(0, 16);
    display.write(pmDisp);
    display.setCursor(0, 24);
    display.write(co2Disp);
    display.write("  ");
    display.write(tempDisp);
    display.display(); 
}

// Reads the PM Sensor and saves it to a user inputted char array
PM25_AQI_Data readPM(char* pmData) {
    char p03[6]; char p05[6]; char p10[6]; char p25[6]; char p50[6]; char p100[6];
    char pm10_std[6]; char pm25_std[6]; char pm100_std[6];
    char pm10_env[6]; char pm25_env[6]; char pm100_env[6];
    PM25_AQI_Data data = pm25();
    // Serial Print
    Serial.println(F("---------------------------------------"));
    Serial.println(F("Concentration Units (standard)"));
    Serial.println(F("---------------------------------------"));
    Serial.print(F("PM 1.0: ")); Serial.print(data.pm10_standard);
    Serial.print(F("\t\tPM 2.5: ")); Serial.print(data.pm25_standard);
    Serial.print(F("\t\tPM 10: ")); Serial.println(data.pm100_standard);
    Serial.println(F("Concentration Units (environmental)"));
    Serial.println(F("---------------------------------------"));
    Serial.print(F("PM 1.0: ")); Serial.print(data.pm10_env);
    Serial.print(F("\t\tPM 2.5: ")); Serial.print(data.pm25_env);
    Serial.print(F("\t\tPM 10: ")); Serial.println(data.pm100_env);
    Serial.println(F("---------------------------------------"));
    Serial.print(F("Particles > 0.3um / 0.1L air:")); Serial.println(data.particles_03um);
    Serial.print(F("Particles > 0.5um / 0.1L air:")); Serial.println(data.particles_05um);
    Serial.print(F("Particles > 1.0um / 0.1L air:")); Serial.println(data.particles_10um);
    Serial.print(F("Particles > 2.5um / 0.1L air:")); Serial.println(data.particles_25um);
    Serial.print(F("Particles > 5.0um / 0.1L air:")); Serial.println(data.particles_50um);
    Serial.print(F("Particles > 10 um / 0.1L air:")); Serial.println(data.particles_100um);
    Serial.println(F("---------------------------------------"));
    Serial.println();

    // Save to pmData
    String(data.particles_03um).toCharArray(p03, 6);
    String(data.particles_05um).toCharArray(p05, 6);
    String(data.particles_10um).toCharArray(p10, 6);
    String(data.particles_25um).toCharArray(p25, 6);
    String(data.particles_50um).toCharArray(p50, 6);
    String(data.particles_100um).toCharArray(p100, 6);
    String(data.pm10_standard).toCharArray(pm10_std, 6);
    String(data.pm25_standard).toCharArray(pm25_std, 6);
    String(data.pm100_standard).toCharArray(pm100_std, 6);
    String(data.pm10_env).toCharArray(pm10_env, 6);
    String(data.pm25_env).toCharArray(pm25_env, 6);
    String(data.pm100_env).toCharArray(pm100_env, 6);
    sprintf(pmData, "%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s", 
            p03, p05, p10, p25, p50, p100, pm10_std, pm25_std, pm100_std, pm10_env, pm25_env, pm100_env);
    return data;
}
