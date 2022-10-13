/*
 * Project Online version
 * Description: WIFI is connected for sync RTC. The raw data will be store in SD card 
 * Author:Kai
 * Date: 10/12/2022
 */

#include "Adafruit_PM25AQI.h"
#include <BME280I2C.h>
#include "Adafruit_EPD.h"
#include "SdFat.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "RTClib.h"
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "wifiCredential.h"
#include <NTPClient.h>
#include <EEPROM.h>

//PRODUCT_ID(11334)
//PRODUCT_VERSION(3)


#define VERSION_R "V1.1.0"
#define NAME_ADDR 10
#define DUTY_SCR_ADDR 255
#define DUTY_SEN_ADDR 260
#define DUTY_PUB_ADDR 265
#define ID_BUFF_SIZE 20
#define SAMPLE_CHAR_SIZE 265
#define PM_RST A0
#define PM_SET A1
#define PP5V0_EN 13 //D8

#define SEALEVELPRESSURE_HPA (1013.25)
#define SD_CS 14 //D2
#define SRAM_CS 32 //D3
#define EPD_CS 15//D4
#define EPD_DC 33//D5
#define EPD_RESET -1 // can set to -1 and share with microcontroller Reset!
#define EPD_BUSY -1  // can set to -1 to not use a pin (will wait a fixed delay)
#define BAT_LEVEL A13
#define EEPROM_SIZE 20

#define GPSSerial Serial1
#define ORIGIN_X 5
#define ORIGIN_Y 5

#define GPSECHO true
#define OUTPUTFILE "/Data.txt"
#define ONE_DAY_MILLIS (24 * 60 * 60 * 1000)

#define WIFI_TIMEOUT_MS (10*1000)

//TIME ZONE functionality for RTC
#define TIME_ZONE -7
#define TIME_ZONE_SECONDS (60 * 60 * TIME_ZONE)

bool deviceConnected = false;
bool oldDeviceConnected = false;

void displayInit();

//FuelGauge fuel;
/* GPS Definitions  */
//Adafruit_GPS GPS(&GPSSerial);

BME280I2C bme;
/* Screen obj */
Adafruit_SSD1675 epd(250,122, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);
/* Particle Sensor obj */
Adafruit_PM25AQI pm = Adafruit_PM25AQI(); // create instance of class

RTC_DS3231 rtc;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", TIME_ZONE_SECONDS);


File myFile;


//SYSTEM_MODE(SEMI_AUTOMATIC);
//SYSTEM_THREAD(ENABLED);


/* Refresh rates */
unsigned long DISPLAY_REFRESH; 
unsigned long SENSOR_CYCLE;
unsigned long PUBLISH_RATE;

int N_SAMPLES;


unsigned long preSensorDutyMillis;
unsigned long predisplayDutyMillis;
unsigned long prePublishDutyMillis;
unsigned long preTimeCheckMillis;
unsigned long preConnectionTimeMillis;

bool SD_WRITE_SUCCESSFUL = false;
bool PUBLISH_FLAG = false;

/* Counter for repeat data counts on I2C line */
uint8_t error_i2c_stale = 0;
float pp03_error = 0;

/* Battery threshold value */
const float bat_threash = 3.2;

/* A struct to keep error and types together */
struct error_type{
  bool active;
  bool i2c_stale;
  bool bat_low;
};

error_type ERROR = {false,false,false};
/*
struct dataAverage{
  uint16_t pt_03;
  uint16_t pt_05;
  uint16_t pt_10;
  uint16_t pt_25;
  uint16_t pt_50;
  uint16_t pt_100;
  
  //Mass Concentration Counts - Environment
  uint16_t pm10_env;
  uint16_t pm25_env;
  uint16_t pm100_env;

  //Mass Concentration Counts - standard
  uint16_t pm10_st;
  uint16_t pm25_st;
  uint16_t pm100_st;

  float temp;
  float humidity;
  float pressure;
  float altitude;
};
dataAverage AVERAGE;
*/

struct Sample{
  uint16_t pt_03_;
  uint16_t pt_05_;
  uint16_t pt_10_;
  uint16_t pt_25_;
  uint16_t pt_50_;
  uint16_t pt_100_;
  
  //Mass Concentration Counts - Environment
  uint16_t pm10_env_;
  uint16_t pm25_env_;
  uint16_t pm100_env_;

  //Mass Concentration Counts - standard
  uint16_t pm10_st_;
  uint16_t pm25_st_;
  uint16_t pm100_st_;

  float temp_;
  float humidity_;
  float pressure_;
  float altitude_;

  int seconds_;
  int minute_;
  int hour_;
  int day_;
  int month_;
  int year_;

  float battery_;

};
Sample latest_sample;

String deviceID;
//flag to check if rtc has been synced with ntp yet, will check once
bool checkRTC = true;



// Particle cloud functions need to be declared
int renameID(String name);
int setDutyCycle(String duty);




void setup()
{ 
  // Sets these reset pins to HIGH during normal operation, also must turn on enable pins for GPS and PM sensor
 
  pinMode(PM_RST,OUTPUT);
  pinMode(PP5V0_EN,OUTPUT);


  digitalWrite(PM_RST,HIGH);
  digitalWrite(PP5V0_EN,HIGH);
  char ID[ID_BUFF_SIZE];


  
  const uint8_t val = 0x01;
  //dct_write_app_data(&val, DCT_SETUP_DONE_OFFSET, 1);
  

  Serial.begin(115200);
  SPIClass * vspi = new SPIClass(VSPI);
  vspi->begin();
  pinMode(SD_CS, OUTPUT);
  if(!SD.begin(SD_CS, *vspi)){
        Serial.println("Card Mount Failed");
        return;
  }
  delay(1000);  
  Wire.begin();
  delay(1000);

  
  if (!bme.begin()) {
    Serial.println("Environmental Sensor did not initialize");
    delay(1000);
  }
  pm.begin_I2C();
  epd.begin();
  SD.begin(SD_CS);
  



  /* 
    Checks to see if duty cycle was ever set. If not, EEPROM value will be empty
    and will set it to a default. If EEPROM is not empty, will grab those values
    and set them to the globals. 
   */
  uint16_t duty_check;
//  EEPROM.get(DUTY_PUB_ADDR,duty_check);
//  if(duty_check != 0xFFFF){
//    EEPROM.get(DUTY_SCR_ADDR,DISPLAY_REFRESH);
//    EEPROM.get(DUTY_SEN_ADDR,SENSOR_CYCLE);
//    EEPROM.get(DUTY_PUB_ADDR,PUBLISH_RATE);
//  }
//  else{
    DISPLAY_REFRESH = 10;
    SENSOR_CYCLE = 5;
    PUBLISH_RATE = 43200;
//    EEPROM.put(DUTY_SCR_ADDR,DISPLAY_REFRESH);
//    EEPROM.put(DUTY_SEN_ADDR,SENSOR_CYCLE);
//    EEPROM.put(DUTY_PUB_ADDR,PUBLISH_RATE);
//  }
  

  
  connectToWiFi();
  delay(200);
  syncRTC();
  Serial.print("ESP32 Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  byte mac[6]; 
  WiFi.macAddress(mac);
  Serial.println(mac[3]);
  sprintf(ID, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  /* 
    Looks at the address where name is stored. If there is no name stored,
    will attempt to run the cloud function to get the name. If it does get a name,
    the EEPROM value will be set. If the cloud is not connected, and there is no value set
    , it will put a "-" character in there instead. 
   */
  
  EEPROM.begin(512);
  delay(2000); 
  EEPROM.put(NAME_ADDR, ID);
  EEPROM.commit();
  if(ID[0] == 0xFFFF){
      strcpy(ID,"-");
      Serial.println("NO ID SET");
      //EEPROM.put(NAME_ADDR,ID);
  }
  
//  char testID[20];
//  EEPROM.get(NAME_ADDR,testID);
//  Serial.println("ID:");
//  Serial.println(testID);
  
  // Set global deviceID to a string converted ID. So we can grab from particle console
  // deviceID = String(ID);

  // Writes header to text file on SD card if the file doesn't exist. 
  if (!SD.exists(OUTPUTFILE)) {
    char label[260];
    sprintf(label, "%10s,%10s,%10s,%5s,%15s,%15s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s",
    "Date","Time", "Battery", "Fix","Latitude","Longitude","Dp>0.3","Dp>0.5","Dp>1.0","Dp>2.5","Dp>5.0","Dp>10.0",
    "PM1_Std","PM2.5_Std","PM10_Std","PM1_Env","PM2.5_Env","PM10_Env","Temp(C)","RH(%)","P(hPa)","Alti(m)");
    writeToFile(OUTPUTFILE,ID);
    delay(10);
    writeToFile(OUTPUTFILE, label);
  }
  
  // Splash Screen
  displayInit();  
  delay(1000);

}



void loop()
{

  
  if (!preSensorDutyMillis || !predisplayDutyMillis || !prePublishDutyMillis || !preTimeCheckMillis)
  {
    preSensorDutyMillis = millis();
    predisplayDutyMillis = millis();
    prePublishDutyMillis = millis();
    preTimeCheckMillis = millis();
    preConnectionTimeMillis = millis();
  }

  unsigned long curSensorDutyMillis = millis();
  unsigned long curDisplayDutyMillis = millis();
  unsigned long curPublishDutyMillis = millis();
  unsigned long curTimeCheckMillis = millis();
  unsigned long curConnectionTimeMillis = millis();



  if (curSensorDutyMillis - preSensorDutyMillis >= SENSOR_CYCLE*1000)
  {
    /*
    //Serial.println("Sensor duty entered");
    // Checks if there are any errors. 
    //   Currently if there is an i2c issue,
    //   device resets. Waits on flag from
    //   a error struct, set by various error
    //   checkers
    
    if(ERROR.active){
      if(ERROR.i2c_stale){
        error_display_I2C();
        delay(10000);

        // GPS/PMSA RESET
        //digitalWrite(GPS_RST,LOW);
        digitalWrite(PM_RST,LOW);
        delay(10);
        //digitalWrite(GPS_RST,HIGH);
        digitalWrite(PM_RST,HIGH);
        delay(10);
        //System.reset();
      }
      if(ERROR.bat_low){
        error_display_BAT();
        delay(10000);
        // Sleep for 5 days. Will just reset and go back to sleep if not charged. 
        //System.sleep(SLEEP_MODE_DEEP, 432000);
      }
    }
    
    // Grab PM data from I2C bus, store in buffer
    SD_WRITE_SUCCESSFUL = false;
    */

    take_sample();
    char sample_data[SAMPLE_CHAR_SIZE];
    sample_write(&latest_sample,sample_data);
    writeToFile(OUTPUTFILE, sample_data);
    
    // If past duty time, reset timer
    preSensorDutyMillis = curSensorDutyMillis;
  }

  // Task for displaying. SENSOR_CYCLE should be set either way. 
  if( (curDisplayDutyMillis - predisplayDutyMillis)  >= DISPLAY_REFRESH*1000){
    take_sample();
    displayDATA();
    predisplayDutyMillis = curDisplayDutyMillis;
  }  
}



void sample_write(Sample* sample, char *output){

  char timeData[20], pmData[150], posLatData[2], posLonData[2], fixData[2], dateData[20], batData[10], tempData[10], rhData[10], altData[10], pressData[10];

  writePM(sample,pmData);
  sprintf(tempData, "%.2f",sample->temp_);
  sprintf(rhData, "%.2f",sample->humidity_);
  sprintf(pressData,"%.2f",sample->pressure_);
  sprintf(altData,"%.2f",sample->altitude_);
  sprintf(timeData, "%u:%u:%u", sample->hour_, sample->minute_, sample->seconds_); // String for Time
  sprintf(dateData, "%u/%u/%u", sample->year_, sample->month_, sample->day_);     // String for date
  sprintf(posLatData,"%u",0); // no GPS set to default 0
  sprintf(posLonData,"%u",0);  // no GPS set to default 0
  sprintf(fixData,"%u",0);  // no GPS set to default 0
  sprintf(batData,"%.2f", sample->battery_);
  
  sprintf(output, "%10s,%10s,%10s,%5s,%15s,%15s,%10s,%10s,%5s,%5s,%5s", dateData, timeData, batData, fixData, posLatData, posLonData, pmData, tempData, rhData, pressData, altData);

}

void writePM(Sample* sample, char* pmData) {
  char p03[6]; char p05[6]; char p10[6]; char p25[6]; char p50[6]; char p100[6];
  char pm10_std[6]; char pm25_std[6]; char pm100_std[6];
  char pm10_env[6]; char pm25_env[6]; char pm100_env[6];
  String(sample->pt_03_).toCharArray(p03, 6);
  String(sample->pt_05_).toCharArray(p05, 6);
  String(sample->pt_10_).toCharArray(p10, 6);
  String(sample->pt_25_).toCharArray(p25, 6);
  String(sample->pt_50_).toCharArray(p50, 6);
  String(sample->pt_100_).toCharArray(p100, 6);
  String(sample->pm10_st_).toCharArray(pm10_std, 6);
  String(sample->pm25_st_).toCharArray(pm25_std, 6);
  String(sample->pm100_st_).toCharArray(pm100_std, 6);
  String(sample->pm10_env_).toCharArray(pm10_env, 6);
  String(sample->pm25_env_).toCharArray(pm25_env, 6);
  String(sample->pm100_env_).toCharArray(pm100_env, 6);
  sprintf(pmData, "%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s", p03, p05, p10, p25, p50, p100, pm10_std, pm25_std, pm100_std, pm10_env, pm25_env, pm100_env);
}



/* 
 * Method handles writing to SD card with a given string, created by sdPrint.
*/
void writeToFile(String path, String content) {
  myFile = SD.open(path, FILE_APPEND);

  // if the file opened okay, write to it:
  if (myFile) {
    myFile.println(content);
    // close the file:
    myFile.close();

    // A flag to check if we wrote to the SD (or attempted to)
    SD_WRITE_SUCCESSFUL = true;

  } else {
    // if the file didn't open, print an error:
    Serial.println("File did not open");
    SD_WRITE_SUCCESSFUL = false;
  }
}

/*  
  ERROR CHECKING FUNCTIONS
*/

/* 
  Compares previous GPS data to current poll.
  Probability that data is exactly the same is very low, 
  indicating the buffer is reciving stale data and something 
  has locked up. Sets global error flag to true. 
  Resets counter to 0 when data is refreshed. 
*/
void repeat_i2c_check(float pp03_data){
  if (pp03_data == pp03_error)
  {
    error_i2c_stale++;
  }
  else{
    error_i2c_stale = 0;
  }
  
  if(error_i2c_stale > 60){
    ERROR.active = true;
    ERROR.i2c_stale = true;
  }
  pp03_error = pp03_data;
}


/*
 * Returns percentage of battery level
 */
float bat_read() {
  float batVolt = bat_volt();
  float percentage = ((batVolt - 3.0) * 100) / (1.2);
  Serial.println(percentage);
  if(percentage > 100) {
    percentage = 100;
  }
  else if(percentage < 0) {
    percentage = 0;
  }
  return percentage;
}

// Returns the voltage of the battery
float bat_volt() {
  float bat = analogRead(BAT_LEVEL);
  bat = (bat / 4095) * 2 * 3.3 * 1.1;
  return bat;
}



/*  
 * DISPLAY FUNCTIONS
 */
void displayInit(){
  epd.clearBuffer();
  epd.fillScreen(EPD_WHITE);
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
  epd.drawBitmap(95, 40, myBitmapBitmap, 40, 35, EPD_BLACK);
  char ID[20] = "-";
  EEPROM.get(NAME_ADDR,ID);
  // prints the text on welcome screen, just for testing so manually printed my device name and version
  epd.setTextSize(1);
  epd.setCursor(97, 80);
  epd.setTextColor(EPD_BLACK);
  epd.print(VERSION_R);
  epd.setCursor(73, 90);
  Serial.println(ID);
  epd.print("Device: ");
  epd.print(ID);
  epd.display();
}

/*
 * displayDATA
 */
void displayDATA()
{
  int mid_offset = 100;
//  CellularSignal sig = Cellular.RSSI();
//  float strength = sig.getStrength();

  char ID[ID_BUFF_SIZE];
  EEPROM.get(NAME_ADDR,ID);
  String IDs(ID);

  /*get time stamp from RTC*/
  DateTime now_time = rtc.now();

  int disHour = now_time.hour();  
  int disMinute  = now_time.minute(); 
  int disSecond = now_time.second();  
  int disDay = now_time.day();  
  int disMonth = now_time.month();  
  int disYear = now_time.year();


  epd.setTextWrap(true);
  epd.setTextSize(1);
  epd.clearBuffer();


  /* Prints Time */
  epd.setCursor(ORIGIN_X, 10);
  epd.setTextColor(EPD_BLACK);
  epd.print(ID);
  
  if(SD_WRITE_SUCCESSFUL){
    epd.print(" ++ ");
  }
  else{
    epd.print(" -- ");    
  }

  //epd.setCursor(ORIGIN_X+mid_offset, 10);
  epd.print(disMonth);
  epd.print("/");
  epd.print(disDay);
  epd.print("/");
  epd.print(disYear);
  epd.print("  ");
  epd.print(disHour);
  epd.print(":");
  epd.print(disMinute);
  epd.print(":");
  epd.print(disSecond);
  
  epd.setCursor(ORIGIN_X, 20);
  epd.print("Battery: ");
  epd.print(bat_read());
  epd.print("%  ");
  epd.setCursor(ORIGIN_X+mid_offset, 20);


  /* Prints PM Sensor Data */
  epd.setCursor(ORIGIN_X, 30);
  epd.setTextColor(EPD_BLACK);
  epd.print("Dp 0.3: ");
  epd.print(latest_sample.pt_03_);
  epd.setCursor(ORIGIN_X+mid_offset, 30);
  epd.print("E PM 1.0: ");
  epd.print(latest_sample.pm10_env_);

  epd.setCursor(ORIGIN_X, 40);
  epd.print("Dp 0.5: ");
  epd.print(latest_sample.pt_05_);
  epd.setCursor(ORIGIN_X+mid_offset, 40);
  epd.print("E PM 2.5: ");
  epd.print(latest_sample.pm25_env_);

  epd.setCursor(ORIGIN_X, 50);
  epd.print("Dp 1.0: ");
  epd.print(latest_sample.pt_10_);
  epd.setCursor(ORIGIN_X+mid_offset, 50);
  epd.print("E PM 10 : ");
  epd.print(latest_sample.pm100_env_);

  epd.setCursor(ORIGIN_X, 60);
  epd.print("Dp 2.5: ");
  epd.print(latest_sample.pt_25_);
  epd.setCursor(ORIGIN_X+mid_offset, 60);
  epd.print("S PM 1.0: ");
  epd.print(latest_sample.pm10_st_);

  epd.setCursor(ORIGIN_X, 70);
  epd.print("Dp 5.0: ");
  epd.print(latest_sample.pt_50_);
  epd.setCursor(ORIGIN_X+mid_offset, 70);
  epd.print("S PM 2.5: ");
  epd.print(latest_sample.pm25_st_);

  epd.setCursor(ORIGIN_X, 80);
  epd.print("Dp10.0: ");
  epd.print(latest_sample.pt_100_);
  epd.setCursor(ORIGIN_X+mid_offset, 80);
  epd.print("S PM 10 : ");
  epd.print(latest_sample.pm100_st_);

  /* Prints GPS data */
  epd.setCursor(ORIGIN_X, 90);
  epd.print("Temp: ");
  epd.print(latest_sample.temp_);
  epd.print("C  RH: ");
  epd.print((int)latest_sample.humidity_);
  epd.print("%  P: ");
  epd.print(latest_sample.pressure_);
  epd.print("kPa");

  epd.setCursor(ORIGIN_X, 100);
  epd.print("Samp: ");
  epd.print(DISPLAY_REFRESH);
  epd.print("s    Disp: ");
  epd.print(SENSOR_CYCLE);
  epd.print("s    Pub: ");
  //epd.print(PUBLISH_RATE/3600);
  //epd.print("h");


  epd.display();
}


/*
 * error_display_I2C
 */
void error_display_I2C(){
  epd.setTextWrap(true);
  epd.setTextSize(1);
  epd.clearBuffer();
  epd.setCursor(10, 10);
  epd.setTextColor(EPD_BLACK);
  epd.print("ERROR");
  epd.setCursor(10, 60);
  epd.setTextColor(EPD_BLACK);
  epd.print("I2C Stale:");
  epd.print("Soft reset in:");
  epd.print(" 10s");
  epd.display();
}

/*
 * error_display_BAT
 */
void error_display_BAT(){
  epd.setTextWrap(true);
  epd.setTextSize(1);
  epd.clearBuffer();
  epd.setCursor(10, 10);
  epd.setTextColor(EPD_BLACK);
  epd.print("ERROR");
  epd.setCursor(10, 60);
  epd.setTextColor(EPD_BLACK);
  epd.print("Battery Low:");
  epd.print("Sleep for 5 days in:");
  epd.print(" 10s");
  epd.display();
}


/* 
 *   Renaming cloud function. handler() is required to parse out the 
 *   returned value from the cloud and store it in eeprom. 
 */
int renameID(String name) {
    char ID[ID_BUFF_SIZE];
    name.toCharArray(ID,ID_BUFF_SIZE);
    EEPROM.put(NAME_ADDR,ID);
    EEPROM.commit();
    deviceID = name;
    return 0;
}


/*
 * take_sample
 */
void take_sample(){
  PM25_AQI_Data data;
  if (! pm.read(&data)) {
    Serial.println("Could not read from AQI");
    return;
  }
  DateTime now_time = rtc.now();
  
  latest_sample.hour_ = now_time.hour();  
  latest_sample.minute_  = now_time.minute(); 
  latest_sample.seconds_ = now_time.second();  
  latest_sample.day_ = now_time.day();  
  latest_sample.month_ = now_time.month();  
  latest_sample.year_ = now_time.year();

  
  latest_sample.pt_03_ = data.particles_03um;
  latest_sample.pt_05_ = data.particles_05um;
  latest_sample.pt_10_ = data.particles_10um;
  latest_sample.pt_25_ = data.particles_25um;
  latest_sample.pt_50_ = data.particles_50um;
  latest_sample.pt_100_ = data.particles_100um;
  latest_sample.pm10_env_ = data.pm10_env;
  latest_sample.pm25_env_ = data.pm25_env;
  latest_sample.pm100_env_ = data.pm100_env;
  latest_sample.pm10_st_ = data.pm10_standard;
  latest_sample.pm25_st_ = data.pm25_standard;
  latest_sample.pm100_st_ = data.pm100_standard;
  latest_sample.temp_ =  bme.temp();
  latest_sample.humidity_ =  bme.hum();
  latest_sample.pressure_ = bme.pres()/10;
  latest_sample.battery_ = bat_read();
}



/*
 * connectToWiFi
 */
void connectToWiFi(){
  Serial.println("Connecting to Wi-Fi");

  WiFi.mode(WIFI_STA);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD); // connect to Wifi

  unsigned long startAttemptTime = millis();

  // wait until wifi connected
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS){
    delay(500);
    Serial.print(".");
  }

  timeClient.begin();
  
  delay(1000);
  Serial.print("Connected, IP Address: ");  // print ip address
  Serial.println(WiFi.localIP());
  delay(100);
}

/*
 * syncRTC
 */
void syncRTC(){
  if(checkRTC && (WiFi.status() == WL_CONNECTED)){
    // Get time multiple times to avoid glitches
    for(int i = 0; i < 5; i++) {
      timeClient.update();
      Serial.println("Time Client Formatted Time");
      Serial.println(timeClient.getFormattedTime());
      delay(100);
    }
    rtc.adjust(DateTime((uint32_t)timeClient.getEpochTime()));
    
    checkRTC = false;
    DateTime now = rtc.now();
    Serial.println("now.unixtime:");
    Serial.println(now.unixtime());
    Serial.print(now.hour());
    Serial.print(":");
    Serial.println(now.minute());
    
    
  }
  else Serial.println("Particle Time Invalid/Incorrect");
}


// Draw our team logo in 80x70 size
// x is the x coordinate of left up corner of the logo
// y is the y coordinate of left up corner of the logo
// void draw_AsLogoL(uint16_t x, uint16_t y) {
//   epd.clearBuffer();
//   epd.fillScreen(EPD_WHITE);
//   const unsigned char myBitmapBitmap [] PROGMEM = {
//  // 'AeroSpec, 80x70px
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc0, 
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
//     0x00, 0x00, 0x03, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xe0, 0x00, 0x00, 
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
//     0x07, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x80, 0x00, 0x00, 0x00, 0x00, 
//     0x00, 0x00, 0x00, 0x00, 0x1f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
//     0x00, 0x00, 0x3e, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x06, 0x00, 0x00, 
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
//     0xf8, 0x0f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf8, 0x1f, 0x80, 0x00, 0x00, 0x00, 
//     0x00, 0x00, 0x00, 0x01, 0xf0, 0x1f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xe0, 0x3f, 
//     0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xe0, 0x7d, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 
//     0x00, 0x07, 0xc0, 0x7d, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xc0, 0xf0, 0xf0, 0x00, 
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x80, 0xf0, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 
//     0x81, 0xe0, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x01, 0xc0, 0x3c, 0x00, 0x00, 0x00, 
//     0x00, 0x00, 0x00, 0x3e, 0x03, 0xc0, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x07, 0x80, 
//     0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x07, 0x80, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 
//     0x00, 0x7c, 0x0f, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x0e, 0x00, 0x07, 0x00, 
//     0x00, 0x00, 0x00, 0x00, 0x01, 0xf8, 0x1e, 0x00, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf0, 
//     0x3c, 0x02, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x03, 0xe0, 0x3c, 0x06, 0x03, 0xc0, 0x00, 0x00, 
//     0x00, 0x00, 0x03, 0xe0, 0x78, 0x07, 0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x07, 0xc0, 0x78, 0x0f, 
//     0x81, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x07, 0xc0, 0xf0, 0x1f, 0x80, 0xf0, 0x00, 0x00, 0x00, 0x00, 
//     0x0f, 0x80, 0xe0, 0x1f, 0xc0, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x81, 0xe0, 0x3f, 0xc0, 0x78, 
//     0x00, 0x00, 0x00, 0x00, 0x1f, 0x03, 0xc0, 0x3f, 0xe0, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x03, 
//     0xc0, 0x7f, 0xf0, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x03, 0x80, 0x79, 0xf0, 0x1e, 0x00, 0x00, 
//     0x00, 0x00, 0x7c, 0x07, 0x80, 0xf9, 0xf8, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x0f, 0x01, 0xf0, 
//     0xf8, 0x0f, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x0f, 0x00, 0x00, 0x7c, 0x07, 0x00, 0x00, 0x00, 0x01, 
//     0xf8, 0x1e, 0x00, 0x00, 0x7c, 0x07, 0x80, 0x00, 0x00, 0x01, 0xf0, 0x1c, 0x00, 0x00, 0x3e, 0x03, 
//     0xc0, 0x00, 0x00, 0x03, 0xe0, 0x3c, 0x00, 0x00, 0x3e, 0x03, 0xc0, 0x00, 0x00, 0x03, 0xe0, 0x7c, 
//     0x00, 0x00, 0x1f, 0x01, 0xe0, 0x00, 0x00, 0x07, 0xc0, 0x7f, 0xff, 0xf0, 0x1f, 0x80, 0xe0, 0x00, 
//     0x00, 0x07, 0xc0, 0xff, 0xff, 0xf8, 0x0f, 0x80, 0xf0, 0x00, 0x00, 0x0f, 0x80, 0xff, 0xff, 0xfc, 
//     0x07, 0xc0, 0x78, 0x00, 0x00, 0x1f, 0x81, 0xff, 0xff, 0xfc, 0x07, 0xc0, 0x78, 0x00, 0x00, 0x1f, 
//     0x00, 0x00, 0x00, 0x00, 0x03, 0xe0, 0x3c, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf0, 
//     0x3c, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf0, 0x1e, 0x00, 0x00, 0x7c, 0x00, 0x00, 
//     0x00, 0x00, 0x01, 0xf8, 0x0e, 0x00, 0x00, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x0f, 0x00, 
//     0x00, 0xfc, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xfc, 0x07, 0x80, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 
//     0xff, 0xfc, 0x07, 0x80, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x03, 0xc0, 0x03, 0xff, 
//     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x03, 0xc0, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
//     0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x00, 0x00, 
//     0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
//     0x00, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x1f, 0xff, 
//     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
//     0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
//   };

//   epd.drawBitmap(x, y, myBitmapBitmap, 80, 70, EPD_BLACK);
//   char ID[20] = "-";
//   EEPROM.get(NAME_ADDR,ID);
//   // prints the text on welcome screen, just for testing so manually printed my device name and version
//   epd.setTextSize(1);
//   // epd.setCursor(20, 80);
//   // epd.setTextColor(EPD_BLACK);
//   // epd.print("Device : Breakout-06    V 1.2.2");
//   epd.setCursor(10, 80);
//   epd.setTextColor(EPD_BLACK);
//   epd.print(VERSION_R);
//   epd.setTextSize(1);
//   epd.setCursor(10, 80);
//   epd.print("Device: ");
//   epd.print(ID);

//   epd.display();
// }

// Draw our team logo in 40x35 size
// x is the x coordinate of left up corner of the logo
// y is the y coordinate of left up corner of the logo
// void draw_AsLogoS(uint16_t x, uint16_t y) {
//   epd.clearBuffer();
//   epd.fillScreen(EPD_WHITE);
//   const unsigned char myBitmapBitmap [] PROGMEM = {
//  // 'AeroSpec, 40x35px
//  0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 
//  0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 
//  0xe3, 0x00, 0x00, 0x00, 0x00, 0xc3, 0x00, 0x00, 0x00, 0x01, 0xc7, 0x80, 0x00, 0x00, 0x01, 0x86, 
//  0x80, 0x00, 0x00, 0x03, 0x8c, 0xc0, 0x00, 0x00, 0x03, 0x08, 0x60, 0x00, 0x00, 0x07, 0x18, 0x60, 
//  0x00, 0x00, 0x0e, 0x30, 0x30, 0x00, 0x00, 0x0c, 0x20, 0x10, 0x00, 0x00, 0x1c, 0x61, 0x18, 0x00, 
//  0x00, 0x18, 0x43, 0x08, 0x00, 0x00, 0x38, 0xc3, 0x8c, 0x00, 0x00, 0x31, 0x87, 0x86, 0x00, 0x00, 
//  0x71, 0x8f, 0xc6, 0x00, 0x00, 0xe3, 0x0c, 0xc3, 0x00, 0x00, 0xc3, 0x00, 0xe1, 0x00, 0x01, 0xc6, 
//  0x00, 0x71, 0x80, 0x01, 0x86, 0x00, 0x30, 0x80, 0x03, 0x8f, 0xfe, 0x38, 0xc0, 0x03, 0x00, 0x00, 
//  0x18, 0x60, 0x07, 0x00, 0x00, 0x1c, 0x60, 0x0e, 0x00, 0x00, 0x0c, 0x30, 0x0f, 0xff, 0xff, 0xfe, 
//  0x10, 0x1f, 0xff, 0xff, 0xff, 0x18, 0x10, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x0c, 
//  0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x06, 0x7f, 0xff, 0xff, 0xff, 0xff
//   };
//   epd.drawBitmap(x, y, myBitmapBitmap, 40, 35, EPD_BLACK);
//   EEPROM.get(NAME_ADDR,ID);
//   // prints the text on welcome screen, just for testing so manually printed my device name and version
//   epd.setTextSize(1);
//   epd.setCursor(82, 80);
//   epd.setTextColor(EPD_BLACK);
//   epd.print(VERSION_R);
//   epd.setCursor(48, 90);
//   epd.print("Device: ");
//   epd.print(ID);
//   epd.display();
// }
