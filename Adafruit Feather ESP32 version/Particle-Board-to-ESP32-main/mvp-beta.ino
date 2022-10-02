/*
 * Project BetaP1_test
 * Description: Adds real time clock to beta master version for offline usage
 *              and removes GPS functionality; sets timezone manually, for some
 *              reason Time.zone(x) function does not work with the RTC
 * Author:
 * Date: 12/15/2020
 */

//#define ARDUINO 1
#include "pmsa_i2c.h"
#include "Adafruit_BME280_RK.h"
#include "pmsa_i2c.h"
#include "Adafruit_EPD.h"
#include "SdFat.h"
#include "Adafruit_GPS.h"
#include "Particle.h"
#include "dct.h"
#include "Adafruit_RTClib_RK.h"

PRODUCT_ID(11334)
PRODUCT_VERSION(3)


/*
 * Project betaP1_test
 * Description: Firmware for MVP Beta board
 * Author: Brenden Singh
 * Date: 08/5/20 latest -> Breakout_port
 * V1.1.0
 */

/* 
    Release for second round of hospital testing. Updated screen for monochrome usage, updated format. Added date format and fixed particle time
    check. 

    Now shows a check to whether the SD card was written to, "++" or "--""

 */



/* 
  This version is for the Beta V2. Basically, this code is a port from the Breakout version. Functionality should be essentially the same from a user standpoint, from V1.2.3 of the
  breakout verion. 

  This device now has a temperature/humidity sensor. It is also using a UART version of the GPS module, not the I2C style. PM sensor is still using the I2C port. 

  The display has been changed up on this version to reflect the new sensor information. Should be straight forward but the display now shows the Temp/Relative humidity on the screen. 
  I've also changed up the screen to show the device's cycle settings "screen refresh rate (s), SD sample time (s), publish rate (h)" so you can see what the device is doing. 


  Unfortunately, V2.0 of beta hardware has the GPS module soldered backwards. To prevent any damage to the module, I'm disabling the GPS and all communication 
  to the module until the problem is fixed on these boards. 
 */





/* 
  New features:
    - Add two functions draw_AsLogoL and draw_AsLogoS which draws the current Aerospec logo in black color
      Can be used on the starting screen. 
        - I've consolidated functions into the displayInit to draw the logo. Commented in code to prevent bloat in the bin file. (Singh)
    - Device now reports rolling average of samples before the publish. *n* Samples will be summed , where n = PUBLISH_RATE/SAMPLE_RATE. Then the summation gets 
      divided by *n* samples, giving the average. *n* can be grabbed using the particle console to verify the number of samples, even though it is automatically calculated.
      Each particle sample is averaged.  
    - Can now set the sample and screen refresh rate via particle console. Use "Cycles" cloud function. Syntax is: screenRefresRate (s), sampleRefreshRate (s)
      Example : So setting the cycle times to a 60 second screen refresh rate and 30 second sample rate, you would enter : 60,30 
      Not following this syntax will yield weird behaviors. If you do it correctly, it should return a 0. If it returns anything else, there was an error. 
  Changes:
    - Changed battery display to percentage. Also now shows the charging state of the device, shown after the battery percentage. 
    - Added signal strength to screen. 
    - Changed format of the PM data. Also changed layout of information. Device info is displayed in red at the top. 
    - Displayed time now shows PST. Recorded time is still in UTC. 
  Issues:
    - Charging issues still persists. 
    - Right now you must set the device ID and sample/refresh rates via the console. For some reason, it doesn't recongnize the default values set in EEPROM. 
      It might be fixed as of now but should pay attention to this. This only needs to be done once, then the device should have a value stored in EEPROM. 
    - Getting the Device ID from the particle console seems to be depreciated. Ignoring this feature for now, so you must change it yourself via the console every time.
      You can at least see what it is currently named to keep track of devices via the cloud variable. 
*/


#define VERSION_R "V1.1.0"
#define NAME_ADDR 10
#define DUTY_SCR_ADDR 255
#define DUTY_SEN_ADDR 260
#define DUTY_PUB_ADDR 265
#define ID_BUFF_SIZE 20

#define PM_RST A0
#define PM_SET A1
#define GPS_1PPS A2
#define GPS_RST A3
#define GPS_EN A4
#define GPS_WAKE_UP A5
#define PP5V0_EN D8


#define SEALEVELPRESSURE_HPA (1013.25)
#define SD_CS D2
#define SRAM_CS D3
#define EPD_CS D4
#define EPD_DC D5
#define EPD_RESET -1 // can set to -1 and share with microcontroller Reset!
#define EPD_BUSY -1  // can set to -1 to not use a pin (will wait a fixed delay)

#define GPSSerial Serial1

#define ORIGIN_X 5
#define ORIGIN_Y 5

#define GPSECHO true
#define OUTPUTFILE "Data.txt"
#define ONE_DAY_MILLIS (24 * 60 * 60 * 1000)

//TIME ZONE functionality for RTC
#define TIME_ZONE 12
#define TIME_ZONE_SECONDS (60 * 60 * TIME_ZONE)




FuelGauge fuel;
/* GPS Definitions  */
Adafruit_GPS GPS(&GPSSerial);

Adafruit_BME280 bme;
/* Screen obj */
Adafruit_SSD1675 epd(250,122, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);
/* Particle Sensor obj */
PMSA003 pm = PMSA003(); // create instance of class

RTC_DS3231 rtc;

SdFat SD;
File myFile;


SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);


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

String deviceID;



// Particle cloud functions need to be declared
int renameID(String name);
int setDutyCycle(String duty);


void setup()
{ 
  // Sets these reset pins to HIGH during normal operation, also must turn on enable pins for GPS and PM sensor
  pinMode(GPS_RST,OUTPUT);
  pinMode(PM_RST,OUTPUT);
  pinMode(GPS_EN,OUTPUT);
  pinMode(PP5V0_EN,OUTPUT);


  digitalWrite(GPS_RST,HIGH); // Active Low
  digitalWrite(PM_RST,HIGH);
  digitalWrite(GPS_EN,LOW); // We are disabling power to the device since the GPS module is soldered backwards. 
  digitalWrite(PP5V0_EN,HIGH);



  /* 
    Cloud functions and variables need to be defined as so BEFORE
    Particle.connect() is called. Otherwise they wont register. 
    This is due to us running in SYSTEM_THREAD(ENABLED), auto is disabled 
    so we need to push this manually. 
  
   */
  Particle.function("Rename Device", renameID);
  Particle.function("Cycles", setDutyCycle);
  Particle.variable("Display Refresh Rate",DISPLAY_REFRESH);
  Particle.variable("Sensor Sample Rate",SENSOR_CYCLE);
  Particle.variable("Publish Rate", PUBLISH_RATE);
  Particle.variable("Device ID",deviceID);
  Particle.variable("n-Samples per Publish",N_SAMPLES);


  /* Currently fixing listening issue, look at again later */
  Cellular.setActiveSim(INTERNAL_SIM);
  Cellular.clearCredentials();

  const uint8_t val = 0x01;
  dct_write_app_data(&val, DCT_SETUP_DONE_OFFSET, 1);
  Cellular.on();
  Particle.connect();
  /*  */

  

  Serial.begin(115200);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate


  delay(1000);
  SD.begin(SD_CS);
  delay(1000);
  GPS.begin(9600);
  bme.begin();
  pm.begin();
  epd.begin();

  /************doesn't work with RTC for whatever reason; time zone just added manually******/
  //Time.zone(12);
  /******************************************************************************************/

  // Wait for a connection to the internet
  delay(5000);



  /* 
    Looks at the address where name is stored. If there is no name stored,
    will attempt to run the cloud function to get the name. If it does get a name,
    the EEPROM value will be set. If the cloud is not connected, and there is no value set
    , it will put a "-" character in there instead. 
   */
  char ID[ID_BUFF_SIZE];
  EEPROM.get(NAME_ADDR,ID);
  if(ID[0] == 0xFFFF){
    if (Particle.connected()){
      renameID("rename");
    }
    else{
      strcpy(ID,"-");
      EEPROM.put(NAME_ADDR,ID);
    }
  }
  // Set global deviceID to a string converted ID. So we can grab from particle console
  deviceID = String(ID);


  /* 
    Checks to see if duty cycle was ever set. If not, EEPROM value will be empty
    and will set it to a default. If EEPROM is not empty, will grab those values
    and set them to the globals. 
   */
  uint16_t duty_check;
  EEPROM.get(DUTY_PUB_ADDR,duty_check);
  if(duty_check != 0xFFFF){
    EEPROM.get(DUTY_SCR_ADDR,DISPLAY_REFRESH);
    EEPROM.get(DUTY_SEN_ADDR,SENSOR_CYCLE);
    EEPROM.get(DUTY_PUB_ADDR,PUBLISH_RATE);
  }
  else{
    DISPLAY_REFRESH = 30;
    SENSOR_CYCLE = 30;
    PUBLISH_RATE = 43200;
    EEPROM.put(DUTY_SCR_ADDR,DISPLAY_REFRESH);
    EEPROM.put(DUTY_SEN_ADDR,SENSOR_CYCLE);
    EEPROM.put(DUTY_PUB_ADDR,PUBLISH_RATE);
  }
  
  N_SAMPLES = (int) PUBLISH_RATE/SENSOR_CYCLE;
  if(N_SAMPLES < 1){
    N_SAMPLES = 1;
  }

  // Writes header to text file on SD card if the file doesn't exist. 
  if (!SD.exists(OUTPUTFILE)) {
    char label[260];
    sprintf(label, "%10s,%10s,%10s,%5s,%15s,%15s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s",
    "Date","Time", "Battery", "Fix","Latitude","Longitude","Dp>0.3","Dp>0.5","Dp>1.0","Dp>2.5","Dp>5.0","Dp>10.0",
    "PM1_Std","PM2.5_Std","PM10_Std","PM1_Env","PM2.5_Env","PM10_Env","Temp(C)","RH(%)","P(hPa)","Alti(m)");
    writeToFile(OUTPUTFILE,ID);
    writeToFile(OUTPUTFILE, label);
  }

  // Splash Screen
  displayInit();  
  delay(5000);

/*Checks for internet connection on start up and if present syncs RTC on the minute*/
  while (Time.second() != 0){};
  if(Time.isValid()){	
    rtc.adjust(DateTime((uint32_t) Time.now() + TIME_ZONE_SECONDS));	
  }
}



void loop()
{

  /******************************************************/
  gps_READ(); // Must be called as frequently as possible, at least faster than the set read freqency. Must be placed in timer interupt in future.
  /******************************************************/

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


  /* 
      Detects whether the device is connected to the cloud. If not, wil attempt to connect to the cloud for 10 seconds. 
      After will stop trying to connect. 
   */ 

  if(curConnectionTimeMillis - preConnectionTimeMillis >= 3600 * 1000){
    if(!Particle.connected){
      Particle.connect();
      waitFor(Particle.connected, 10000);
    }
  }


  if (curTimeCheckMillis - Particle.timeSyncedLast() >= ONE_DAY_MILLIS && Particle.connected){
    Particle.syncTime();
    // Wait until the device receives time from Particle Device Cloud (or connection to Particle Device Cloud is lost)
    // Will wait 10 seconds for the sync to be done. Blocking. 
    waitFor(Particle.syncTimeDone,10000);
  }

  if (curSensorDutyMillis - preSensorDutyMillis >= SENSOR_CYCLE*1000)
  {
    //Serial.println("Sensor duty entered");
    /* Checks if there are any errors. 
       Currently if there is an i2c issue,
       device resets. Waits on flag from
       a error struct, set by various error
       checkers
     */
    if(ERROR.active){
      if(ERROR.i2c_stale){
        error_display_I2C();
        delay(10000);

        // GPS/PMSA RESET
        digitalWrite(GPS_RST,LOW);
        digitalWrite(PM_RST,LOW);
        delay(10);
        digitalWrite(GPS_RST,HIGH);
        digitalWrite(PM_RST,HIGH);
        delay(10);
        System.reset();
      }
      if(ERROR.bat_low){
        error_display_BAT();
        delay(10000);
        // Sleep for 5 days. Will just reset and go back to sleep if not charged. 
        System.sleep(SLEEP_MODE_DEEP, 432000);
      }

    }
    // Grab PM data from I2C bus, store in buffer

    SD_WRITE_SUCCESSFUL = false;
    sd_Print();    
    // If past duty time, reset timer
    preSensorDutyMillis = curSensorDutyMillis;
  }

  // Task for displaying. SENSOR_CYCLE should be set either way. 
  if( (curDisplayDutyMillis - predisplayDutyMillis)  >= DISPLAY_REFRESH*1000){
    displayDATA();
    predisplayDutyMillis = curDisplayDutyMillis;
  }

  if( (curPublishDutyMillis - prePublishDutyMillis)  >= PUBLISH_RATE*1000){
    PUBLISH_FLAG = true;
    sd_Print();
    prePublishDutyMillis = curPublishDutyMillis;
  }



}


/* 
    gps_READ must be called every cycle of the main loop in order to collect enough data worthwile. 
    Setting this to a slower call speed will result in more junk data. 
    Places last collected data from the NMEA sentence in a buffer. This buffer contains the latest data
    and is accessed via a parsed method, described later. 
 */

void gps_READ()
{
  
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO){
    if (c) 
      {Serial.print(c);}
  }
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
}

void sd_Print(){
  pm.poll();
  delay(10);

  update_average(); // updates a running average. May overflow if too many samples. 
  char data[270]; // should be same size or larger than sum of smaller buffers. should be dynamic later. 
  char timeData[20]; char pmData[150];
  char posLatData[10];char posLonData[10];
  char fixData[1]; char dateData[20]; char batData[10];
  char tempData[10], rhData[10], altData[10], pressData[10];

  if(!GPS.fix && Time.isValid()){
    sprintf(timeData, "%u:%u:%u", Time.hour(),Time.minute(),Time.second()); // String for Time
    sprintf(dateData, "%u/%u/%u", Time.year(),Time.month(),Time.day()); // String for date
  }
  else{
    sprintf(timeData, "%u:%u:%u", GPS.hour,GPS.minute,GPS.seconds); // String for Time
    sprintf(dateData, "%u/%u/%u", GPS.year,GPS.month,GPS.day); // String for date
  }

  sprintf(posLatData,"%f",GPS.latitudeDegrees); // String for latitude position, givein as degrees/minutes as received from the GPS (DDDMM.MMMM)
  sprintf(posLonData,"%f",GPS.longitudeDegrees); // String for longitude position, givein as degrees/minutes as received from the GPS (DDDMM.MMMM)
  sprintf(fixData,"%u",GPS.fixquality); // String for the fix quality. 0 is no fix, 1 is GPS sat, 2 is DGPS
  sprintf(batData,"%f", fuel.getVCell());
  sprintf(tempData, "%f",bme.readTemperature());
  sprintf(rhData, "%f",bme.readHumidity());
  sprintf(pressData,"%f",bme.readPressure());
  sprintf(altData,"%f",bme.readAltitude(SEALEVELPRESSURE_HPA));
  readParticle(pmData);
  sprintf(data, "%10s,%10s,%10s,%5s,%15s,%15s,%10s,%10s,%5s,%5s,%5s", dateData, timeData, batData, fixData, posLatData, posLonData, pmData, tempData, rhData, pressData, altData);
  

  /* 
      Publishes data to the cloud if flag is set true by timer. Will not write to SD card. Should only be called
      based upon PUBLISH RATE. Since we are now uploading an average, this will take updated average, divide by the amount of samples
      in the given time period, rewrite this new averaged sample to the pmData buffer, rewrite the data buffer with the new pmData
      publish, then reset the running average to 0. 
   */
  if (PUBLISH_FLAG == true)
  {
    N_SAMPLES = (int) PUBLISH_RATE/SENSOR_CYCLE;
    take_average();
    average_PM_write(pmData);
    sprintf(data, "%10s,%10s,%10s,%5s,%15s,%15s,%10s,%5s,%5s,%5s,%5s", dateData, timeData, batData, fixData, posLatData, posLonData, pmData, tempData, rhData, pressData, altData);

    if(Particle.connected){
      Particle.publish("sensor-device-to-db", data, PRIVATE);
      PUBLISH_FLAG = false;
    }
    reset_average();
    return;
  }
  repeat_i2c_check(pm.pt_03());
  writeToFile(OUTPUTFILE, data);
}


void readParticle(char* pmData) {
  char p03[6]; char p05[6]; char p10[6]; char p25[6]; char p50[6]; char p100[6];
  char pm10_std[6]; char pm25_std[6]; char pm100_std[6];
  char pm10_env[6]; char pm25_env[6]; char pm100_env[6];
  String(pm.pt_03()).toCharArray(p03, 6);
  String(pm.pt_05()).toCharArray(p05, 6);
  String(pm.pt_10()).toCharArray(p10, 6);
  String(pm.pt_25()).toCharArray(p25, 6);
  String(pm.pt_50()).toCharArray(p50, 6);
  String(pm.pt_100()).toCharArray(p100, 6);
  String(pm.pm10_st()).toCharArray(pm10_std, 6);
  String(pm.pm25_st()).toCharArray(pm25_std, 6);
  String(pm.pm100_st()).toCharArray(pm100_std, 6);
  String(pm.pm10_env()).toCharArray(pm10_env, 6);
  String(pm.pm25_env()).toCharArray(pm25_env, 6);
  String(pm.pm100_env()).toCharArray(pm100_env, 6);
  sprintf(pmData, "%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s", p03, p05, p10, p25, p50, p100, pm10_std, pm25_std, pm100_std, pm10_env, pm25_env, pm100_env);
}

void average_PM_write(char* pmData) {
  char p03[6]; char p05[6]; char p10[6]; char p25[6]; char p50[6]; char p100[6];
  char pm10_std[6]; char pm25_std[6]; char pm100_std[6];
  char pm10_env[6]; char pm25_env[6]; char pm100_env[6];
  String(AVERAGE.pt_03).toCharArray(p03, 6);
  String(AVERAGE.pt_05).toCharArray(p05, 6);
  String(AVERAGE.pt_10).toCharArray(p10, 6);
  String(AVERAGE.pt_25).toCharArray(p25, 6);
  String(AVERAGE.pt_50).toCharArray(p50, 6);
  String(AVERAGE.pt_100).toCharArray(p100, 6);
  String(AVERAGE.pm10_st).toCharArray(pm10_std, 6);
  String(AVERAGE.pm25_st).toCharArray(pm25_std, 6);
  String(AVERAGE.pm100_st).toCharArray(pm100_std, 6);
  String(AVERAGE.pm10_env).toCharArray(pm10_env, 6);
  String(AVERAGE.pm25_env).toCharArray(pm25_env, 6);
  String(AVERAGE.pm100_env).toCharArray(pm100_env, 6);
  sprintf(pmData, "%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s", p03, p05, p10, p25, p50, p100, pm10_std, pm25_std, pm100_std, pm10_env, pm25_env, pm100_env);
}





/* 
    Method handles writing to SD card with a given string, created by sdPrint.
*/

void writeToFile(String path, String content) {
  myFile = SD.open(path, FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    myFile.println(content);
    // close the file:
    myFile.close();

    // A flag to check if we wrote to the SD (or attempted to)
    SD_WRITE_SUCCESSFUL = true;

  } else {
    // if the file didn't open, print an error:
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
  Checks battery, sets error flag when bat voltage < 3.1V.
  Only sets flag if primary power source is battery. 
*/
void bat_check(){
  int powerSource = System.powerSource();
  if(fuel.getVCell() < bat_threash && powerSource == POWER_SOURCE_BATTERY){
    ERROR.active = true;
    ERROR.bat_low = true;
  }
  // Incase battery gets charged, removes error flag
  if(fuel.getVCell() > 3.1 ){
    ERROR.active = false;
    ERROR.bat_low = false;
  }
}



/*  
    DISPLAY FUNCTIONS
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
  epd.print("Device: ");
  epd.print(ID);
  epd.display();
}


void displayDATA()
{
  int mid_offset = 105;
  CellularSignal sig = Cellular.RSSI();
  float strength = sig.getStrength();

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
    epd.print("   ++");
  }
  else{
    epd.print("   --");    
  }

  epd.setCursor(ORIGIN_X+mid_offset, 10);
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
  epd.print("Batery: ");
  epd.print(System.batteryCharge());
  epd.print("%  ");
  epd.setCursor(ORIGIN_X+mid_offset, 20);
  epd.print("Cell:");
  epd.print(strength);
  epd.print("   Ch: ");
  epd.print(System.batteryState());



  /* Prints PM Sensor Data */
  epd.setCursor(ORIGIN_X, 30);
  epd.setTextColor(EPD_BLACK);
  epd.print("Dp 0.3: ");
  epd.print(pm.pt_03());
  epd.setCursor(ORIGIN_X+mid_offset, 30);
  epd.print("E PM 1.0: ");
  epd.print(pm.pm10_env());

  epd.setCursor(ORIGIN_X, 40);
  epd.print("Dp 0.5: ");
  epd.print(pm.pt_05());
  epd.setCursor(ORIGIN_X+mid_offset, 40);
  epd.print("E PM 2.5: ");
  epd.print(pm.pm25_env());

  epd.setCursor(ORIGIN_X, 50);
  epd.print("Dp 1.0: ");
  epd.print(pm.pt_10());
  epd.setCursor(ORIGIN_X+mid_offset, 50);
  epd.print("E PM 10 : ");
  epd.print(pm.pm100_env());

  epd.setCursor(ORIGIN_X, 60);
  epd.print("Dp 2.5: ");
  epd.print(pm.pt_25());
  epd.setCursor(ORIGIN_X+mid_offset, 60);
  epd.print("S PM 1.0: ");
  epd.print(pm.pm10_st());

  epd.setCursor(ORIGIN_X, 70);
  epd.print("Dp 5.0: ");
  epd.print(pm.pt_50());
  epd.setCursor(ORIGIN_X+mid_offset, 70);
  epd.print("S PM 2.5: ");
  epd.print(pm.pm25_st());

  epd.setCursor(ORIGIN_X, 80);
  epd.print("Dp10.0: ");
  epd.print(pm.pt_100());
  epd.setCursor(ORIGIN_X+mid_offset, 80);
  epd.print("S PM 10 : ");
  epd.print(pm.pm100_st());

  /* Prints GPS data */
  epd.setCursor(ORIGIN_X, 90);
  epd.print("Temp: ");
  epd.print(bme.readTemperature());
  epd.print("C  RH: ");
  epd.print((int)bme.readHumidity());
  epd.print("%  P: ");
  epd.print(bme.readPressure()/1000);
  epd.print("kPa");

  epd.setCursor(ORIGIN_X, 100);
  epd.print("Samp: ");
  epd.print(DISPLAY_REFRESH);
  epd.print("s    Disp: ");
  epd.print(SENSOR_CYCLE);
  epd.print("s    Pub: ");
  epd.print(PUBLISH_RATE/3600);
  epd.print("h");

/*removing GPS display, the modules do not work
  epd.setCursor(ORIGIN_X, 110);
  epd.print("Lat: ");
  epd.print(GPS.latitudeDegrees);
  epd.print("   Long: ");
  epd.print(GPS.longitudeDegrees);
  epd.print("   Fix: ");
  epd.print(GPS.fixquality);
  */

  epd.display();
}



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
  CLOUD FUNCTIONS
*/

/* 
    Renaming cloud function. handler() is required to parse out the 
    returned value from the cloud and store it in eeprom. 
 */
int renameID(String name) {
    char ID[ID_BUFF_SIZE];
    name.toCharArray(ID,ID_BUFF_SIZE);
    EEPROM.put(NAME_ADDR,ID);
    deviceID = name;

    return 0;
}


/* 
    Sets the duty cycles for the display, sampling and publishing, directly from the console. Parses the correct format
    and places it in the EEPROM storage. 
 */
int setDutyCycle(String duty){
  unsigned long scrDuty;
  unsigned long senDuty;
  unsigned long pubDuty;

  float scrf,senf,pubf;
  int del,del2;

  // find delimiter
  del = duty.indexOf(",");
  del2 = duty.lastIndexOf(",");
  if(del==-1 || del2 == -1){
    // you didnt follow the convention, didnt find the ","
    return 1;
  }

  // set the first substring to everything up to the delimiter and set as float
  // set second string to delimiter + 1 to the end and set as float
  scrf = duty.substring(0,del).toFloat();
  senf = duty.substring(del+1,del2).toFloat();
  pubf = duty.substring(del2+1).toFloat();

  // typecast the float to unsigned long
  scrDuty = (unsigned long)scrf;
  senDuty = (unsigned long)senf;
  pubDuty = (unsigned long)pubf;

  // place the values in eeprom
  EEPROM.put(DUTY_SCR_ADDR,scrDuty);
  EEPROM.put(DUTY_SEN_ADDR,senDuty);
  EEPROM.put(DUTY_PUB_ADDR,pubDuty);
  // set the global duty cycle values to the new ones from the cloud
  DISPLAY_REFRESH = scrDuty;
  SENSOR_CYCLE = senDuty;
  PUBLISH_RATE = pubDuty;

  //requires a return int. 
  return 0;
}

void update_average(){
  AVERAGE.pt_03 = AVERAGE.pt_03 + pm.pt_03();
  AVERAGE.pt_05 = AVERAGE.pt_05 + pm.pt_05();
  AVERAGE.pt_10 = AVERAGE.pt_10 + pm.pt_10();
  AVERAGE.pt_25 = AVERAGE.pt_25 + pm.pt_25();
  AVERAGE.pt_50 = AVERAGE.pt_50 + pm.pt_50();
  AVERAGE.pt_100 = AVERAGE.pt_100 + pm.pt_100();
  AVERAGE.pm10_env = AVERAGE.pm10_env + pm.pm10_env();
  AVERAGE.pm25_env = AVERAGE.pm25_env + pm.pm25_env();
  AVERAGE.pm100_env = AVERAGE.pm100_env + pm.pm100_env();  pinMode(GPS_EN,OUTPUT);
  AVERAGE.pm10_st = AVERAGE.pm10_st + pm.pm10_st();
  AVERAGE.pm25_st = AVERAGE.pm25_st + pm.pm25_st();
  AVERAGE.pm100_st = AVERAGE.pm100_st + pm.pm100_st();
  AVERAGE.temp = AVERAGE.temp + bme.readTemperature();
  AVERAGE.humidity = AVERAGE.humidity + bme.readHumidity();
  AVERAGE.pressure = AVERAGE.pressure + bme.readPressure();
  AVERAGE.altitude = AVERAGE.altitude + bme.readAltitude(SEALEVELPRESSURE_HPA);
}

void take_average(){
  AVERAGE.pt_03 = AVERAGE.pt_03/N_SAMPLES;
  AVERAGE.pt_05 = AVERAGE.pt_05/N_SAMPLES;
  AVERAGE.pt_10 = AVERAGE.pt_10/N_SAMPLES;
  AVERAGE.pt_25 = AVERAGE.pt_25/N_SAMPLES;
  AVERAGE.pt_50 = AVERAGE.pt_50/N_SAMPLES;
  AVERAGE.pt_100 = AVERAGE.pt_100/N_SAMPLES;
  AVERAGE.pm10_env = AVERAGE.pm10_env/N_SAMPLES;
  AVERAGE.pm25_env = AVERAGE.pm25_env/N_SAMPLES;
  AVERAGE.pm100_env = AVERAGE.pm100_env/N_SAMPLES;
  AVERAGE.pm10_st = AVERAGE.pm10_st/N_SAMPLES;
  AVERAGE.pm25_st = AVERAGE.pm25_st/N_SAMPLES;
  AVERAGE.pm100_st = AVERAGE.pm100_st/N_SAMPLES;
  AVERAGE.temp = AVERAGE.temp /N_SAMPLES;
  AVERAGE.humidity = AVERAGE.humidity/N_SAMPLES;
  AVERAGE.pressure = AVERAGE.pressure/N_SAMPLES;
  AVERAGE.altitude = AVERAGE.altitude/N_SAMPLES;
}

void reset_average(){
  AVERAGE.pt_03 = 0;
  AVERAGE.pt_05 = 0;
  AVERAGE.pt_10 = 0;
  AVERAGE.pt_25 = 0;
  AVERAGE.pt_50 = 0;
  AVERAGE.pt_100 = 0;
  AVERAGE.pm10_env = 0;
  AVERAGE.pm25_env = 0;
  AVERAGE.pm100_env = 0;
  AVERAGE.pm10_st = 0;
  AVERAGE.pm25_st = 0;
  AVERAGE.pm100_st = 0;
  AVERAGE.temp = 0;
  AVERAGE.humidity = 0;
  AVERAGE.pressure = 0;
  AVERAGE.altitude = 0;
}


// Draw our team logo in 80x70 size
// x is the x coordinate of left up corner of the logo
// y is the y coordinate of left up corner of the logo
// void draw_AsLogoL(uint16_t x, uint16_t y) {
//   epd.clearBuffer();
//   epd.fillScreen(EPD_WHITE);
//   const unsigned char myBitmapBitmap [] PROGMEM = {
// 	// 'AeroSpec, 80x70px
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
// 	// 'AeroSpec, 40x35px
// 	0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 
// 	0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 
// 	0xe3, 0x00, 0x00, 0x00, 0x00, 0xc3, 0x00, 0x00, 0x00, 0x01, 0xc7, 0x80, 0x00, 0x00, 0x01, 0x86, 
// 	0x80, 0x00, 0x00, 0x03, 0x8c, 0xc0, 0x00, 0x00, 0x03, 0x08, 0x60, 0x00, 0x00, 0x07, 0x18, 0x60, 
// 	0x00, 0x00, 0x0e, 0x30, 0x30, 0x00, 0x00, 0x0c, 0x20, 0x10, 0x00, 0x00, 0x1c, 0x61, 0x18, 0x00, 
// 	0x00, 0x18, 0x43, 0x08, 0x00, 0x00, 0x38, 0xc3, 0x8c, 0x00, 0x00, 0x31, 0x87, 0x86, 0x00, 0x00, 
// 	0x71, 0x8f, 0xc6, 0x00, 0x00, 0xe3, 0x0c, 0xc3, 0x00, 0x00, 0xc3, 0x00, 0xe1, 0x00, 0x01, 0xc6, 
// 	0x00, 0x71, 0x80, 0x01, 0x86, 0x00, 0x30, 0x80, 0x03, 0x8f, 0xfe, 0x38, 0xc0, 0x03, 0x00, 0x00, 
// 	0x18, 0x60, 0x07, 0x00, 0x00, 0x1c, 0x60, 0x0e, 0x00, 0x00, 0x0c, 0x30, 0x0f, 0xff, 0xff, 0xfe, 
// 	0x10, 0x1f, 0xff, 0xff, 0xff, 0x18, 0x10, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x0c, 
// 	0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x06, 0x7f, 0xff, 0xff, 0xff, 0xff
//   };
//   epd.drawBitmap(x, y, myBitmapBitmap, 40, 35, EPD_BLACK);
//   char ID[20] = "-";
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
