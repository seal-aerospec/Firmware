/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "d:/Education/UW/Aerospec/particle_boron/firmware-beta/mvp-beta/src/mvp-beta.ino"
/* 
  MVP Beta
  04/21/2020
 */



#include "pmsa_i2c.h"
#include "Adafruit_BME280_RK.h"
#include "Adafruit_EPD.h"
#include "SdFat.h"
#include "Particle.h"
#include "dct.h"

void setup();
void loop();
void writeToFile(String path, String content);
void displayInit();
void ble_init();
void write_to_ble(char *packet);
void backUp2BLE(char *output);
#line 15 "d:/Education/UW/Aerospec/particle_boron/firmware-beta/mvp-beta/src/mvp-beta.ino"
#define VERSION_R "V2.2.2"
#define NAME_ADDR 10
#define DUTY_SCR_ADDR 255
#define DUTY_SEN_ADDR 260
#define DUTY_PUB_ADDR 265
#define ID_BUFF_SIZE 20
#define SAMPLE_CHAR_SIZE 265

#define PM_RST A0
#define PM_SET A1
#define PP5V0_EN D8


#define SEALEVELPRESSURE_HPA (1013.25)
#define SD_CS D2
#define SRAM_CS D3
#define EPD_CS D4
#define EPD_DC D5
#define EPD_RESET -1 // can set to -1 and share with microcontroller Reset!
#define EPD_BUSY -1  // can set to -1 to not use a pin (will wait a fixed delay)

#define ORIGIN_X 5
#define ORIGIN_Y 5

#define OUTPUTFILE "Data.txt"
#define BACKUPFILE "Backup.txt"
#define ONE_DAY_MILLIS (24 * 60 * 60 * 1000)


//for the SD read functionality
#define SD_FAT_TYPE 0



SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);


FuelGauge fuel;

Adafruit_BME280 bme;
/* Screen obj */
//Tri-Color
Adafruit_IL0373 epd(212, 104, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);
//Bi-Color
//Adafruit_SSD1675 epd(250,122, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);
/* Particle Sensor obj */
PMSA003 pm = PMSA003(); // create instance of class
SdFat SD;
File myFile;


/* Refresh rates */
unsigned long DISPLAY_REFRESH; 
unsigned long SENSOR_CYCLE;
unsigned long PUBLISH_RATE;
//changing this to every 30 seconds for dev
unsigned long BACKUP_RATE = 60*15;

unsigned long preSensorDutyMillis;
unsigned long predisplayDutyMillis;
unsigned long prePublishDutyMillis;
unsigned long preTimeCheckMillis;
unsigned long preBackupSampleMillis;


bool SD_WRITE_SUCCESSFUL = false;
//adding a bluetooth connected/disconnected flag
bool PHONE_BACKUP_REQUEST = false;


/* 
  Struct to store average data, running average. normalized is a flag for whether each sample has been divided by the total number of samples. 
 */
typedef struct Data_average{
  bool normalized = false;

  uint32_t pt_03;
  uint32_t pt_05;
  uint32_t pt_10;
  uint32_t pt_25;
  uint32_t pt_50;
  uint32_t pt_100;
  
  //Mass Concentration Counts - Environment
  uint32_t pm10_env;
  uint32_t pm25_env;
  uint32_t pm100_env;

  //Mass Concentration Counts - standard
  uint32_t pm10_st;
  uint32_t pm25_st;
  uint32_t pm100_st;

  float temp;
  float humidity;
  float pressure;
  float altitude;

  uint16_t N = 0;


}Data_average;
Data_average run_average_CLOUD;
Data_average run_average_BLE;



/* 
  New feature, global struct to save the latest sample. Makes each latest sample globably accessable. 
 */


typedef struct Sample{
  bool PUB_CLOUD = false;
  bool PUB_BLE = false;

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

  uint8_t seconds;
  uint8_t minute;
  uint8_t hour;
  uint8_t day;
  uint8_t month;
  uint16_t year;

  float battery;

}Sample;
Sample latest_sample;
Sample av_sample_CLOUD;
Sample av_sample_BLE;


String deviceID;

// Enum for sample rate definitions
typedef enum{
  RATE_HIGH = 3,
  RATE_MED = 2,
  RATE_LOW = 1
}TaskRates;


// Enum for Sample writing destinations, decision making on whether to write to SD card, publish to cloud or potential BLE
typedef enum {
  DEST_SD,
  DEST_CLOUD,
  DEST_BLE,
  DEST_BLE_BACKUP,
  DEST_BACKUP_PHONE
} SampleDestination;



const size_t UART_TX_BUF_SIZE = SAMPLE_CHAR_SIZE;

void onDataReceived(const uint8_t *data, size_t len, const BlePeerDevice &peer, void *context);

//adding in prototypes because my computer seems to need them
void record_sample(SampleDestination dest);
void sample_write(Sample* sample, char *output);
void take_average(Sample *latest, Sample *av_sample, Data_average* average);
void reset_average(Data_average* average);
void take_sample(Sample* latest);
void displayDATA(Sample* sample);
void update_average(Sample* latest, Data_average* average );
void writePM(Sample* sample, char* pmData);

// These UUIDs were defined by Nordic Semiconductor and are now the defacto standard for
// UART-like services over BLE. Many apps support the UUIDs now, like the Adafruit Bluefruit app.
const BleUuid serviceUuid("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
const BleUuid rxUuid("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
const BleUuid txUuid("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");

BleCharacteristic txCharacteristic("tx", BleCharacteristicProperty::NOTIFY, txUuid, serviceUuid); // originally NOTIFY rather than READ -- Charlie
BleCharacteristic rxCharacteristic("rx", BleCharacteristicProperty::WRITE_WO_RSP, rxUuid, serviceUuid, onDataReceived, NULL);

//BleCharacteristic rxCharacteristic("rx", BleCharacteristicProperty::WRITE, rxUuid, serviceUuid);
//BleCharacteristic heartRateMeasurementCharacteristic;



// Particle cloud functions need to be declared
int renameID(String name);
int setSampleRate(String usr_input);
int setDisplayRate(String usr_input);
int setPublishRate(String usr_input);

// flag for counting number of transmitting data
int count_flag = 0;

void setup()
{ 

  // Sets these reset pins to HIGH during normal operation, also must turn on enable pins PM sensor
  pinMode(PM_RST,OUTPUT);
  pinMode(PP5V0_EN,OUTPUT);
  digitalWrite(PM_RST,HIGH);
  digitalWrite(PP5V0_EN,HIGH);



 /* 
    Cloud functions and variables need to be defined as so BEFORE
    Particle.connect() is called. Otherwise they wont register. 
    This is due to us running in SYSTEM_THREAD(ENABLED), auto is disabled 
    so we need to push this manually. 
  
   */
  Particle.function("Rename Device", renameID);

  Particle.function("Set Display Rate (1-3)",setDisplayRate);
  Particle.function("Set Publish Rate (1-3)",setPublishRate);
  Particle.function("Set Sample Rate (1-3)",setSampleRate);

  Particle.variable("Display Refresh Rate",DISPLAY_REFRESH);
  Particle.variable("Sensor Sample Rate",SENSOR_CYCLE);
  Particle.variable("Publish Rate", PUBLISH_RATE);
  Particle.variable("Device ID",deviceID);



  /* Currently fixing listening issue, look at again later */
  Cellular.setActiveSim(INTERNAL_SIM);
  Cellular.clearCredentials();

  const uint8_t val = 0x01;
  dct_write_app_data(&val, DCT_SETUP_DONE_OFFSET, 1);
  //Cellular.on();
  //Particle.connect();
  Cellular.off();
  /*  */

  

  // Serial.begin(115200);


  delay(1000);
  SD.begin(SD_CS);
  delay(1000);

  bme.begin();
  pm.begin();
  epd.begin();

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
    DISPLAY_REFRESH = 60*15;
    SENSOR_CYCLE = 30;
    PUBLISH_RATE = 36000;
    EEPROM.put(DUTY_SCR_ADDR,DISPLAY_REFRESH);
    EEPROM.put(DUTY_SEN_ADDR,SENSOR_CYCLE);
    EEPROM.put(DUTY_PUB_ADDR,PUBLISH_RATE);
  }
  


  ble_init();
  

  /*
    default behavior is to dump the backup file to phone if the device turns on
    and it exists.  
  */
  PHONE_BACKUP_REQUEST = true;

  // Splash Screen
  displayInit();  
  delay(5000);

}

void loop()
{
  SampleDestination samp_dest;
  //samp_dest = DEST_BACKUP_PHONE;

  if (!preSensorDutyMillis || !predisplayDutyMillis || !prePublishDutyMillis || !preTimeCheckMillis )
  {
    preSensorDutyMillis = millis();
    predisplayDutyMillis = millis();
    prePublishDutyMillis = millis();
    preTimeCheckMillis = millis();
    preBackupSampleMillis = millis();
  }

  unsigned long curSensorDutyMillis = millis();
  unsigned long curDisplayDutyMillis = millis();
  unsigned long curPublishDutyMillis = millis();
  unsigned long curTimeCheckMillis = millis();
  unsigned long curBackupSampleMillis = millis();


  if (curTimeCheckMillis - Particle.timeSyncedLast() >= ONE_DAY_MILLIS && Particle.connected){
    Particle.syncTime();
    // Wait until the device receives time from Particle Device Cloud (or connection to Particle Device Cloud is lost)
    // Will wait 10 seconds for the sync to be done. Blocking. 
    waitFor(Particle.syncTimeDone,10000);

  }

  
  if (curSensorDutyMillis - preSensorDutyMillis >= SENSOR_CYCLE*1000)
  {
        // Writes header to text file on SD card if the file doesn't exist.
    samp_dest = DEST_SD;
    take_sample(&latest_sample);
    SD_WRITE_SUCCESSFUL = false;
    record_sample(samp_dest);

    samp_dest  = DEST_BLE;
    record_sample(samp_dest);

    Serial.println("This is paritcle3\n");

    //set to run with the sensor cycle, checks to see if the phone was disconnected>15 minutes
    //if true it runs the backup protocol 
    //PHONE_BACKUP_REQUEST = true;
    if(PHONE_BACKUP_REQUEST){
      if(BLE.connected()){
        Serial.println("This is paritcle2\n");
      samp_dest = DEST_BACKUP_PHONE;
      record_sample(samp_dest);
      }
    }

    // If past duty time, reset timer
    preSensorDutyMillis = curSensorDutyMillis;
  }

  // Task for displaying. SENSOR_CYCLE should be set either way. 
  if( (curDisplayDutyMillis - predisplayDutyMillis)  >= DISPLAY_REFRESH*1000){
    displayDATA(&latest_sample);
    predisplayDutyMillis = curDisplayDutyMillis;
  }


  // BLE and Cloud publishes are currently tied here in the same task. If they should be different,
  if( (curPublishDutyMillis - prePublishDutyMillis)  >= PUBLISH_RATE*1000){
    samp_dest = DEST_CLOUD;
    record_sample(samp_dest);
    prePublishDutyMillis = curPublishDutyMillis;
  }

  if( (curBackupSampleMillis - preBackupSampleMillis) >=  BACKUP_RATE*1000){
    //check if we have ble connecting, if not 
    if(!BLE.connected())PHONE_BACKUP_REQUEST = true;

    if (PHONE_BACKUP_REQUEST){
      samp_dest = DEST_BLE_BACKUP;
      record_sample(samp_dest);
    }

    preBackupSampleMillis = curBackupSampleMillis;
  }


}

/* 
  Records a sample to a destination. The destination will determine what type of sample. For SD card writes, it will be the latest sample. 

  For cloud writes, it will be the average of certain sets of data over a period of time. The period of time is determined by the last time the average batch was sampled. 

 */
void record_sample(SampleDestination dest){
  char data[SAMPLE_CHAR_SIZE];

  
  Serial.println("I am here\n");
  dest = DEST_BACKUP_PHONE;
  switch(dest){

    case DEST_SD :
      sample_write(&latest_sample,data);
      writeToFile(OUTPUTFILE, data);
      break;

    case DEST_BLE :

      sample_write(&latest_sample,data);
      if (BLE.connected()){
        write_to_ble(data);
      }
      break;

    case DEST_CLOUD :

      take_average(&latest_sample, &av_sample_CLOUD, &run_average_CLOUD);
      sample_write(&av_sample_CLOUD,data);
      if(Particle.connected){
        Particle.publish("sensor-device-to-db", data, PRIVATE);
        av_sample_CLOUD.PUB_CLOUD = true;
      }
      reset_average(&run_average_CLOUD);
      break;

    case DEST_BLE_BACKUP :

      if(!BLE.connected()){
        take_average(&latest_sample, &av_sample_BLE, &run_average_BLE);
        sample_write(&av_sample_BLE,data);
        writeToFile(BACKUPFILE,data);
        reset_average(&run_average_BLE);
      }

      break;

      case DEST_BACKUP_PHONE:
      Serial.println("I am here2\n");
        backUp2BLE(data);
      break;

  }

}

/* 

  Most of these data/average write functions below now have parameters to allow the use of multiple structs while enforcing a workflow.
  The structs are still globally accessable although now this could change in the future. 

 */

/* 
  Setter for taking a sample and placing it in the latest struct. 
 */

void take_sample(Sample* latest){
  pm.poll();



  if (Time.isValid()){
    latest->seconds = Time.second();
    latest->minute = Time.minute();
    latest->hour = Time.hour();
    latest->day = Time.day();
    latest->month = Time.month();
    latest->year = Time.year();

  }


  latest->pt_03 = pm.pt_03();
  latest->pt_05 = pm.pt_05();
  latest->pt_10 = pm.pt_10();
  latest->pt_25 = pm.pt_25();
  latest->pt_50 = pm.pt_50();
  latest->pt_100 = pm.pt_100();
  latest->pm10_env = pm.pm10_env();
  latest->pm25_env = pm.pm25_env();
  latest->pm100_env = pm.pm100_env();
  latest->pm10_st = pm.pm10_st();
  latest->pm25_st = pm.pm25_st();
  latest->pm100_st = pm.pm100_st();
  latest->temp = bme.readTemperature();
  latest->humidity = bme.readHumidity();
  latest->pressure = bme.readPressure();
  latest->altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  latest->battery = System.batteryCharge();

  // Update the average values every time a sample is taken
  update_average(latest, &run_average_BLE);
  update_average(latest, &run_average_CLOUD);
}





/* 
  Writes a sample or average sample to a char array. This is just for formating the data. Taking and resetting the data is done here. The running average is checked for normalization before 
  writing the averaged data out with the normalized flag. 

 */

void sample_write(Sample* sample, char *output){

  char timeData[20], pmData[150], posLatData[2], posLonData[2], fixData[2], dateData[20], batData[10], tempData[10], rhData[10], altData[10], pressData[10];

  writePM(sample,pmData);
  sprintf(tempData, "%.2f",sample->temp);
  sprintf(rhData, "%.2f",sample->humidity);
  sprintf(pressData,"%.2f",sample->pressure);
  sprintf(altData,"%.2f",sample->altitude);
  sprintf(timeData, "%u:%u:%u", sample->hour, sample->minute, sample->seconds); // String for Time
  sprintf(dateData, "%u/%u/%u", sample->year, sample->month, sample->day);     // String for date
  sprintf(posLatData,"%u",0); // no GPS set to default 0
  sprintf(posLonData,"%u",0);  // no GPS set to default 0
  sprintf(fixData,"%u",0);  // no GPS set to default 0
  sprintf(batData,"%.2f", sample->battery);
  
  sprintf(output, "%10s,%10s,%10s,%5s,%15s,%15s,%10s,%10s,%5s,%5s,%5s", dateData, timeData, batData, fixData, posLatData, posLonData, pmData, tempData, rhData, pressData, altData);

}


/* 
  These two functions format only the PM data, its a hold over from previous itterations and I dont want to break it. It will either write the sampled data or the average data. 
 */


void writePM(Sample* sample, char* pmData) {
  char p03[6]; char p05[6]; char p10[6]; char p25[6]; char p50[6]; char p100[6];
  char pm10_std[6]; char pm25_std[6]; char pm100_std[6];
  char pm10_env[6]; char pm25_env[6]; char pm100_env[6];
  String(sample->pt_03).toCharArray(p03, 6);
  String(sample->pt_05).toCharArray(p05, 6);
  String(sample->pt_10).toCharArray(p10, 6);
  String(sample->pt_25).toCharArray(p25, 6);
  String(sample->pt_50).toCharArray(p50, 6);
  String(sample->pt_100).toCharArray(p100, 6);
  String(sample->pm10_st).toCharArray(pm10_std, 6);
  String(sample->pm25_st).toCharArray(pm25_std, 6);
  String(sample->pm100_st).toCharArray(pm100_std, 6);
  String(sample->pm10_env).toCharArray(pm10_env, 6);
  String(sample->pm25_env).toCharArray(pm25_env, 6);
  String(sample->pm100_env).toCharArray(pm100_env, 6);
  sprintf(pmData, "%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s", p03, p05, p10, p25, p50, p100, pm10_std, pm25_std, pm100_std, pm10_env, pm25_env, pm100_env);
}


/* 
    Method handles writing to SD card with a given string, created by sdPrint.
*/

void writeToFile(String path, String content) {
  //create the file with headers if it doesn't exist.
  if (!SD.exists(path)) {
    char label[260];
    
    myFile = SD.open(path, FILE_WRITE);

    sprintf(label, "%10s,%10s,%10s,%5s,%15s,%15s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s",
    "Date","Time", "Battery", "Fix","Latitude","Longitude","Dp>0.3","Dp>0.5","Dp>1.0","Dp>2.5","Dp>5.0","Dp>10.0",
    "PM1_Std","PM2.5_Std","PM10_Std","PM1_Env","PM2.5_Env","PM10_Env","Temp(C)","RH(%)","P(hPa)","Alti(m)");
    myFile.println(deviceID);
    myFile.println(label);
  }
  else myFile = SD.open(path, FILE_WRITE);

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





void update_average(Sample* latest, Data_average* average ){

  average->pt_03 = average->pt_03 + latest->pt_03;
  average->pt_05 = average->pt_05 + latest->pt_05;
  average->pt_10 = average->pt_10 + latest->pt_10;
  average->pt_25 = average->pt_25 + latest->pt_25;
  average->pt_50 = average->pt_50 + latest->pt_50;
  average->pt_100 = average->pt_100 + latest->pt_100;
  average->pm10_env = average->pm10_env + latest->pm10_env;
  average->pm25_env = average->pm25_env + latest->pm25_env;
  average->pm100_env = average->pm100_env + latest->pm100_env;
  average->pm10_st = average->pm10_st + latest->pm10_st;
  average->pm25_st = average->pm25_st + latest->pm25_st;
  average->pm100_st = average->pm100_st + latest->pm100_st;
  average->temp = average->temp + latest->temp;
  average->humidity = average->humidity + latest->humidity;
  average->pressure = average->pressure + latest->pressure;
  average->altitude = average->altitude + latest->altitude;
  average->N++;
}


void take_average(Sample *latest, Sample *av_sample, Data_average* average){

  av_sample->pt_03 = average->pt_03/average->N;
  av_sample->pt_05 = average->pt_05/average->N;
  av_sample->pt_10 = average->pt_10/average->N;
  av_sample->pt_25 = average->pt_25/average->N;
  av_sample->pt_50 = average->pt_50/average->N;
  av_sample->pt_100 = average->pt_100/average->N;
  av_sample->pm10_env = average->pm10_env/average->N;
  av_sample->pm25_env = average->pm25_env/average->N;
  av_sample->pm100_env = average->pm100_env/average->N;
  av_sample->pm10_st = average->pm10_st/average->N;
  av_sample->pm25_st = average->pm25_st/average->N;
  av_sample->pm100_st = average->pm100_st/average->N;
  av_sample->temp = average->temp /average->N;
  av_sample->humidity = average->humidity/average->N;
  av_sample->pressure = average->pressure/average->N;
  av_sample->altitude = average->altitude/average->N;


  av_sample->seconds = latest->seconds;
  av_sample->minute = latest->minute;
  av_sample->hour = latest->hour;
  av_sample->day = latest->day;
  av_sample->month = latest->month;
  av_sample->year = latest->year;

}

void reset_average(Data_average* average){
  average->normalized = false;
  average->pt_03 = 0;
  average->pt_05 = 0;
  average->pt_10 = 0;
  average->pt_25 = 0;
  average->pt_50 = 0;
  average->pt_100 = 0;
  average->pm10_env = 0;
  average->pm25_env = 0;
  average->pm100_env = 0;
  average->pm10_st = 0;
  average->pm25_st = 0;
  average->pm100_st = 0;
  average->temp = 0;
  average->humidity = 0;
  average->pressure = 0;
  average->altitude = 0;
  average->N = 0;
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


void displayDATA(Sample* sample)
{

  uint8_t mid_offset = 105;
  CellularSignal sig = Cellular.RSSI();
  float strength = sig.getStrength();

  char ID[ID_BUFF_SIZE];
  EEPROM.get(NAME_ADDR,ID);
  String IDs(ID);


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
  epd.print(sample->month);
  epd.print("/");
  epd.print(sample->day);
  epd.print("/");
  epd.print(sample->year);
  epd.print("  ");
  epd.print(sample->hour);
  epd.print(":");
  epd.print(sample->minute);
  epd.print(":");
  epd.print(sample->seconds);
  
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
  epd.print(sample->pt_03);
  epd.setCursor(ORIGIN_X+mid_offset, 30);
  epd.print("E PM 1.0: ");
  epd.print(sample->pm10_env);

  epd.setCursor(ORIGIN_X, 40);
  epd.print("Dp 0.5: ");
  epd.print(sample->pt_05);
  epd.setCursor(ORIGIN_X+mid_offset, 40);
  epd.print("E PM 2.5: ");
  epd.print(sample->pm25_env);

  epd.setCursor(ORIGIN_X, 50);
  epd.print("Dp 1.0: ");
  epd.print(sample->pt_10);
  epd.setCursor(ORIGIN_X+mid_offset, 50);
  epd.print("E PM 10 : ");
  epd.print(sample->pm100_env);

  epd.setCursor(ORIGIN_X, 60);
  epd.print("Dp 2.5: ");
  epd.print(sample->pt_25);
  epd.setCursor(ORIGIN_X+mid_offset, 60);
  epd.print("S PM 1.0: ");
  epd.print(sample->pm10_st);

  epd.setCursor(ORIGIN_X, 70);
  epd.print("Dp 5.0: ");
  epd.print(sample->pt_50);
  epd.setCursor(ORIGIN_X+mid_offset, 70);
  epd.print("S PM 2.5: ");
  epd.print(sample->pm25_st);

  epd.setCursor(ORIGIN_X, 80);
  epd.print("Dp10.0: ");
  epd.print(sample->pt_100);
  epd.setCursor(ORIGIN_X+mid_offset, 80);
  epd.print("S PM 10 : ");
  epd.print(sample->pm100_st);

  epd.setCursor(ORIGIN_X, 90);
  epd.print("Temp: ");
  epd.print(sample->temp);
  epd.print("C  RH: ");
  epd.print((int) sample->humidity);
  epd.print("%  P: ");
  epd.print(sample->pressure/1000);
  epd.print("kPa");

  epd.setCursor(ORIGIN_X, 100);
  epd.print("Samp: ");
  epd.print(DISPLAY_REFRESH);
  epd.print("s    Disp: ");
  epd.print(SENSOR_CYCLE);
  epd.print("s    Pub: ");
  epd.print(PUBLISH_RATE);
  epd.print("h");

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




int setSampleRate(String usr_input){
  unsigned long senDuty;
  int num_input = usr_input.toInt();

  TaskRates enum_input;
  enum_input = (TaskRates) num_input;

  switch(enum_input)
  {
    case 3:
      senDuty = 5;
      break;

    case 2:
      senDuty = 10;
      break;

    case 1:
      senDuty = 20;
      break;

    // LOW
    default:
      senDuty = 20;
      return 1;
      break;
  }

  EEPROM.put(DUTY_SEN_ADDR,senDuty);
  SENSOR_CYCLE = senDuty;

  return 0;

}




int setDisplayRate(String usr_input){

  unsigned long scrDuty;
  int num_input = usr_input.toInt();

  TaskRates enum_input;
  enum_input = (TaskRates) num_input;

  switch(enum_input)
  {
    case 3:
      scrDuty = 5;
      // Serial.println("Case 3");
      break;

    case 2:
      scrDuty = 10;
      // Serial.println("Case 2");

      break;

    case 1:
      scrDuty = 20;
      // Serial.println("Case 1");

      break;

    // LOW
    default:
      scrDuty = 20;
      // Serial.println("Falt");
      return 1;
      break;
  }

  EEPROM.put(DUTY_SEN_ADDR,scrDuty);
  DISPLAY_REFRESH = scrDuty;

  return 0;

}

int setPublishRate(String usr_input){

  unsigned long pubDuty;
  int num_input = usr_input.toInt();

  TaskRates enum_input;
  enum_input = (TaskRates) num_input;


  switch(enum_input)
  {
    case 3:
      pubDuty = 60;
      break;

    case 2:
      pubDuty = 300;
      break;

    case 1:
      pubDuty = 600;
      break;

    // LOW
    default:
      pubDuty = 600;
      return 1;
      break;
  }

  EEPROM.put(DUTY_PUB_ADDR,pubDuty);
  PUBLISH_RATE = pubDuty;

  return 0;
}


void onDataReceived(const uint8_t *data, size_t len, const BlePeerDevice &peer, void *context)
{


  //Log.trace("Received data from: %02X:%02X:%02X:%02X:%02X:%02X:\n", peer.address()[0], peer.address()[1], peer.address()[2], peer.address()[3], peer.address()[4], peer.address()[5]);

  //char* data_receive = (char*) data;
  //Serial.printlnf("The Data is %d \n", data[1]);

   Serial.printlnf("The onDataRecived is working. \n");

  //  for (size_t ii = 0; ii < len; ii++)
  //  {
  //    Serial.printlnf("The Data is %c \n", (char) data[ii]);
  //  }

  // for (size_t j = 0; j < len; j++)
  // {
  //   val = val * 10 + data[j] - '0';
  // }

  // if (val > 29)
  // {
  //   //txCharacteristic.setValue(txBuf, txLen);
  //   PUBLISH_RATE = val;
  //   // Serial.write(val);
  // }


    // checking whether the last string "BACKUPEND" is sent back from device to smartphone 
    // char str[10] = "BACKUPEND";
    // char* data_receive = (char*) data;

    // if(strncmp(str, data_receive, 10) == 0){

    //   Serial.printlnf("We have received the end of data and can delete the file\n");  
    //   Serial.printlnf("The Data is %s \n", data_receive);
    //   //return ;
    //   // Delete file:
    //   // SD.remove(BACKUPFILE);
    // }else{

    //     // if the data does not correspond with "BACKUPEND".

    //     //Serial.printlnf("The sent data has not reached the end of data\n");
    //     Serial.printlnf("The data we received\n");
    //     Serial.printlnf("The Data is %s \n", data_receive);
    //     //return ;
      
    // }
    
    // new version of onDataReceived
    //data[0] contains numbers of received data 
    int val = (int) data[0];
    if(val == count_flag){

      Serial.printlnf("We had received all datas\n");
      count_flag = 0;
      // SD.remove(BACKUPFILE);
    }else{

      Serial.printlnf("There are some errors in the data and we need to transmit data again \n");
      Serial.printlnf("The Data is %d \n", val);
    }

}

void ble_init(){

  BLE.on();

  // robert testing (possible solution)
  rxCharacteristic.onDataReceived(onDataReceived,NULL);

  //heartRateMeasurementCharacteristic.onDataReceived(onDataReceived, NULL);

  //BLE.addCharacteristic(heartRateMeasurementCharacteristic);
  BLE.addCharacteristic(txCharacteristic);
  BLE.addCharacteristic(rxCharacteristic);

  BleAdvertisingData data;
  data.appendServiceUUID(serviceUuid);

  String myID = System.deviceID();
  String LocalName = deviceID + " " + myID;
  String LocalName2 = "A e06";
  
  data.appendLocalName(deviceID);
  uint8_t buf[BLE_MAX_ADV_DATA_LEN];

  size_t offset = 0;

  // Manufacturer-specific data
  // 16-bit: Company ID (0xffff)
  // Byte: Internal packet identifier (0x55)
  // 32-bit: Color code

  // Company ID (0xffff internal use/testing)
  buf[offset++] = 0xff;
  buf[offset++] = 0xff;

  // Internal packet type. This is arbitrary, but provides an extra
  // check to make sure the data is my data, since we use the 0xffff company
  // code.
  buf[offset++] = 0x55;
  const uint32_t myColor = 0xff0000;
  memcpy(&buf[offset], &myColor, 4);
  // Our specific data, color code
  
  offset += 4;

  data.appendCustomData(buf,offset );
  BLE.advertise(&data);

  
}


void write_to_ble(char *packet){

  uint8_t txBuf[UART_TX_BUF_SIZE];
  size_t txLen = 0;

  for (size_t i = 0; i < strlen(packet); i++)
  {
    txBuf[txLen++] = packet[i];
    //Serial.write(txBuf[txLen - 1]);
  }
  if (txLen > 0)
  {
    txCharacteristic.setValue(txBuf, txLen);
  }

}

/* 
  function takes in a blank char array and uses it to store output from the sd card
  global variables used: BACKUPFILE,SAMPLE_CHAR_SIZE,PHONE_BACKUP_REQUEST
  functions: write_to_ble
  Uses the File class for reading from the sd card but the SDFat class to delete the file
*/
// void backUp2BLE(char *output){
//   myFile = SD.open(BACKUPFILE);
//   if(myFile){
//     char start[12] ="BACKUPSTART";
//     char end[10] ="BACKUPEND";
//     write_to_ble(start);
//     while (myFile.available()) {
//       if(!myFile.fgets(output,SAMPLE_CHAR_SIZE)){
//         break;
//       }
//       write_to_ble(output);
//     }
//     myFile.close();
//     write_to_ble(end);
   
//     SD.remove(BACKUPFILE);
//   }
//   // else Serial.println("backup faied");
//   /*
//     for cases where the phone starts and the backup file does not exist, SD.open
//     will fail and the backup request will be turned off.
//   */
//     PHONE_BACKUP_REQUEST = false;
// }


/*

Robert's version.

*/

void backUp2BLE(char *output){
  Serial.println("I came here, please find me\n");
  myFile = SD.open("Backup.txt");
  if(!myFile)
  {
  Serial.println("Ensure your macro file open\n");
  }
  if(myFile){
    Serial.println("File exist\n");

    // This is the flag for counting the number has been sent;
    // The reason why it is -2 is becasue one for "device name" and the other for "header of the table" 
    count_flag = -2 ;
    char start[12] ="BACKUPSTART";
    char end[10] ="BACKUPEND";

    write_to_ble(start);
    

    Serial.println(millis());
    while (myFile.available()) {
      if(!myFile.fgets(output,SAMPLE_CHAR_SIZE)) break;
      write_to_ble(output);
      count_flag++;
      if(!BLE.connected()) {
        Serial.println("connection lost");
        return;
        }
    }
    myFile.close();
    write_to_ble(end);
    
    //removes the file here and prints to serial
    delay(1000);
    //if (DELETE_FLAG)SD.remove(BACKUPFILE);
  } else {
    // Serial.println("File not Found");
    PHONE_BACKUP_REQUEST = false;
    return;
  }
  Serial.printlnf("Communication Broadcast");
  Serial.println(millis());
  /*
    for cases where the phone starts and the backup file does not exist, SD.open
    will fail and the backup request will be turned off.
  */
    PHONE_BACKUP_REQUEST = false;
}





