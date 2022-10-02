
/* EINK display demo for Aerospec
 * Avoid refrshing display more than once every 3 minutes
 * 
 */
 
#include <Adafruit_GFX.h>    // Core graphics library
#include "Adafruit_EPD.h"

// SCK -> 18
// MISO -> 19
// MOSI -> 23
#define EPD_CS     5
#define EPD_DC      17
#define SRAM_CS     16
#define EPD_RESET   4 // can set to -1 and share with microcontroller Reset!
#define EPD_BUSY    34 // can set to -1 to not use a pin (will wait a fixed delay) 
#define EPD_ENA     27 // set low to turn off display

#define FLEXIBLE_213
#define COLOR1 EPD_BLACK
#define COLOR2 EPD_RED

void displayPower(bool enable) {
  if(enable) {
    digitalWrite(EPD_ENA, HIGH);
    Serial.println("Powering on Display");
  }
  else {
    digitalWrite(EPD_ENA, LOW);
    Serial.println("Powering off Display");
  }
}

void resetEInk() {
  digitalWrite(EPD_RESET, HIGH);
  digitalWrite(EPD_RESET, LOW);
}

bool isBusy() {
  bool busy =  digitalRead(EPD_BUSY) == LOW;
  if(busy) {
    Serial.println("Busy");
  }
  return busy;
}

void setup(void) {
  Serial.begin(115200);
  Serial.print("Hello! EPD Test");

  pinMode(EPD_BUSY, INPUT);
  pinMode(EPD_RESET, OUTPUT);
  digitalWrite(EPD_RESET, LOW);
  pinMode(EPD_ENA, OUTPUT);
  displayPower(true);
  // Init 2.13" 212x104 Eink display
  Adafruit_IL0373 display(212, 104, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);
  delay(500);
  display.begin();
  
  #if defined(FLEXIBLE_213) || defined(FLEXIBLE_290)
    // The flexible displays have different buffers and invert settings!
    display.setBlackBuffer(1, false);
    display.setColorBuffer(1, false);
  #endif

  Serial.println("Initialized");
  display.setRotation(2);
  
  // large block of text
  display.clearBuffer();
  display.setTextWrap(true);

  display.setCursor(10, 10);
  display.setTextSize(1);
  display.setTextColor(EPD_BLACK);
  display.print("Is this working? I sure hope so.");
  display.display();
  displayPower(false);
  
//  delay(10 * 1000);
//  display.clearBuffer();
//  for (int16_t i=0; i<display.width(); i+=4) {
//    display.drawLine(0, 0, i, display.height()-1, COLOR1);
//  }
//
//  for (int16_t i=0; i<display.height(); i+=4) {
//    display.drawLine(display.width()-1, 0, 0, i, COLOR2);  // on grayscale this will be mid-gray
//  }
//  
//  display.display();
//  delay(10 * 1000);
//  
//  display.clearBuffer();
//  display.fillRect(display.width()/3, 0, display.width()/3, display.height(), EPD_RED);
//  display.fillRect((display.width()*2)/3, 0, display.width()/3, display.height(), EPD_BLACK);
//  display.display();
//
//  delay(15 * 1000);
//
//  display.clearBuffer();
//  // play
//  display.fillScreen(EPD_WHITE);
//  display.fillRoundRect(25, 10, 78, 60, 8, EPD_BLACK);
//  display.fillTriangle(42, 20, 42, 60, 90, 40, EPD_RED);
// 
//  display.display();

}

void loop() {
  delay(500);
  Serial.print("Hello! EPD Test");
}
