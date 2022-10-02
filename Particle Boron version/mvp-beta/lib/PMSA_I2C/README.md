# PMSA003 I2C Driver v1.1 - Testing

## Version - v1.1 
Date : 5/8/2020

Changed poll() to return a boolean instead of being a void. 

- `true` if `available()` returns non-zero and can grab bytes if from the i2c address
- `false` if `available()` returns 0 



## General - v1.0
Author : Brenden Singh

Date : 2/12/20

This driver is meant to be used with the PMSA003 Plantower particle sensor, i2c version. 

https://github.com/ZIOCC/Zio-Qwiic-PM2.5-Air-Quality-Sensor-Adapter-Board

This will not work with the UART version. Can be modified to work with UART in future. If you want to use this on arduinos, remove the includes `particle.h` and `application.h`. Add the include `wire.h`. Currently working on Particle Boron. Add .cpp and .h files to the src folder of your sketch. Add `#include "pmsa_i2c.h"` to your sketch. 


To use the driver, create an instance of the class PMSA003 before the setup loop:

` PMSA003 pm = PMSA003();`

Where `pm` is the instance. 

Then call `pm.begin();` during the setup 

`pm.poll()` collects data from the device and stores it in a buffer. Call this every time you want fresh data. Printing data before calling this will call whatever data was stored from the previous poll.


Then call the following functions to retrieve data:

```
	// Particle Counts
    const uint16_t pt_03();
    const uint16_t pt_05();
    const uint16_t pt_10();
    const uint16_t pt_25();
    const uint16_t pt_50();	
    const uint16_t pt_100();
    
    // Mass Concentration Counts - Environment
    const uint16_t pm10_env();
    const uint16_t pm25_env();
    const uint16_t pm100_env();

    // Mass Concentration Counts - Standard
    const uint16_t pm10_st();
    const uint16_t pm25_st();
    const uint16_t pm100_st();

```

`pm.print_test` will print out a block of the data for you over the serial port. Initialize `Serial.begin()` before calling this method. 

## Example

Prints data every 5 seconds: 

```
#include"pmsa_i2c.h"

PMSA003 pm = PMSA003(); // create instance of class

void setup()
{     // join i2c bus (address optional for master)
  Serial.begin(9600); // start serial for output
  pm.begin();
  delay(1000);
}


void loop()
{
  pm.poll();
  
  pm.test_print();
  delay(5000);
}
```