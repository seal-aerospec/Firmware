#include "pmsa_i2c.h"
#include "application.h"
#include <Particle.h>

/* 
    Brenden Singh -  singhb3@uw.edu

    PMSA003 - I2C :
    https://github.com/ZIOCC/Zio-Qwiic-PM2.5-Air-Quality-Sensor-Adapter-Board

    This driver is meant to be used with the PMSA003 i2c version. Pretty straight forward. 

    Create instance

    call begin() in setup phase

    call functions as nessecary. 

    Tested and working on Particle Boron. Needs arduino portability in the future. 
    If not done, just add the wire.h library and remove the particle.h and application.h. 





 */



PMSA003::PMSA003(){}

/* 
    These 3 begin functions just initalize the address to the correct
    i2c instance. Only use the third one, no parameters in 99% of cases. 
*/
bool PMSA003::begin(TwoWire *theWire) //dont use
{
    _wire = theWire;
    _i2caddr = PMSA_ADDRESS;
    return init();
}

bool PMSA003::begin(uint8_t addr) // dont use
{
    _i2caddr = addr;
    _wire = &Wire;
    return init();
}

bool PMSA003::begin() // use this, in set up
{
    _i2caddr = PMSA_ADDRESS;
    _wire = &Wire;
    return init();
}

bool PMSA003::init() // private, called by begin only
{
    _wire->begin();

    // To prevent null readings, sets to 0
    for (int i = 0; i < 15; i++)
    {
        _datBuffer[i] = 0;
    }
    return true;
}


/* 
    This needs to be called before taking data. Allocates the recorded i2c data and stores in buffer. 

    Call this everytime you want new data. 

    v1.1 - changed to a bool, returns true if data is non-zero and new. 
 */
bool PMSA003::poll() 
{   
    _wire->requestFrom(PMSA_ADDRESS,PMSA_DEFAULT_SIZE);

    uint8_t i = 0;

    bool CONN_FLAG = false;

    //pull 32 bytes of data from i2c address and place it in the buffer
    while (_wire->available())
    {
        _streamBuffer[i++] = _wire->read();
        CONN_FLAG = true;
    }

    // Data is little endian'd, sets it to a data buffer from stream
    for (uint8_t i = 0; i < 15; i++)
    {
        _datBuffer[i] = _streamBuffer[2 + i * 2 + 1];
        _datBuffer[i] += (_streamBuffer[2 + i * 2] << 8);
    }

    return CONN_FLAG;
}

/* 
    Call these to get the data you want. Returns a const integer. 
    Description in order:

    Concentration (standard):
        PM 1.0
        PM 2.5
        PM 10


    Concentration (Environmental)
        PM 1.0
        PM 2.5
        PM 10

    Particle Count
        Particles > 0.3um
        Particles > 0.5um
        Particles > 1.0um
        Particles > 2.5um
        Particles > 5.0um
        Particles > 10.0um
 */

const uint16_t PMSA003::pm10_st()
{
    return _datBuffer[1];
}

const uint16_t PMSA003::pm25_st()
{
    return _datBuffer[2];
}

const uint16_t PMSA003::pm100_st()
{
    return _datBuffer[3];
}

const uint16_t PMSA003::pm10_env()
{
    return _datBuffer[4];
}

const uint16_t PMSA003::pm25_env()
{
    return _datBuffer[5];
}

const uint16_t PMSA003::pm100_env()
{
    return _datBuffer[6];
}

const uint16_t PMSA003::pt_03()
{
    return _datBuffer[7];
}

const uint16_t PMSA003::pt_05()
{
    return _datBuffer[8];
}

const uint16_t PMSA003::pt_10()
{
    return _datBuffer[9];
}

const uint16_t PMSA003::pt_25()
{
    return _datBuffer[10];
}

const uint16_t PMSA003::pt_50()
{
    return _datBuffer[11];
}

const uint16_t PMSA003::pt_100()
{
    return _datBuffer[12];
}



/* 
    Prints out a block of the data when called. 

 */
void PMSA003::test_print(){
    Serial.println();
    Serial.println("---------------------------------------");
    Serial.println("Concentration");
    Serial.print("PM 1.0: ");
    Serial.print(pm10_st());
    Serial.print("PM 2.5: ");
    Serial.print(pm25_st());
    Serial.print("PM 10: ");
    Serial.println(pm100_st());
    Serial.println("---------------------------------------");
    Serial.println("Environmental");
    Serial.print("PM 1.0: ");
    Serial.print(pm10_env());
    Serial.print("PM 2.5: ");
    Serial.print(pm25_env());
    Serial.print("PM 10: ");
    Serial.println(pm100_env());
    Serial.println("---------------------------------------");
    Serial.print("Particles > 0.3um / 0.1L air:");
    Serial.println(pt_03());
    Serial.print("Particles > 0.5um / 0.1L air:");
    Serial.println(pt_05());
    Serial.print("Particles > 1.0um / 0.1L air:");
    Serial.println(pt_10());
    Serial.print("Particles > 2.5um / 0.1L air:");
    Serial.println(pt_25());
    Serial.print("Particles > 5.0um / 0.1L air:");
    Serial.println(pt_50());
    Serial.print("Particles > 10.0 um / 0.1L air:");
    Serial.println(pt_100());
    Serial.println("---------------------------------------");
}


