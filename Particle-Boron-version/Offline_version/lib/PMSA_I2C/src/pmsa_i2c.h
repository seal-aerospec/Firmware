#ifndef _PMSA_I2C
#define _PMSA_I2C

#define PMSA_ADDRESS 0x12
#define PMSA_DEFAULT_SIZE 32

#include "application.h"
#include <Particle.h>

class PMSA003
{
    private:
        uint16_t _datBuffer[15];
        uint16_t _streamBuffer[PMSA_DEFAULT_SIZE];

        bool init();
        TwoWire *_wire;
        uint8_t _i2caddr;

    public: 
        PMSA003();
        bool begin();
        bool begin(TwoWire *theWire);
        bool begin(uint8_t addr);
        bool begin(uint8_t addr, TwoWire *theWire);

        

        bool poll();
        //Particle Counts
        const uint16_t pt_03();
        const uint16_t pt_05();
        const uint16_t pt_10();
        const uint16_t pt_25();
        const uint16_t pt_50();
        const uint16_t pt_100();
        
        //Mass Concentration Counts - Environment
        const uint16_t pm10_env();
        const uint16_t pm25_env();
        const uint16_t pm100_env();

        //Mass Concentration Counts - standard
        const uint16_t pm10_st();
        const uint16_t pm25_st();
        const uint16_t pm100_st();


        void test_print();
};





#endif