#ifndef DRA818V_H
#define DRA818V_H
#include "aprs_global.h"
#include <stdint.h>
#include <string>
#include <stdbool.h>
#include <SoftwareSerial.h>
using namespace std;

#include "Arduino.h" //For compiling in the Arduino IDE, uncomment this

#define USE_HW_SERIAL false  //override use of software serial by defining true here

static const int SOFT_SERIAL_BAUD = 9600;
static const int SERIAL_BAUD = 9600;
static const bool CHANNEL_SCAN_BW = true; //true: 25kHz, false: 12.5kHz

class DRA818V
{

public:
    DRA818V(uint8_t PTT, uint8_t audioOut, uint8_t mic, uint8_t draTX = 0, uint8_t draRX = 0);
    void init();
    void setSquelch(uint8_t sq_level);
    void setPTTDelay(uint16_t delayMs);
    #if USE_HW_SERIAL == true
        HardwareSerial *radioSerial;
    #else
        SoftwareSerial *radioSerial;
    #endif
private:
    void configSettings();
    uint8_t pttPin = 0;
    uint8_t micPin = 0;
    uint8_t audioOutPin = 0;
    uint8_t pwmPin = 0;
    uint16_t pttDelay = 0;
    uint8_t squelch;
};

#endif // DRA818V_H
