#ifndef APRS_GLOBAL_H
#define APRS_GLOBAL_H

#include "Arduino.h"

struct SSID {
    char* address;
    uint8_t ssid_designator;
};

const bool DEBUG = true;
#define LED_PIN 13
#define PTT_PIN 2
#define MIC_PIN A14
#define PWM_PIN A0
#define AUDIO_PIN A8
#define DRATX 7
#define DRARX 8
#define TX_CTCSS "0000"
#define RX_CTCSS "0000"
#define PTT_DELAY 700 //ms
#define N_HDLC_FLAGS 2

#define SINE_WAVE_RESOLUTION 12
#if defined(APRS_LIBRARY)
#  define APRSSHARED_EXPORT Q_DECL_EXPORT
#else
#  define APRSSHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // APRS_GLOBAL_H
