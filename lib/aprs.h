#ifndef APRS_H
#define APRS_H

#include "aprs_global.h"
#include "dra818v.h"
#include "Arduino.h"
#include "afsk.h"
#include <SoftwareSerial.h>
using namespace std;

static const uint8_t HDLC_FLAG = 0x7E;
static const int BUFFER_SIZE_MAX = 256; //bytes
static const uint8_t BIT_STUFF_THRESHOLD = 5;

class APRS
{
public:
    APRS(DRA818V* DRA, SSID* addr, uint8_t nSSIDs);
    void setSSIDs(SSID* addr, uint8_t numSSIDs);
    void sendPacket(uint8_t *data_buffer, String comment, int len);
    int getPacketSize();
    void clearPacket();
private:
    void loadHeader();
    void loadData(uint8_t *data_buffer, uint8_t length);
    void loadFooter();
    void loadByte(uint8_t byte);
    void loadBit(uint8_t bit, bool bitStuff);
    void loadString(char* str, uint8_t length);
    void loadHDLCFlag();
    void update_crc(uint8_t bit);
    DRA818V* radio;
    uint8_t num_HDLC_Flags;
    SSID* ssids;
    uint8_t num_ssids;
    volatile uint8_t* packet_buffer;
    int packet_size;
};
#endif // APRS_H
