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
    
    void sendPacketGPS(const uint8_t dayOfMonth, const uint8_t hour, const uint8_t min,
    const float lat,
    const float lon, // degrees
    const float altitude, // meters
    const uint16_t heading, // degrees
    const float speed,
    String comment);
    
    void sendPacketGPS(const uint8_t dayOfMonth, const uint8_t hour, const uint8_t min,
    const float lat,
    const float lon, // degrees
    const float altitude, // meters
    const uint16_t heading, // degrees
    const float speed,
    const char * const comment);
    
    void sendPacketNoGPS(String data);
    void sendPacketNoGPS(char* data);
    
    int getPacketSize();
    void clearPacket();
private:
    void loadHeader();
    void loadData(uint8_t *data_buffer, uint8_t length);
    void loadFooter();
    void loadTrailingBits(uint8_t bitIndex);
    void loadByte(uint8_t byte);
    void loadBit(uint8_t bit, bool bitStuff);
    void loadString(String str);
    void loadString(char* str);
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
