#include "aprs.h"
uint16_t crc = 0;
uint8_t consecutiveOnes = 0;
uint8_t bitMask= B00000000;
uint8_t bitPos = 8;
void latToStr(char * const s, const int size, float lat);
void lonToStr(char * const s, const int size, float lon);
APRS::APRS(DRA818V *DRA, SSID *addr, uint8_t nSSIDs) {
   radio = DRA;
   packet_buffer = new uint8_t[BUFFER_SIZE_MAX]();
   num_HDLC_Flags = N_HDLC_FLAGS;
   APRS::setSSIDs(addr, nSSIDs);
}

void APRS::sendPacketGPS(
    const uint8_t dayOfMonth, const uint8_t hour, const uint8_t min,
    const float lat,
    const float lon, // degrees
    const float altitude, // meters
    const uint16_t heading, // degrees
    const float speed,
    const char * const comment) {

    crc = 0xffff;
    consecutiveOnes = 0;
    bitMask = 0;
    bitPos = 8;
    APRS::clearPacket();
    APRS::loadHeader();
    char temp[12];
    APRS::loadByte('/'); // Report w/ timestamp, no APRS messaging. $ = NMEA raw data
    snprintf(temp, sizeof(temp), "%02u%02u%02uz", (unsigned int) dayOfMonth,
        (unsigned int) hour, (unsigned int) min);
    APRS::loadString(temp);
    latToStr(temp, sizeof(temp), lat);
    APRS::loadString(temp);             // Lat:
    APRS::loadByte('/');          // Which Symbol table to use
    lonToStr(temp, sizeof(temp), lon);
    APRS::loadString(temp);     // Lon: 000deg and 25.80 min
    APRS::loadByte('O');  // The symbol
    snprintf(temp, sizeof(temp), "%03u", heading);
    APRS::loadString(temp);             // Heading (degrees)
    APRS::loadByte('/');                // and
    snprintf(temp, sizeof(temp), "%03d", (unsigned int) (speed + 0.5));
    APRS::loadString(temp);             // speed (knots)
    APRS::loadString("/A="); // Altitude (feet). Goes anywhere in the comment area
    snprintf(temp, sizeof(temp), "%06ld", (long) (altitude / 0.3048)); // 10000 ft = 3048 m
    APRS::loadString(temp);  
    APRS::loadString(comment);
    APRS::loadFooter();
    APRS::loadTrailingBits(bitPos);//load the trailing bits that might exist due to bitstuffing
    afsk_modulate_packet(packet_buffer, APRS::getPacketSize(),(8-bitPos));
    }
    
void APRS::sendPacketGPS(
    const uint8_t dayOfMonth, const uint8_t hour, const uint8_t min,
    const float lat,
    const float lon, // degrees
    const float altitude, // meters
    const uint16_t heading, // degrees
    const float speed,
    String comment) {
      
    crc = 0xffff;
    consecutiveOnes = 0;
    bitMask = 0;
    bitPos = 8;
    APRS::clearPacket();
    APRS::loadHeader();
    char temp[12];
    APRS::loadByte('/'); // Report w/ timestamp, no APRS messaging. $ = NMEA raw data
    snprintf(temp, sizeof(temp), "%02u%02u%02uz", (unsigned int) dayOfMonth,
        (unsigned int) hour, (unsigned int) min);
    APRS::loadString(temp);
    latToStr(temp, sizeof(temp), lat);
    APRS::loadString(temp);             // Lat:
    APRS::loadByte('/');          // Which Symbol table to use
    lonToStr(temp, sizeof(temp), lon);
    APRS::loadString(temp);     // Lon: 000deg and 25.80 min
    APRS::loadByte('O');  // The symbol
    snprintf(temp, sizeof(temp), "%03u", heading);
    APRS::loadString(temp);             // Heading (degrees)
    APRS::loadByte('/');                // and
    snprintf(temp, sizeof(temp), "%03d", (unsigned int) (speed + 0.5));
    APRS::loadString(temp);             // speed (knots)
    APRS::loadString("/A="); // Altitude (feet). Goes anywhere in the comment area
    snprintf(temp, sizeof(temp), "%06ld", (long) (altitude / 0.3048)); // 10000 ft = 3048 m
    APRS::loadString(temp);  
    APRS::loadString(comment);
    APRS::loadFooter();
    APRS::loadTrailingBits(bitPos);//load the trailing bits that might exist due to bitstuffing
    afsk_modulate_packet(packet_buffer, APRS::getPacketSize(),(8-bitPos));  
    }
    
void APRS::sendPacketNoGPS(String data) {
    crc = 0xffff;
    consecutiveOnes = 0;
    bitMask = 0;
    bitPos = 8;
    APRS::clearPacket();
    APRS::loadHeader();
    APRS::loadString(data);
    APRS::loadFooter();
    APRS::loadTrailingBits(bitPos);//load the trailing bits that might exist due to bitstuffing
    afsk_modulate_packet(packet_buffer, APRS::getPacketSize(),(8-bitPos));
}

void APRS::clearPacket() {
    packet_size = 0;
}

void APRS::setSSIDs(SSID *addr, uint8_t numSSIDs) {
    ssids = new SSID[numSSIDs];
    for(int i = 0; i < numSSIDs; i++) {
        ssids[i].address = addr[i].address;
        ssids[i].ssid_designator = addr[i].ssid_designator;
    }
    num_ssids = numSSIDs;
}

void APRS::loadHeader() {
    for(int i = 0; i < num_HDLC_Flags; i++) {
        APRS::loadHDLCFlag();
    }
    for (int addr = 0; addr < num_ssids; addr++) {
        // Transmit callsign   
        uint8_t j = 0; 
        for (j = 0; j<strlen(ssids[addr].address); j++) {
          APRS::loadByte(ssids[addr].address[j] <<1);
        }
        // Transmit pad 
        while (j < 6) {
            APRS::loadByte(' '<<1);
            j++;
        }
      // Transmit SSID. Termination signaled with last bit = 1
      if (addr == num_ssids - 1)
        APRS::loadByte(('0' + ssids[addr].ssid_designator) << 1 | 1);
      else
        APRS::loadByte(('0' + ssids[addr].ssid_designator) << 1);
    }

    // Control field Byte: 3 = APRS-UI frame
    APRS::loadByte(0x03);

    // Protocol ID Byte: 0xf0 = no layer 3 data
    APRS::loadByte(0xf0);
}
void APRS::loadData(uint8_t* data_buffer, uint8_t length) {
    for(int i = 0; i < length;i++ ) {
        APRS::loadByte(data_buffer[i]);
    }
}

void APRS::loadFooter() {
    uint16_t final_crc = crc;
    // Send the CRC
    APRS::loadByte(~(final_crc & 0xff));
    final_crc >>= 8;
    APRS::loadByte(~(final_crc & 0xff));
    APRS::loadHDLCFlag();
}

//Transmits a byte of information, which can be anything except for the FCS sequence.
//By protocol, all bytes are transmitted least significant byte first, except for the FCS sequence.
void APRS::loadByte(uint8_t byte) {
//  Serial.println(byte,BIN);
    for(int i = 0; i < 8; i++) {
        APRS::loadBit(byte & 1,true);
        byte>>=1;//next iteration transmits the next bit to the left
    }  
//  Serial.print(".");
//  Serial.print((char) byte);
//  Serial.println(byte,BIN);
}

//the loadBit function will load each bit of data to a bitmask, but will only insert the bitmask into the packet buffer after 8 bits
//have been loaded. If bitstuffing occurs (a nonmultiple of 8 times), then there will be leftover bits in the bitmask that need to be added to the packet.
void APRS::loadTrailingBits(uint8_t bitIndex) {
    if(bitIndex==0) {
        return; //if there were no trailing bits, return
    }
    packet_buffer[packet_size/8] = bitMask;
}

void APRS::loadBit(uint8_t bit, bool bitStuff) {
//    Serial.println(bit);
//    Serial.println(bitPos);
    if (bitStuff) {
        APRS::update_crc(bit);
    }
    if(bitPos == 0) {
        bitPos = 8;
        packet_buffer[packet_size/8 - 1] = bitMask;
//        Serial.print("*");
//        Serial.println(bitMask,BIN);
        bitMask = B00000000;  
    }
    if (bit) {
//          Serial.println(bitMask,BIN);
//          Serial.println(bitPos);
        bitMask |= (1<<(bitPos-1));
//            Serial.println(bitMask,BIN);

        bitPos--;
        packet_size++;
        if (bitStuff && ++consecutiveOnes == BIT_STUFF_THRESHOLD) {
            APRS::loadBit(0, false);
//            Serial.print("!");
            consecutiveOnes = 0;
        }
    } else {
        consecutiveOnes = 0;
        bitPos--;
        packet_size++;
    }
}

void APRS::loadHDLCFlag() {
    APRS::loadBit(0, false);
    for (int i = 0; i < 6; i++) {
        APRS::loadBit(1, false);
    }
    APRS::loadBit(0, false);
}

void APRS::loadString(String str) {
    for(uint8_t i = 0; i < str.length(); i++) {
        APRS::loadByte(str[i]);
    }
}
void APRS::loadString(char* str) {
    for(uint8_t i = 0; i < strlen(str); i++) {
        APRS::loadByte(str[i]);
    }
}

void APRS::update_crc(uint8_t bit) {
    const uint16_t xor_int = crc ^ bit;  // XOR lsb of CRC with the latest bit
    crc >>= 1;                          // Shift 16-bit CRC one bit to the right
    if (xor_int & 0x0001) {              // If XOR result from above has lsb set
        crc ^= 0x8408;                  // Shift 16-bit CRC one bit to the right
    }
    return;
}

int APRS::getPacketSize() {
    return packet_size;
}

void latToStr(char * const s, const int size, float lat)
{
  char hemisphere = 'N';
  if (lat < 0) {
    lat = -lat;
    hemisphere = 'S';
  }
  const int deg = (int) lat;
  lat = (lat - (float) deg) * 60.0f;
  const int min = (int) lat;
  lat = (lat - (float) min) * 100.0f;
  const int minTenths = (int) (lat + 0.5); // Round the tenths
  snprintf(s, size, "%02d%02d.%02d%c", deg, min, minTenths, hemisphere);
}

// Convert latitude from a float to a string
void lonToStr(char * const s, const int size, float lon)
{
  char hemisphere = 'E';
  if (lon < 0) {
    lon = -lon;
    hemisphere = 'W';
  }
  const int deg = (int) lon;
  lon = (lon - (float) deg) * 60.0f;
  const int min = (int) lon;
  lon = (lon - (float) min) * 100.0f;
  const int minTenths = (int) (lon + 0.5); // Round the tenths
  snprintf(s, size, "%03d%02d.%02d%c", deg, min, minTenths, hemisphere);
}
