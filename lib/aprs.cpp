#include "aprs.h"
uint16_t crc = 0;
uint8_t consecutiveOnes = 0;
uint8_t bitMask= 0;
uint8_t bitPos = 7;

APRS::APRS(DRA818V *DRA, SSID *addr, uint8_t nSSIDs) {
   radio = DRA;
   packet_buffer = new uint8_t[BUFFER_SIZE_MAX]();
   num_HDLC_Flags = N_HDLC_FLAGS;
   APRS::setSSIDs(addr, nSSIDs);
}

void APRS::sendPacket(uint8_t* data_buffer, String comment, int len) {
    crc = 0;
    consecutiveOnes = 0;
    bitMask = 0;
    bitPos = 7;
    APRS::clearPacket();
    APRS::loadHeader();
    APRS::loadData(data_buffer, len);
    APRS::loadFooter();
    afsk_modulate_packet(packet_buffer, APRS::getPacketSize());
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
        for (j = 0; j<sizeof(ssids[addr].address); j++) {
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
    for(int i = 0; i < num_HDLC_Flags; i++) {
      APRS::loadHDLCFlag();
    }
}
void APRS::loadByte(uint8_t byte) {
    uint8_t i = 0;
    while (i < 8) {
      APRS::loadBit(byte & 1, 1);
      byte >>= 1;
      i++;
    }
}
void APRS::loadBit(uint8_t bit, bool bitStuff) {
    if (bitStuff) {
        APRS::update_crc(bit);
    }
    if(bitPos == 0) {
        bitPos = 7;
        packet_buffer[packet_size]  = bitMask;
        packet_size++; 
        bitMask = 0;  
    }
    if (bit) {
        bitMask |= (1<<bitPos);
        if (bitStuff && ++consecutiveOnes == BIT_STUFF_THRESHOLD) {
            APRS::loadBit(0, false);
            consecutiveOnes = 0;
        }
        bitPos--;
    } else {
        consecutiveOnes = 0;
        bitPos--;
    }
}

void APRS::loadHDLCFlag() {
    APRS::loadBit(0, false);
    for (int i = 0; i < 6; i++) {
        APRS::loadBit(1, false);
    }
    APRS::loadBit(0, false);

}

void APRS::loadString(char* str, uint8_t len) {
    for(int i = 0; i < len; i++) {
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

