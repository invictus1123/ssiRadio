#include "afsk.h"

// Interrupt-related constants and instance variables
IntervalTimer interruptTimer;
volatile bool txing = false;
volatile bool rxing = false;

volatile int freq = MARK_FREQ;

volatile uint16_t analogOut = 0;
volatile uint32_t currentPhase = 0; //32bit integer for higher res integer math
volatile uint32_t increment = MARK_INCREMENT;

//the index used to count how many samples we've sent for the current bit
volatile byte bitIndex = 0;
//the index denoting which bit within the current byte we are transmitting
volatile byte byteIndex = 0;
//the index denoting which bit we are transmitting
volatile int packetIndex = 0;
//the current byte being transmitted
volatile char currentByte = 0; 
//data structure for the packet
volatile uint8_t* packet;
//the total number of bits in the packet
volatile int packet_size = 0;

// 1200 Baud settings
void radioISR(void);

uint16_t sineLookup(const int currentPhase) {
    uint16_t analogOut = 0;
    int index = currentPhase % (SINE_TABLE_LENGTH/2);
    analogOut = (index < SINE_TABLE_LENGTH/4) ? sineTable[index] : sineTable[SINE_TABLE_LENGTH/2 - index - 1];
    analogOut = (currentPhase >= ( SINE_TABLE_LENGTH/2 )) ? ((SINE_WAVE_MAX/2)- 1) - analogOut : analogOut;
    return analogOut;
}

void afsk_modulate_packet(volatile uint8_t *buffer, int size, int trailingBits) {
    packet = buffer;
    packet_size = size;
    Serial.println(size);
    analogWriteResolution(SINE_WAVE_RESOLUTION);
    afsk_timer_begin();
}

void afsk_timer_begin() {
    if(DEBUG) {
        interruptTimer.begin(radioISR,(float)1E6/(SAMPLE_RATE/DEBUG_PRESCALER)); //microseconds
//        Serial.println(SAMPLES_PER_BIT);
//        Serial.println(MARK_INCREMENT);
//        Serial.println(SPACE_INCREMENT);
    } else {
        interruptTimer.begin(radioISR,(float)1E6/(SAMPLE_RATE));
    }
    resetVolatiles();
    digitalWrite(PTT_PIN,LOW);
    delay(PTT_DELAY);//todo: change this to match DRA object's delay
    txing = true;
}

void resetVolatiles() {
    freq = MARK_FREQ;
    analogOut = 0;
    currentPhase = 0;
    increment = MARK_INCREMENT;
    bitIndex = 0;
    byteIndex = 0;
    packetIndex = 0;
    currentByte = 0;
}

void afsk_timer_stop() {
    interruptTimer.end();
}

void radioISR() {
    if(!txing) return;
    if(DEBUG) digitalWrite(LED_PIN,HIGH);
    if(bitIndex == 0) {
        if(packetIndex >= packet_size) {
            txing = false;
            analogWrite(MIC_PIN,SINE_WAVE_MAX/2);
            digitalWrite(PTT_PIN,HIGH);
            if(DEBUG) {
                digitalWrite(LED_PIN,LOW);
            }
            afsk_timer_stop();
            return;
        } else if (byteIndex== 0) {
            currentByte = packet[packetIndex/8]; //grab the next byte to transmit
            byteIndex = B10000000; //reset the byte bitmask to the first bit
        }
        //if we are supposed to send a 1 at this bit, maintain current TX frequency (NRZ encoding), unless we have sent 5 1s in a row and must stuff a 0
        if(currentByte & byteIndex) { //transmitting a 1
          if(DEBUG) {
//              Serial.println("-1-");
          }
            //freq remains unchanged
        } else { //transmitting a 0
            if(DEBUG)  {
//                Serial.println("-0-");
            }
            freq = (freq == MARK_FREQ) ? SPACE_FREQ : MARK_FREQ; //if we are supposed to send a 0 at this bit, change the current TX frequency
        }
        increment = (freq == MARK_FREQ) ? MARK_INCREMENT : SPACE_INCREMENT; //adjust the phase delta we add each sample depending on whether we are transmitting 0 or 1
        bitIndex = SAMPLES_PER_BIT; //reset the bitIndex;
        packetIndex++;
        byteIndex>>=1; //shift to the next bit to be transmitted
    } //endif: bitIndex == 0
    currentPhase+=increment;
    if(currentPhase > SINE_TABLE_LENGTH*ANGLE_RESOLUTION_PRESCALER) {
        currentPhase-=SINE_TABLE_LENGTH*ANGLE_RESOLUTION_PRESCALER;
    }
//    Serial.println(currentPhase/ANGLE_RESOLUTION_PRESCALER);
    bitIndex--;
    analogOut = sineLookup((currentPhase/ANGLE_RESOLUTION_PRESCALER));
//    Serial.println(analogOut);
    if(freq == MARK_FREQ) {
        analogOut = analogOut*PREEMPHASIS_RATIO;
    }
    analogWrite(MIC_PIN, analogOut);
    if(DEBUG) digitalWrite(LED_PIN,LOW);
}


