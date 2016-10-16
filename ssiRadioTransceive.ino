/*  File: SSIRadioTransceive.ino
 *  SSI APRS Radio Transceiver Program
 *  Written by Vinh Nguyen, '19
 *  Attributions: 
 *  MicroModem, by Mark Qvist https://github.com/markqvist/MicroModem
 *  Kirill Safin for starting this project
 *  
 *  This file programs the DRA818V on Teensy 3.2 to send and receive APRS packets.
 *  This program utilizes interrupts which run at a set interval in between normal code, unless the macro runAtomicCode() is used, which
 *  prevents that code from being interrupted until completion. (an atomic operation is one that ignores interrupts).
 *  
 *  Overview of program:
 *  Foreground Code (non-interrupt operations):
 *  -Setup pins and radio, and make sure that radio is working. (atomic)
 *  -Assemble data into a string for transmission (TODO: integrate with avionics code, implementing a ClearToSend flag)
 *  -Convert the data into an APRS packet using the AX.25 Protocol.
 *      *AX.25 Protocol: A protocol used by APRS; it is a packet format which consists of:
 *                       *HDLC Flag*DestinationCallsign(s)*OwnCallsign*DigipeaterPath*DataIDBit*Data*FCS*HDLC Flag*
 *      *FCS: Frame Check Sequence, a 16 bit sequence calculated using some bit math which is sent with the transmission. The receiving end does the same math
 *            and determines whether any of the packet is corrupted: The results will be the same for a clean packet. (see calculateFCS() and checkIntegrity())
 *      **For a more in-depth explanation see http://www.aprs.org/doc/APRS101.PDF p.22      
 *  -Put the packet into a queue which will be read and transmitted with an interrupt function (atomic)    
 *  -Verify the integrity of and print out any received packets
 *  Background Code (interrupt operations, running at SAMPLE_RATE times per second inbetween foreground operations)
 *  -Take the bytes of the packet and split them into bits
 *  -Convert those bits into an analog waveform using Audio Frequency Shift Keying (AFSK) and a sine table using Non-Return to Zero (NRZ) encoding and Bit-Stuffing
 *      *AFSK: Send 1s and 0s by changing an audio tone between two frequencies: MARK (1200Hz) and SPACE (2200Hz).
 *             See NRZ: These frequencies don't represent 1 and 0 in digital data, but an unchanging signal "mark" and a changing signal "space"
 *      *NRZ:  A method for encoding digital data common in packet radio: The first digital bit equals a Mark ("1") and changes to Space ("0")
 *             if the next digital bit is different. Example: 01100101 is transmitted as MSMSMSSS, or 10101000. This, in combination with Bit-Stuffing, helps   
 *             make sure the transmitter and receiver are synchronized, that is, the receiver does not lose track of which part of a byte it is on or what part
 *             of a transmission it is on.
 *      *Bit-Stuffing: If at any point too many marks are transmitted (which would result in one tone being played for many samples),
 *             the receiver may lose track of where it is in the transmission ('bit slip'). Thus in transmissions whenever the data results in 5 or more marks being
 *             transmitted, a space will be put in (which must be parsed out later) to cause the tone to change and prevent bit slip.
 *  -Read samples coming in from the audioPin and discern what frequency they are    
 *  -Convert the samples into NRZ bits, looking for an APRS flag character (HDLC_FLAG) which will indicate the beginning of a transmission
 *  -Convert the NRZ bits into data once we have a transmission, filtering out any stuffed bits.
 *         
 *  TODOs:       
 *  Proper documentation for every constant and function
 *  Test transmission software by reading outputs of functions
 *  Test reception software by writing a prepared sequence of bits and seeing if they are parsed correctly
 *  Test hardware+software
 *  Create .h files and .cpp files for the code
 */

#define radioSerial Serial1 //Pins 1 (data from DRA to Teensy; RX) and 2 (data from Teensy to DRA, TX)
#define signalTransitioned(bitStream) XORTwoBit(bitStream, bitStream>>2)
#define XORTwoBit(bit1,bit2) ( ( ((bit1)^(bit2)) & 0x03 ) == 0x03)
#define lastBitsDiffer(bit1,bit2) ( ((bit1)^(bit2)) & 0x01 )
#define runAtomicCode(CODE) \
    unsigned char sregCopy; \
    sregCopy = SREG; \
    cli(); \
    CODE; \
    SREG = sregCopy;
#include <QueueArray.h>

//flag for enabling data structures for debugging transmission and reception. Setting true will declare multiple large data structures.
#define DEBUG_ARRAYS true

//Instance variables used for debugging//
// integers for counting the data sent and received
int deqBits = 0;
int deqSamples = 0;
int deqDemodSamples = 0;
int count8 = 0;

static const bool INTERRUPTS_ON = true; //flag to turn off interrupts for debugging other parts of the code

//flags for turning on debug mode for either transmission or reception
volatile bool debug = true;
volatile bool debugTx = true;
volatile bool debugRx = false;
void debugReceive();
void debugTransmit();

//Data structures for storing received data and sent data, which are only
#if DEBUG_ARRAYS == true
static const int SAMPLES_IN_CAP = 1024;
volatile int sizeSamplesOut = 0;
volatile uint16_t samplesOut[SAMPLES_IN_CAP];
volatile uint16_t samplesIn[SAMPLES_IN_CAP];
volatile int sizeSamplesIn = 0;
volatile uint8_t demodSamples[SAMPLES_IN_CAP];
volatile int sizeDemodSamples = 0;
static const int BITS_IN_CAP = 256;
volatile uint8_t bitsIn[BITS_IN_CAP];
volatile uint8_t sizeBitsIn = 0;
#endif

// DRA818V pin declaration and settings
static const int bw = 1;                                        // bandwith in KHz ( 0= 12.5KHz or 1= 25KHz )
static const float ftx = 144.3900;                    // tx frequency in MHz (134.0000 - 174.0000)
static const float frx = 144.3900;                    // rx frequency in MHz (134.0000 - 174.0000) 
String tx_ctcss = "0000";               // ctcss frequency ( 0000 - 0038 ); 0000 = "no CTCSS"  
String rx_ctcss = "0000";               // ctcss frequency ( 0000 - 0038 ); 0000 = "no CTCSS"  
static const int squ = 0;                                       // squelch level  ( 0 - 8 ); 0 = "open"  
static const int PTT = 2;
static const int LED_PIN = 3;
static const int audioPin = A8;         //the pin outputting received audio/signals
static const int micPin = A14;          //the pin doing the modulation

//APRS parameters
String messageTx = "SSI-43 Test: Hello World!";
static const String DESTINATION_ADDRESS = "AG6WH";
static const String SOURCE_ADDRESS = "KM4SEE";
static const String DIGIPEATER_PATH = "WIDE2-1";

char incomingByte;                      //storage for serial communication bytes

//flags relating to system status
bool radioGood = false;
bool clearToSend = false;
bool gotData = false;
bool checkIntegrity(String packet);     //for verifying the format of a received packet. [stub]

/* Radio Initialization */
void setupRadio();                      //Initializes radio parameters and starts talking to the module
void configSettings();                  //Sets radio parameters

/* Digital-to-Analog Conversion constants and functions */
void analogWriteTone(const byte mic, long frequency, long durationMillis);
uint16_t sineLookup(const int currentPhase);

static const uint16_t BIT_RATE = 12500;
static const uint16_t SAMPLE_RATE = BIT_RATE*8;
static const int SAMPLES_PER_BIT = 8; //SAMPLE_RATE/BIT_RATE
static const int SINE_WAVE_RESOLUTION = 512;

/* Analog-to-Digital Conversion constants and functions */
void parseNRZToDataBits(byte b);

//demodulation data structures and variables
static const int RX_BUFFER_LEN = 64;
QueueArray<int8_t> lastFiveSamples; //used for demodulation
int delayedSamples[2] = {0, 0}; //samples delayed by four timesteps
volatile uint8_t rxBuffer[RX_BUFFER_LEN];
int lowPassOut = 0;
int8_t sampledBitStream = 0;
int8_t demodulatedBitStream = 0;
int8_t dataBits = B11111111;
bool firstBit = true;
bool firstByte = false;
byte bitsRead = 0;
byte consecutiveOnes = 0;
String incomingTransmission;

//demodulation constants and instance variable dealing with phase synchronization.
static const int RECORD_BIT_THRESHOLD = SINE_WAVE_RESOLUTION/8;
static const int SYNC_ADJUST_THRESHOLD = SINE_WAVE_RESOLUTION/16;
static const int SYNC_SAMPLE_INCREMENT = SINE_WAVE_RESOLUTION/64;
static const int SYNC_ADJUST = SYNC_SAMPLE_INCREMENT/8;
int syncCounter = 0;

//Bell 202 standardized frequencies for 1(mark) and 0(space).
static int MARK_FREQ = 1200; //Hz
static int SPACE_FREQ = 2200; //Hz

//The distance in the sine table array between samples of the sine wave; how far we step forward in the sine wave for each sample sent
static int MARK_INCREMENT = SINE_WAVE_RESOLUTION * MARK_FREQ / (SAMPLE_RATE); //64
static int SPACE_INCREMENT = SINE_WAVE_RESOLUTION * SPACE_FREQ / (SAMPLE_RATE); // 117.333

//IntervalTimer object used to run an interrupt service routine at the sampling rate to transmit and receive bits.
IntervalTimer interruptTimer;

/* Boolean flags for indicating whether we are receiving a message or transmitting */
volatile bool rxing = false;
volatile bool txing = false;

/* Instance variables for use in interrupt functions */

//The analog voltage to be written out to the RF module.
volatile uint8_t analogOut = 0; //assumes 8 bit resolution

//The phase angle used to calculate what voltage to write at each sampling interval.
volatile int currentPhase = 0;

//The increment dtheta used to step forward in the sine table.
volatile int increment = 0;

//Indexes for keeping track of where we are at in the transmission: within each byte and bit.
volatile byte bitIndex = 0;
volatile byte byteIndex = 0;

//The current byte being transmitted.
volatile char currentByte = 0;

//The number of ones in a row that have been transmitted. The protocol makes sure that not more than 5 are 
//transmitted in a row to keep the transmitter and receiver in sync.
volatile byte bitStuffCounter = 0; 
static const byte BIT_STUFF_LIMIT = 5; //the maximum number of ones we can transmit before we have to stuff in a 0

//Variable denoting the actively transmitting frequency.
volatile int freq = MARK_FREQ;

// variables for testing ISR performance
volatile long t0 = 0;
volatile long t1 = 0;

//Storage for the actual bytes to be transmitted and the array containing all data.
QueueArray<char> txBuffer;
String dataBuffer = "";

/* Transmission Functions */
void enqueueData();
void transmitAPRSPacket(String &data);  //Assembles data into the correct format and tells the module to start transmitting

/* Functions called during interrupts */
void radioISR();
void radioTXISR();
void radioRXISR();
void processSample(int8_t sample);

/* Functions associated with the AX.25 / APRS protocol */
static const int PTT_ON_DELAY = 500; //ms
void transmissionProtocolInit();
void txVariablesInit();
void attachHDLCFlag();
void attachSSIDs();
int calculateFCS(String packet);
void attachFCS(const int FSCBits);

/* AX.25 Protocol constants */
//These are bytes that by protocol must go in a certain place in our packet.
static const char HDLC_FLAG = 0x7E; //01111110: six 1s in a row, which will distinguish between bitstuffed data and the wrapping HDLC char.
static const char HDLC_RESET = B01111111; //if we receive one of these, something went wrong in the transmission.
static const char CONTROL_FIELD_BYTE = 0x03; //00000011
static const char PROTOCOL_ID_BYTE = 0xF0; //11110000
static const char INFORMATION_FIELD_BYTE = 'T'; //T stands for telemetry, denoting data like GPS alt temp etc.
static const byte PACKET_FOOTER_LENGTH = 3; //the length of the FCS bytes and the HDLC byte = 3 bytes

/* Program Memory */
//Table of 128 values representing the first 128 values of the sine wave, scaled to 8 bit resolution.
//Used to modulate digital data into an analog waveform.
static int sineTable[128] = 
{
    128, 129, 131, 132, 134, 135, 137, 138, 140, 142, 143, 145, 146, 148, 149, 151,
    152, 154, 155, 157, 158, 160, 162, 163, 165, 166, 167, 169, 170, 172, 173, 175,
    176, 178, 179, 181, 182, 183, 185, 186, 188, 189, 190, 192, 193, 194, 196, 197,
    198, 200, 201, 202, 203, 205, 206, 207, 208, 210, 211, 212, 213, 214, 215, 217,
    218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233,
    234, 234, 235, 236, 237, 238, 238, 239, 240, 241, 241, 242, 243, 243, 244, 245,
    245, 246, 246, 247, 248, 248, 249, 249, 250, 250, 250, 251, 251, 252, 252, 252,
    253, 253, 253, 253, 254, 254, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255,
};

// The length of the CRC checksum in bits.
static byte CRC_LENGTH = 16;

// Lookup table containing all possible remainders for a byte being divided by the CRC polynomial.
// Used to quickly calculate a Cyclic Redundancy Check, which determines whether there has been any
// loss or distortion of the packet across transmission. (stub)
static int crcTable[256] = {
   0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
   0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
   0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485, 0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
   0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4, 0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
   0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823, 0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
   0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
   0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
   0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70, 0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
   0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
   0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
   0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D, 0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
   0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
   0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB, 0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
   0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A, 0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
   0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
   0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0 
};

//-----------------------------------------------------------------------------------------------------------------------------------------------------------TODO: Header Files
void setup()
{
  radioSerial.begin(9600);                      // open serial at 9600 bps
  Serial.begin(9600);
  pinMode(PTT,OUTPUT);
  pinMode(micPin,OUTPUT);  
  pinMode(3,OUTPUT);
  analogReadResolution(10);
  analogWriteResolution(12);
  setupRadio();
  if(INTERRUPTS_ON) {
      interruptTimer.begin(radioISR,(float) 1E6/SAMPLE_RATE); //microseconds
  }
}

void setupRadio() {
    digitalWrite(PTT,LOW);
    delay(200);
    radioSerial.print("AT+DMOCONNECT\r\n");
    digitalWrite(PTT,HIGH);
    delay(200);
    String messageRx = "";
      while (radioSerial.available() > 0) {
      incomingByte = radioSerial.read();
      messageRx = messageRx + incomingByte;
    }
    if(messageRx!="") {
      Serial.print("UART received: ");
      Serial.println(messageRx);
    }
    digitalWrite(PTT,LOW);
    delay(200);
    configSettings();
    digitalWrite(PTT,HIGH);
    delay(200);
    messageRx = "";
      while (radioSerial.available() > 0) {
      incomingByte = radioSerial.read();
      messageRx = messageRx + incomingByte;
    }
    if(messageRx!="") {
      Serial.print("UART received: ");
      Serial.println(messageRx);
    }
}
void loop() {  
    if(debugTx) {
        debugTransmit();
    } else {
        debugReceive();
    }
}

void configSettings() {
    radioSerial.print("AT+DMOSETGROUP=");      
    radioSerial.print(bw,1);
    radioSerial.print(",");
    radioSerial.print(ftx,4); 
    radioSerial.print(",");
    radioSerial.print(frx,4);
    radioSerial.print(",");
    radioSerial.print(tx_ctcss);
    radioSerial.print(",");
    radioSerial.print(squ);
    radioSerial.print(",");
    radioSerial.print(rx_ctcss);
    radioSerial.print("\r\n"); 
}

void analogWriteTone(byte mic, long freq, long durationMillis) {
  float phase = 0.0;
  float twopi = 3.14159 * 2;
  float analogVal = 0.0;
//  elapsedMicros usec = 0;
  long num_samples = (durationMillis)*SAMPLE_RATE/1000;
  long dt = durationMillis*1000/(num_samples);
  float phase_delta =  ((freq == MARK_FREQ) ? MARK_INCREMENT: SPACE_INCREMENT) * (twopi/SINE_WAVE_RESOLUTION);
  Serial.println(phase_delta);
  while(long i = 0 < num_samples) {
      analogVal = sin(phase) * 127 + 128;
      analogWrite(mic,(int)analogVal);
      phase+=phase_delta;
      if(phase > twopi) {
        phase -=twopi;
      }
      delayMicroseconds(dt);
      num_samples--;
  }   
}

void radioISR() {
    if(txing) {
        radioTXISR();
    } else {
        radioRXISR();
    }
//    } else {
//        radioRXISR();
//    }
}

void radioTXISR() {
  if(!debugRx) digitalWrite(LED_PIN,HIGH);
      if(bitIndex == 0) {
          if(byteIndex== 0) {
              if(!txBuffer.isEmpty()) {
                  currentByte = txBuffer.dequeue(); //grab the next byte to transmit
              } else {
                  txing = false;
                  digitalWrite(PTT,HIGH);
                  analogWrite(micPin,0);
                  digitalWrite(LED_PIN,LOW);
                  return;
              }
              byteIndex = B10000000; //reset the byte bitmask to the first bit
              //if we are supposed to send a 1 at this bit, maintain current TX frequency (NRZ encoding), unless we have sent 5 1s in a row and must stuff a 0
          }
          if ( (currentByte & byteIndex) && (bitStuffCounter<BIT_STUFF_LIMIT || currentByte == HDLC_FLAG) ) { 
              bitStuffCounter++;
//              if(debugTx) {
//                  bitsOut.enqueue(1);
//              }
          } else {
              freq = (freq == MARK_FREQ) ? SPACE_FREQ : MARK_FREQ; //if we are supposed to send a 0 at this bit, change the current TX frequency
//              if(debugTx) {
//                  bitsOut.enqueue(bitStuffCounter == BIT_STUFF_LIMIT ? -1 : 0);
//              }
              bitStuffCounter = 0;
          }
          byteIndex >>=1; //shift right to the next lowest bit in the byte being transmitted
          bitIndex = SAMPLES_PER_BIT; //reset the bitIndex;
      }
      increment = (freq == MARK_FREQ) ? MARK_INCREMENT : SPACE_INCREMENT;    
      currentPhase+=increment;
      if(currentPhase > SINE_WAVE_RESOLUTION) {
          currentPhase-=SINE_WAVE_RESOLUTION;
      }    
      bitIndex--;
      analogOut = sineLookup(currentPhase);
      if(debugTx) {
          if(sizeSamplesOut<SAMPLES_IN_CAP) {
              samplesOut[sizeSamplesOut] = analogOut;
              sizeSamplesOut++;
          } else {
              sizeSamplesOut = 0;
              samplesOut[sizeSamplesOut] = analogOut;
          }
      }
      analogWrite(micPin, analogOut);
  // t1 = micros();
      if(!debugRx) digitalWrite(LED_PIN,LOW);
}

uint16_t sineLookup(const int currentPhase) {
    uint16_t analogOut = 0;
    int index = currentPhase % (SINE_WAVE_RESOLUTION/2);
    analogOut = (index < SINE_WAVE_RESOLUTION/4) ? sineTable[index] : sineTable[SINE_WAVE_RESOLUTION/2 - index - 1];
    analogOut = (currentPhase >= ( SINE_WAVE_RESOLUTION/2 )) ? SINE_WAVE_RESOLUTION/2-1 - analogOut : analogOut;
    return analogOut;
}
void radioRXISR() {
    if(!debugTx) digitalWrite(LED_PIN, HIGH);
    processSample((int16_t) analogRead(audioPin)); //processSample() references the sine wave to 0, so the reading must be cast to a signed int
    if(!debugTx) digitalWrite(LED_PIN,LOW);
}

void processSample(int8_t sample) {
    if(debugRx) {
        if(sizeSamplesIn < SAMPLES_IN_CAP) {
            samplesIn[sizeSamplesIn] = sample;
            sizeSamplesIn++;
        } else {
            sizeSamplesIn = 0;
            samplesIn[sizeSamplesIn] = sample;
        }
    }
    if(lastFiveSamples.count() < 5) {
        lastFiveSamples.enqueue(sample);
        syncCounter+=SYNC_SAMPLE_INCREMENT;
        return;
    }
    delayedSamples[0] = delayedSamples[1];
    delayedSamples[1] = ((int16_t)lastFiveSamples.dequeue() * sample ) >> 2;
    lowPassOut = (lowPassOut >> 1) + delayedSamples[0] + delayedSamples[1];
    if(debugRx) {
        if(sizeDemodSamples < SAMPLES_IN_CAP) {
            demodSamples[sizeDemodSamples] = (lowPassOut > 0) ? 1 : 0;
            sizeDemodSamples++;
        } else {
            sizeDemodSamples = 0;
            demodSamples[sizeDemodSamples] = (lowPassOut > 0) ? 1 : 0;
        }
    }
    lastFiveSamples.enqueue(sample);
    sampledBitStream <<= 1;
    sampledBitStream |= (lowPassOut > 0) ? 1 : 0;
    if(signalTransitioned(sampledBitStream)) {
        if(syncCounter > SYNC_ADJUST_THRESHOLD) {
            syncCounter+=SYNC_ADJUST;
        } else {
            syncCounter-=SYNC_ADJUST;
        }
    }
    
    syncCounter+=SYNC_SAMPLE_INCREMENT;
    
    if(syncCounter >=RECORD_BIT_THRESHOLD) {
        syncCounter-=RECORD_BIT_THRESHOLD;
        demodulatedBitStream <<=1;
        uint8_t lastThreeBits = (sampledBitStream & B00000111);  //take only the last three bits and leave all others 0
        if (lastThreeBits == B00000111 ||
            lastThreeBits == B00000110 ||
            lastThreeBits == B00000101 ||
            lastThreeBits == B00000011)   
        { 
            demodulatedBitStream |= 1; //the recorded bit is a 1; enqueue it into the stream
            parseNRZToDataBits(1);
        } else {
            parseNRZToDataBits(0);
        }
    }
}

void transmitAPRSPacket(String &data) {
    if(!txing && !rxing) {
        while(!txBuffer.isEmpty()) {
            txBuffer.dequeue();
        }   
        transmissionProtocolInit();
        enqueueData();
        transmissionProtocolAppend();
        txVariablesInit();
        if(debugTx) { 
            Serial.println("PTT Turned low");
        }
        digitalWrite(PTT,LOW);
        delay(PTT_ON_DELAY); //wait for radio to key
        txing = true; //allow the TX ISR to run and transmit the data
    } else {
        if(debug) {
            Serial.println("Transmission aborted: Currently transmitting or receiving");
        }
    }
}

void transmissionProtocolInit() {
    attachHDLCFlag();
    attachSSIDs();
    txBuffer.enqueue(CONTROL_FIELD_BYTE);
    txBuffer.enqueue(PROTOCOL_ID_BYTE);
    txBuffer.enqueue(INFORMATION_FIELD_BYTE);
    if(debugTx) {
        Serial.println("Packet header appended");
    }
}

void transmissionProtocolAppend() {
    int FSCBits = calculateFCS(dataBuffer);
    attachFCS(FSCBits);
    attachHDLCFlag();
    if(debugTx) {
        Serial.println("Packet footer appended");
    }
}

void attachHDLCFlag() {
    txBuffer.enqueue(B01111110); // 0x7E, the HDLC Flag. This wraps all transmissions to indicate that a packet is coming/finished.
}

void attachSSIDs() {
      //todo: eventually reference a config file
    for(byte i = 0; i < DESTINATION_ADDRESS.length(); i++) {
        txBuffer.enqueue(DESTINATION_ADDRESS.charAt(i));
    }
    for(byte j = 0; j < SOURCE_ADDRESS.length(); j++ ) {
        txBuffer.enqueue(SOURCE_ADDRESS.charAt(j));
    }
    for(byte k = 0; k < DIGIPEATER_PATH.length(); k++) {
        txBuffer.enqueue(DIGIPEATER_PATH.charAt(k));
    }
}

int calculateFCS(String packet) {
    int crcRemainder = 0;
    uint8_t intermediateCalc = 0;
    for(int dataIndex = 0; dataIndex < packet.length(); dataIndex++) {
        intermediateCalc = packet[dataIndex] ^ (crcRemainder >> (CRC_LENGTH - 8));
        crcRemainder = crcTable[intermediateCalc] ^ (crcRemainder << 8);
    }
    return crcRemainder;
}

void attachFCS(const int FCSBits) {
    if(debug) {
        int copynum = FCSBits;
        Serial.println("FCS split parsing");
        Serial.print(copynum, BIN);
        Serial.println();
        Serial.print((int8_t) FCSBits >> 8, BIN);
        Serial.println();
        Serial.print((int8_t) FCSBits & 0xFF,BIN);
        Serial.println();
    }
    txBuffer.enqueue((char) FCSBits >> 8);
    txBuffer.enqueue((char) FCSBits & 0xFF);
}

void enqueueData() {
    for(int i = 0; i < messageTx.length(); i++) {
        txBuffer.enqueue(messageTx[i]);
    }
    if(debugTx) {
        Serial.println("Data enqueued into transmission buffer");    
    }
}

void txVariablesInit() {
    byteIndex = 0;
    bitIndex = 0;
    currentPhase = 0;
    freq = MARK_FREQ;
    increment = MARK_INCREMENT;
}

void parseNRZToDataBits(byte b) {
    if(firstBit) {
        if(b == 0) {
            dataBits |=0;
            firstBit = false;
            bitsRead++;
        }
    } else {
        dataBits<<=1;
        if(consecutiveOnes == 5) {
          //consecutiveOnes will be set to 0 below
            demodulatedBitStream>>=1;
            dataBits>>=1;
            bitsRead--;
        }
        if(!lastBitsDiffer(demodulatedBitStream,b)) {
            dataBits |= 1;
            consecutiveOnes++;
            if(debugRx) {
               if(sizeBitsIn < BITS_IN_CAP) {
                   bitsIn[sizeBitsIn] = 1;
                   sizeBitsIn++;
               } else {
                   sizeBitsIn = 0;
                   bitsIn[sizeBitsIn] = 1;
               }
            }   
        } else {
            consecutiveOnes = 0;
            if(debugRx) {
                if(sizeBitsIn < BITS_IN_CAP) {
                    bitsIn[sizeBitsIn] = 0;
                    sizeBitsIn++;
                } else {
                    sizeBitsIn = 0;
                    bitsIn[sizeBitsIn] = 0;
                }
            }
        }
        bitsRead++;
    }
    if(bitsRead == 8) {
        bitsRead = 0;
        if(dataBits == HDLC_FLAG) {
            if(!rxing || debug) {
              //Synchronize with the transmission
                bitIndex = 0;
                byteIndex = 0;
            } else {
                //transmission has completed
                incomingTransmission+=HDLC_FLAG;
                gotData = true;
            }
            if(!debug) {
                rxing = !rxing;
            }
        } else {
            if(dataBits==HDLC_RESET) {
                rxing = false;
                bitsRead = 0;
                dataBits = B11111111;
            }
        }
        if(rxing) {
            //the money line
            incomingTransmission+=(char)dataBits;
        }
    }
}

bool checkIntegrity(String packet) {
    char fcsFirstEightBits = packet[packet.length()-PACKET_FOOTER_LENGTH];
    char fcsLastEightBits = packet[packet.length()-2];
    int frameCheckSeq = (int)fcsFirstEightBits << 8 + fcsLastEightBits;
    if(calculateFCS( packet.substring(0,packet.length()-PACKET_FOOTER_LENGTH) ) == frameCheckSeq) {
        return true;
    }
    return false;
}

void debugTransmit() {
    transmitAPRSPacket(messageTx);
//      analogWriteTone(micPin,2200,2000);
//      analogWriteTone(micPin,1200,2000);
//      analogWrite(micPin,128);
    int sizeSamplesOutCopy = sizeSamplesOut;
    while(txing) { //wait until transmission is complete, after which it will be turned false
//        if(deqSamples > sizeSamplesOutCopy) {
//            while (deqSamples < SAMPLES_IN_CAP) {
//                Serial.println(samplesOut[deqSamples]);
//                deqSamples++;
//            }
//                deqSamples=0;
//            }        
//        while(deqSamples < sizeSamplesOutCopy ) {
//            Serial.println(samplesOut[deqSamples]);
//            deqSamples++; 
//        }
    }
//    int count8 = 0;
//    Serial.println("Bits Sent:");
//    while(!bitsOutSize==0) {
//        if(count8 == 8) {
//          count8 = 0;
//          Serial.print("|");
//        }
//        Serial.print(bitsOut[bitsOutSize]);
//        count8++;
//    }
    
    Serial.println("done");
    delay(4000);
}

void debugReceive() {    
//    Serial.println("start");  
//    int t0 = millis();
//    Serial.println(t0);
//    rxing = true;
//    while(rxing) {
//        if(millis() > t0 + DEBUG_RECEIVE_DURATION) {
//            rxing = false;
//        }
//    }
        int sizeSamplesInCopy = sizeSamplesIn;
        int sizeDemodSamplesCopy = sizeDemodSamples;
        int sizeBitsInCopy = sizeBitsIn;
//        Serial.println("Samples Received:");
        if(deqSamples > sizeSamplesInCopy) {
          
            while (deqSamples < SAMPLES_IN_CAP) {
                Serial.println(samplesIn[deqSamples]);
                deqSamples++;
            }
            deqSamples=0;
        }        
        while(deqSamples < sizeSamplesInCopy ) {
            Serial.println(samplesIn[deqSamples]);
            deqSamples++; 
        }
//        Serial.println();
//        Serial.println("Demodulated Samples:");
        if(deqSamples > sizeDemodSamplesCopy) {
            while(deqDemodSamples < SAMPLES_IN_CAP) {
//                Serial.print(demodSamples[deqDemodSamples]);
                deqDemodSamples++;
            }
            deqDemodSamples = 0;
        }
        while(deqDemodSamples < sizeDemodSamplesCopy) {
//            Serial.print(demodSamples[deqDemodSamples]);
              deqDemodSamples++;
        }

//        Serial.println();
//        Serial.println("Bits Received:");
        if(deqBits > sizeBitsInCopy) {
            while(deqBits < BITS_IN_CAP) {
                if(count8 == 8) {
                    count8 = 0;
//                    Serial.print("|");
                }
//                Serial.print(bitsIn[deqBits],BIN);
                count8++;
                deqBits++;
            }
            deqBits = 0;
        }
        while(deqBits < sizeBitsInCopy) {
            if(count8 == 8) {
                count8 = 0;
//                Serial.print("|");
            }
//            Serial.print(bitsIn[deqBits],BIN);
            count8++;
            deqBits++;
        }
//        Serial.println();
//        Serial.println(incomingTransmission);
//        Serial.println(sizeSamplesIn);
//        Serial.println(sizeDemodSamples);
//        Serial.println(sizeBitsIn);
        incomingTransmission = "";
    delay(100);
//    Serial.println("done");
}
