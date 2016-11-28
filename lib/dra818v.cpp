#include "dra818v.h"

DRA818V::DRA818V(uint8_t PTT, uint8_t audioOut, uint8_t mic, uint8_t draTX, uint8_t draRX)
{
    pttPin = PTT;
    pttDelay = PTT_DELAY;
    audioOutPin = audioOut;
    micPin = mic;
    #if USE_HW_SERIAL== true
        radioSerial = &Serial1;
    #else
        radioSerial = new SoftwareSerial(draRX,draTX);
    #endif
}

void DRA818V::init() {
    pinMode(pttPin,OUTPUT);
    pinMode(micPin,OUTPUT); 
    char incomingByte;
    radioSerial->begin(SOFT_SERIAL_BAUD);
    if(!Serial) {
        Serial.begin(SOFT_SERIAL_BAUD);
    }
    digitalWrite(pttPin,LOW);
    delay(200);
    radioSerial->print("AT+DMOCONNECT\r\n");
    digitalWrite(pttPin,HIGH);
    delay(200);
    if(DEBUG) {
        String messageRx = "";
          while (radioSerial->available() > 0) {
          incomingByte = radioSerial->read();
          messageRx = messageRx + incomingByte;
        }
        if(messageRx!="") {
          Serial.print("UART received: ");
          Serial.println(messageRx);
        }
    }
    digitalWrite(pttPin,LOW);
    delay(200);
    configSettings();
    digitalWrite(pttPin,HIGH);
    delay(200);
    if(DEBUG) {
        String messageRx = "";
          while (radioSerial->available() > 0) {
          incomingByte = radioSerial->read();
          messageRx = messageRx + incomingByte;
        }
        if(messageRx!="") {
          Serial.print("UART received: ");
          Serial.println(messageRx);
        }
    }
}

void DRA818V::setPTTDelay(uint16_t delayMs) {
    pttDelay = delayMs;
}

void DRA818V::configSettings() {
//    #if USE_HW_SERIAL ==true
//        digitalWrite(pttPin,HIGH);
//        delay(500);
//        digitalWrite(pttPin,LOW);
//        radioHWSerial.print("AT+DMOSETGROUP=");      
//        radioHWSerial.print(1,1);
//        radioHWSerial.print(",");
//        radioHWSerial.print(144.390,4); 
//        radioHWSerial.print(",");
//        radioHWSerial.print(144.390,4);
//        radioHWSerial.print(",");
//        radioHWSerial.print("0000");
//        radioHWSerial.print(",");
//        radioHWSerial.print(0);
//        radioHWSerial.print(",");
//        radioHWSerial.print("0000");
//        radioHWSerial.print("\r\n"); 
//        delay(500);
//        digitalWrite(pttPin,HIGH);
//        delay(500);
//        digitalWrite(pttPin,LOW);
//        radioHWSerial.print("AT+SETFILTER=1,0,0\r\n");
//    #else
        digitalWrite(pttPin,HIGH);
        delay(500);
        digitalWrite(pttPin,LOW);
        radioSerial->print("AT+DMOSETGROUP=");
        radioSerial->print(CHANNEL_SCAN_BW,1);
        radioSerial->print(",");
        radioSerial->print(APRS_NA_FTX,4);
        radioSerial->print(",");
        radioSerial->print(APRS_NA_FRX,4);
        radioSerial->print(",");
        radioSerial->print(TX_CTCSS);
        radioSerial->print(",");
        radioSerial->print(squelch);
        radioSerial->print(",");
        radioSerial->print(RX_CTCSS);
        radioSerial->print("\r\n");
        delay(500);
        digitalWrite(pttPin,HIGH);
        delay(500);
        digitalWrite(pttPin,LOW);
        radioSerial->print("AT+SETFILTER=1,0,0\r\n");
        digitalWrite(pttPin,HIGH);
//    #endif
}

