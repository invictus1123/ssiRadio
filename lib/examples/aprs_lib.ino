#include "afsk.h"
#include "aprs.h"
#include "aprs_global.h"
#include "dra818v.h"
//#if USE_HW_SERIAL ==false
////SoftwareSerial radioSerial(A8,A9);
//#endif
//Bug: SSIDs have to be defined in the main code because multiple definition compiler errors are thrown if put in a .h file
static const uint8_t n_ssids = 3;
SSID myssids[n_ssids] = {
  {(char*) "APRS", 0},
  {(char*) "KM6HBK",11},
  {(char*) "WIDE2-1",11}
};
uint8_t dayOfMonth = 0; 
uint8_t hour = 0;
uint8_t minute = 0;
float lat = 0;
float lon = 0; // degrees
float altitude = 0;// meters
uint16_t heading = 0; // degrees
float speed = 0;
String msg1 = "Do not go gentle into that good night, ";
String msg2 = "Old age should burn and rave at close of day;";
String msg3 = "Rage, rage against the dying of the light.";
long randomNum = 0;
DRA818V radio(PTT_PIN,AUDIO_PIN,MIC_PIN,DRATX,DRARX);
APRS aprs(&radio, myssids,n_ssids);
void setup() {  
  Serial.begin(9600);
//  pinMode(A0,OUTPUT);
//  digitalWrite(A0,LOW);
  delay(750);
  radio.init();
}

void loop() {
  Serial.println("Start");
  aprs.sendPacketGPS(dayOfMonth,hour,minute,lat,lon,altitude,heading,speed,"ABCDEFGHIJKLMNOPQRSTUVWXYZ");
//  uint8_t myData[4] = {(uint8_t)(randomNum>>24),(uint8_t)(randomNum>>16),(uint8_t) (randomNum>>8),(uint8_t) (randomNum & 0xFF)};
//  uint8_t chooseStr = random(1,5);
//  String comment = ""; 
//  switch(chooseStr) {
//      case 1: comment = msg1;break;
//      case 2: comment = msg2;break;
//      case 3: comment = msg3;break;
//      default: comment = (String)randomNum; break;
//  };
//  comment = comment + "aljjalsdjlaksdjf;lkj;jasdfj;ajkdfasdfj;j";
//  aprs.sendPacket(myData,comment,sizeof(myData) + comment.length());
  delay(5000);
}
