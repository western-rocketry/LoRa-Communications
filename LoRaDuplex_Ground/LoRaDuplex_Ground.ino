#include <SPI.h>              // include libraries for LoRa Communications
#include <LoRa.h>             //
#include <Wire.h>                 //Sensor Data Transmission
#include "decode.h"

#define csPin 10              // LoRa radio chip select
#define resetPin 6            // LoRa radio reset
#define irqPin 1              // change for your board; must be a hardware interrupt pin
#define localAddress 0x45     // address of this device (ASCII W)
#define destination 0x57      // destination to send to (ASCII E)
#define MPU 0x68

uint16_t msgCount = 0;

void setup() {
  Serial.begin(19200);                 
  while (!Serial);
  //initalize LoRa communications
  LoRa.setPins(csPin, resetPin, irqPin);    // override the default CS, reset, and IRQ pins (optional)
  if (!LoRa.begin(915E6)) {                 // initialize at 915 MHz
    Serial.println(F("LoRa initialization failed, retrying...\n\n")); delay(5000);
    setup();
  } Serial.println(F("LoRa initialized successfully"));
  delay(500);
}

void loop() {
  onReceive(LoRa.parsePacket());
}

void onReceive(int packetSize){
  if (packetSize == 0) return;  
  byte dest = LoRa.read();      //destination, should be us
  byte send = LoRa.read();      //sender, should be the rocket
  byte mode = LoRa.read();      //frame type
  byte count = LoRa.read();     //Rocket msg ID counter
  byte msgCount = 0;            //Ground msg ID counter
  uint32_t rocketTime = 0;          //rocket time in ms
  rocketTime = parseTime(LoRa.read(),LoRa.read(),LoRa.read(),LoRa.read());
  byte len = LoRa.read();       //message length
  byte msg[len];                //message
  Serial.print("Message length: "); Serial.println(len+9);
  Serial.print("Message destination: "); Serial.println(dest,HEX);
  Serial.print("Message sender: "); Serial.println(send,HEX);
  Serial.print("Message mode: "); Serial.println(mode);
  Serial.print("Message count id: "); Serial.println(count);
  while (LoRa.available())      //read message
    msg[msgCount++] += (char)LoRa.read(); 
  switch(mode){
    case 0:
      Serial.println("Mode Prep");
      Serial.println("Time " + String(rocketTime));
      (msg[0]==32) ? Serial.println(F("Acc/Gyro OK")) : Serial.println("Acc/Gyro ERROR" + String(msg[0]));
      break;
    case 1:
      Serial.println("Mode Launch");
    case 2:
      if(mode==2) Serial.println("Mode Normal");
      Serial.println("Time " + String(rocketTime));
      int16_t acx,acy,acz,gyx,gyy,gyz;
      byte chk0,chk1;
      parseData16(gyx,msg[0],msg[1]);
      parseData16(gyy,msg[2],msg[3]);
      parseData16(gyz,msg[4],msg[5]);
      parseData8(chk0,msg[6]);
      parseData16(acx,msg[7],msg[8]);
      parseData16(acy,msg[9],msg[10]);
      parseData16(acz,msg[11],msg[12]);
      parseData8(chk1,msg[13]);
      Serial.print(msg[0],HEX);
      Serial.print(msg[1],HEX);
      Serial.print(msg[2],HEX);
      Serial.println(msg[3],HEX);
      Serial.println("GyX " + String(gyx) + "\nGyY " + String(gyy) + "\nGyZ " + String(gyz));
      if(byte((gyx+gyy+gyz)%256)!=byte(chk0)) Serial.println(F("CS FAILED"));
      Serial.println("AcX " + String(acx) + "\nAcY " + String(acy) + "\nAcZ " + String(acz));
      if(byte((acx+acy+acz)%256)!=byte(chk1)) Serial.println(F("CS FAILED"));
      Serial.println();
      break;
     default:
       Serial.println(mode);
  }
//  Serial.println("RSSI " + String((LoRa.packetRssi()/-120)*100) + "%");
//  Serial.println("SNR: " + String(((LoRa.packetSnr()-10)/30)*100) + "%");
}
