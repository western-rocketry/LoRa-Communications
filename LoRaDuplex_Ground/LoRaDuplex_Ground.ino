#include <SPI.h>              // include libraries for LoRa Communications
#include <LoRa.h>             //
#include <Wire.h>                 //Sensor Data Transmission
#include "decode.h"

#define PRINTTOSERIAL

#define csPin 10              // LoRa radio chip select
#define resetPin 6            // LoRa radio reset
#define irqPin 1              // change for your board; must be a hardware interrupt pin
#define localAddress 0x45     // address of this device (ASCII W)
#define destination 0x57      // destination to send to (ASCII E)
#define MPU 0x68

uint16_t msgCount = 0;

void setup() {
  Serial.begin(115200);                 
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
  while (LoRa.available()){      //read message
    byte temp = LoRa.read();
    msg[msgCount] = temp; 
    msgCount++;
  }
  switch(mode){
    case 0:
      #ifdef PRINTTOSERIAL
      Serial.println("\n\nMode Prep");
      Serial.println("Time " + String(rocketTime));
      //Accelerometer/Gyro
      (msg[0]==255) ? Serial.println(F("Acc/Gyro OK")) : Serial.println("Acc/Gyro ERROR " + String(msg[0]));
      //GPS
      if(msg[1]==255) Serial.println(F("GPS OK"));
      else if (msg[1]==254) Serial.println("GPS Fix Error");
      else Serial.println("GPS ERROR: " + String(msg[0]));
      //Temperature Sensor
      (msg[2]==255) ? Serial.println(F("TEMP1 OK")) : Serial.println("TEMP1 ERROR " + String(msg[2]));
      #endif
      break;
    case 1:
      #ifdef PRINTTOSERIAL
      Serial.println("\n\nMode Launch");
      #endif
    case 2:{
      #ifdef PRINTTOSERIAL
      if(mode==2) Serial.println("\n\nMode Normal");
      Serial.print("Message length: "); Serial.println(len+9);
      Serial.print("Message destination: "); Serial.println(dest,HEX);
      Serial.print("Message sender: "); Serial.println(send,HEX);
      Serial.print("Message mode: "); Serial.println(mode);
      Serial.print("Message count id: "); Serial.println(count);
      Serial.println("Time " + String(rocketTime));
      #endif

      //******************************************************************//
      //                        Data Parsing                              //
      //******************************************************************//
      //Gyroscope and Accelerometer
      int16_t acx,acy,acz,gyx,gyy,gyz,tmp;
      byte chk0,chk1;
      parseData16(acx,msg[0],msg[1]);
      parseData16(acy,msg[2],msg[3]);
      parseData16(acz,msg[4],msg[5]);
      parseData8(chk0,msg[6]);
      parseData16(gyx,msg[7],msg[8]);
      parseData16(gyy,msg[9],msg[10]);
      parseData16(gyz,msg[11],msg[12]);
      parseData8(chk1,msg[13]);
      parseData16(tmp,msg[14],msg[15]);
      //GPS
      floatunion_t alt,lat,lon,knots,angle,vdop,hdop,pdop;
      byte fix,num,fail,chk2,chk3,chk4,chk5;
      parseData8(fix,msg[16]);
      parseData8(num,msg[17]);
      parseData8(fail,msg[18]);
      parseFloat32(alt,msg[19],msg[20],msg[21],msg[22]);
      parseData8(chk2,msg[23]);
      parseFloat32(lat,msg[24],msg[25],msg[26],msg[27]);
      parseFloat32(lon,msg[28],msg[29],msg[30],msg[31]);
      parseData8(chk3,msg[32]);
      parseFloat32(knots,msg[33],msg[34],msg[35],msg[36]);
      parseFloat32(angle,msg[37],msg[38],msg[39],msg[40]);
      parseData8(chk4,msg[41]);
      parseFloat32(vdop,msg[42],msg[43],msg[44],msg[45]);
      parseFloat32(hdop,msg[46],msg[47],msg[48],msg[49]);
      parseFloat32(pdop,msg[50],msg[51],msg[52],msg[53]);
      parseData8(chk5,msg[54]);
      //Temp
      floatunion_t temp1;
      byte chk6;
      parseFloat32(temp1,msg[55],msg[56],msg[57],msg[58]);


      
      #ifdef PRINTTOSERIAL
      ///////////////////////////// Begin sending parsed Data ///////////////////////////////
      //Gyro and Accelerometer
      Serial.println("GyX " + String(gyx) + 
                   "\nGyY " + String(gyy) + 
                   "\nGyZ " + String(gyz));
      if(byte((acx+acy+acz)%256)!=byte(chk0)) Serial.println(F("CS FAILED - GYRO"));
      Serial.println("AcX " + String(acx) + 
                   "\nAcY " + String(acy) + 
                   "\nAcZ " + String(acz));
      if(byte((gyx+gyy+gyz)%256)!=byte(chk1)) Serial.println(F("CS FAILED - ACCELEROMETER"));
      //GPS
      String gpsConnected = fix ? "True" : "False"; 
      Serial.println("Connection "+gpsConnected+
                   "\nSatellites "+String(num)+
                   "\nFails " + String(fail)+
                   "\nAltitude " + String(alt.num));  
      if(byte((msg[16]+msg[17]+msg[18]+msg[19]+msg[20]+msg[21]+msg[22])%256)!=byte(chk2)) Serial.println(F("CS FAILED - GPS ALTITUDE/CONNECTION")); 
      Serial.println("Latitude "+String(lat.num)+
                  "\nLongitude "+String(lon.num));
      if(byte((msg[24]+msg[25]+msg[26]+msg[27]+msg[28]+msg[29]+msg[30]+msg[31])%256)!=byte(chk3)) Serial.println(F("CS FAILED - GPS COORDINATES"));   
      Serial.println("Speed (Knots) "+String(knots.num)+
                   "\nAngle from N "+String(angle.num));           
      if(byte((msg[33]+msg[34]+msg[35]+msg[36]+msg[37]+msg[38]+msg[39]+msg[40])%256)!=byte(chk4)) Serial.println(F("CS FAILED - SPEED/ANGLE")); 
      Serial.println("Vertical DoP "+String(hdop.num)+
                 "\nHorizontal DoP "+String(vdop.num)+
                 "\nPositional DoP "+String(pdop.num));
      if(byte((msg[42]+msg[43]+msg[44]+msg[45]+msg[46]+msg[47]+msg[48]+msg[49]+msg[50]+msg[51]+msg[52]+msg[53])%256)!=byte(chk5)) Serial.println(F("CS FAILED - DILUTION OF PERCISION")); 
      Serial.println("Temp1: "+String(temp1.num));             




      //RSSI/SNR Footer
      Serial.println("Temp " + String(float(tmp)/340.0 + 36.53));
      Serial.println("RSSI " + String((LoRa.packetRssi()/-120)*100) + "%");
      Serial.println("SNR: " + String(((LoRa.packetSnr()-10)/30)*100) + "%");
      #endif
      //Serial plotter
      //Serial.println("AcX " + String(acx) + "\tAcY " + String(acy) + "\tAcZ " + String(acz) + "\tTemp" +String(float(tmp)/340.0 + 36.53) );
      break;
    }default:{
      #ifdef PRINTTOSERIAL
      Serial.println(mode);
      #endif
    }
  }
}
