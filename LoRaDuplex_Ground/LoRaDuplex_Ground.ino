#include <SPI.h>              // include libraries for LoRa Communications
#include <LoRa.h>             //
#include <Wire.h>                 //Sensor Data Transmission

#define csPin 10          // LoRa radio chip select
#define resetPin 6       // LoRa radio reset
#define irqPin 1         // change for your board; must be a hardware interrupt pin
#define localAddress 0x45     // address of this device
#define destination 0x57      // destination to send to
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
}

void loop() {
  onReceive(LoRa.parsePacket());
}

void sendMessage(String outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount++);               // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}

void onReceive(int packetSize){
  if (packetSize == 0) return;  
  byte dest = LoRa.read();
  byte send = LoRa.read();
  byte mode = LoRa.read();
  byte count = LoRa.read();
  uint32_t rocketTime = LoRa.read()<<24 + LoRa.read()<<16 + LoRa.read()<<8 + LoRa.read();
  byte len = LoRa.read();
  byte msg[len];
  byte msgCount = 0;
  while (LoRa.available()) msg[msgCount++] = (char)LoRa.read();
  switch(mode){
    case 0:
      Serial.println("Mode Prep");
      Serial.println("Time " + String(rocketTime));
      if(msg[0]==255) Serial.println(F("Acc/Gyro OK"));
      else if(msg[0]==0) Serial.println(F("Acc/Gyro ERROR"));
      break;
    case 1:
      Serial.println("Mode Launch");
    case 2:
      if(mode==2) Serial.println("Mode Normal");
      Serial.println("Time " + String(rocketTime));
      int16_t acx,acy,acz,gyx,gyy,gyz;
      uint8_t chk0,chk1;
      gyx = msg[0]<<8 + msg[1];
      gyy = msg[2]<<8 + msg[3];
      gyz = msg[4]<<8 + msg[5];
      chk0 = msg[6];
      acx = msg[7]<<8 + msg[8];
      acy = msg[9]<<8 + msg[10];
      acz = msg[11]<<8 + msg[12];
      chk1 = msg[13];
      if((gyx+gyy+gyz)%256==chk0) Serial.println("GyX " + String(gyx) + "\nGyY " + String(gyy) + "\nGyZ " + String(gyz));
      else Serial.println(F("GyX CS FAILED\nGyY CS FAILED\nGyZ CS FAILED"));
      if((acx+acy+acz)%256==chk1) Serial.println("AcX " + String(acx) + "\nAcY " + String(acy) + "\nAcZ " + String(acz));
      else Serial.println(F("AcX CS FAILED\nAcY CS FAILED\nAcZ CS FAILED"));
      break;
  }
  Serial.println("RSSI " + String((LoRa.packetRssi()/-120)*100) + "%");
  Serial.println("SNR: " + String(((LoRa.packetSnr()-10)/30)*100) + "%");
}
