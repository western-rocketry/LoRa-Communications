#include <SPI.h>              // include libraries for LoRa Communications
#include <LoRa.h>             //
#include <Wire.h>                 //Sensor Data Transmission
#include "decode.h"


//Take this many samples to average out. Higher values will lower resolution, but give smoother data
#define AVERAGE 127
#define PREPLENGTH 5000      //Duration in which the rocket is in prep mode before switching to normal

#define csPin 10              // LoRa radio chip select
#define resetPin 6            // LoRa radio reset
#define irqPin 1              // change for your board; must be a hardware interrupt pin
#define MPU 0x68
#define localAddress 0x57     // address of this device (ASCII W)
#define destination 0x45      // destination to send to (ASCII E)

const int interval = 250;     // interval between LoRa transmissions in milliseconds

String outgoing;              // outgoing message
byte msgCount = 0;            // count of outgoing messages
byte mode = 0;
long lastSendTime = 0;        // last send time
int16_t AAcX,AAcY,AAcZ,ATmp,AGyX,AGyY,AGyZ;
float realTemp;

void setup() {
  Serial.begin(19200);                 
  while (!Serial);

  //initalize LoRa communications
  LoRa.setPins(csPin, resetPin, irqPin);    // override the default CS, reset, and IRQ pins (optional)
  if (!LoRa.begin(915E6)) {                 // initialize at 915 MHz
    Serial.println(F("LoRa initialization failed, retrying...\n\n")); delay(5000);
    setup();
  } Serial.println(F("LoRa initialized successfully"));
  //Initalize gyro/accelerometer
  Wire.begin();
  Wire.setClock(400000);
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  Wire.write(0x00);    
  Wire.endTransmission(true);
}

void loop() {
  switch(mode){
    case 0:{ //curly bracket is to redeclare string: "message" in its own scope so the IDE doesnt get mad
      if(millis()>=PREPLENGTH){
        mode=2;
      }else{
        byte accelOK = gyroFunctional() ? 0 : 255;
        byte message = {char(accelOK)};
        sendMessage(0, message, 1); //change to proper frame later
      }
      break;
    }case 1:{ //curly bracket is to redeclare string: "message" in its own scope so the IDE doesnt get mad
      Wire.beginTransmission(MPU);
      Wire.write(0x3B);  
      Wire.endTransmission(false);
      Wire.requestFrom(MPU,14,true);  
      AAcX+=(Wire.read()<<8|Wire.read());
      AAcY+=(Wire.read()<<8|Wire.read());
      AAcZ+=(Wire.read()<<8|Wire.read());
      ATmp+=(Wire.read()<<8|Wire.read());
      AGyX+=(Wire.read()<<8|Wire.read());
      AGyY+=(Wire.read()<<8|Wire.read());
      AGyZ+=(Wire.read()<<8|Wire.read());
      byte arr[] = {
        char(AAcX>>8), 
        char(AAcX%256),
        char(AAcY>>8),
        char(AAcY%256),
        char(AAcZ>>8),
        char(AAcZ%256),
        char((AAcX+AAcY+AAcZ)%256), //Checksum
        char(AGyX>>8),
        char(AGyX%256),
        char(AGyY>>8),
        char(AGyY%256),
        char(AGyZ>>8),
        char(AGyZ%256),
        char((AGyX+AGyY+AGyZ)%256) //checksum
      };
      sendMessage(1, arr, sizeof(arr));                   //max 255 bytes, 4 reserved for frame
      AAcX = AAcY = AAcZ = ATmp = AGyX = AGyY = AGyZ = 0;
      break;
    }case 2:{ //curly bracket is to redeclare string: "message" in its own scope so the IDE doesnt get mad
      //Accelerometer & Gyroscope
      for(int i=0;i<AVERAGE;i++){
        Wire.beginTransmission(MPU);
        Wire.write(0x3B);  
        Wire.endTransmission(false);
        Wire.requestFrom(MPU,14,true);  
        AAcX+=(Wire.read()<<8|Wire.read())/AVERAGE;
        AAcY+=(Wire.read()<<8|Wire.read())/AVERAGE;
        AAcZ+=(Wire.read()<<8|Wire.read())/AVERAGE;
        ATmp+=(Wire.read()<<8|Wire.read())/AVERAGE;
        AGyX+=(Wire.read()<<8|Wire.read())/AVERAGE;
        AGyY+=(Wire.read()<<8|Wire.read())/AVERAGE;
        AGyZ+=(Wire.read()<<8|Wire.read())/AVERAGE;
      }
      byte arr[] = {
        char(AAcX>>8), 
        char(AAcX%256),
        char(AAcY>>8),
        char(AAcY%256),
        char(AAcZ>>8),
        char(AAcZ%256),
        char((AAcX+AAcY+AAcZ)%256), //Checksum
        char(AGyX>>8),
        char(AGyX%256),
        char(AGyY>>8),
        char(AGyY%256),
        char(AGyZ>>8),
        char(AGyZ%256),
        char((AGyX+AGyY+AGyZ)%256), //checksum
        char(ATmp)
      };
//      Serial.println(String(AAcX) +" "+ String(AAcY) +" "+ String(AAcZ) +" " + String(AGyX) +" "+ String(AGyY) +" "+ String(AGyZ) );
      sendMessage(2, arr, sizeof(arr));                   //max 255 bytes, 4 reserved for frame
      //Serial plotter
      //Serial.println(String(AAcX) +"\t"+ String(AAcY) +"\t"+ String(AAcZ));
      AAcX = AAcY = AAcZ = ATmp = AGyX = AGyY = AGyZ = 0;
      break;
    }
  }
//  delay(500);
  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());
}

bool gyroFunctional(){
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,14,true);  
    AAcX=Wire.read()<<8|Wire.read();    
    AAcY=Wire.read()<<8|Wire.read();  
    AAcZ=Wire.read()<<8|Wire.read();   
    ATmp=Wire.read()<<8|Wire.read(); 
    AGyX=Wire.read()<<8|Wire.read();  
    AGyY=Wire.read()<<8|Wire.read();  
    AGyZ=Wire.read()<<8|Wire.read();      
    if(AAcX==-1 && AAcY==-1 && AAcZ==-1) return(false);
    AAcX = AAcY = AAcZ = ATmp = AGyX = AGyY = AGyZ = 0;
    return(true);
}

void sendMessage(byte msgmode, byte outgoing[], byte outgoingSize) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address      
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgmode);                  // Message frame type
  LoRa.write(msgCount);               // add message ID
  uint32_t timeM = millis();
  LoRa.write(timeM>>24);                 // Time from when LoRa started
  LoRa.write(timeM>>16%256);             // Time from when LoRa started
  LoRa.write(timeM>>8%256);              // Time from when LoRa started
  LoRa.write(timeM%256);                 // Time from when LoRa started
  LoRa.write(outgoingSize);             // add payload length
  for(int i=0; i<outgoingSize;i++)
    LoRa.write(outgoing[i]);            // add payload
  LoRa.endPacket();                     // finish packet and send it
  //Serial.println("Sent message: " + String(msgCount) + " with length " + String((outgoingSize+9)) + " at time: " + String(timeM)); //9 includes 1 for destination, local addr, msg mode, msg count, msg length, and 4 for millis()
  msgCount++;
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return
  // decode frame
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length
  String incoming = "";
  //Read message
  while (LoRa.available()) incoming += (char)LoRa.read();
  
  if (incomingLength != incoming.length()) {   // check length for error
    //Serial.println("error: message length does not match length");return;
  }
  if (recipient != localAddress && recipient != 0xFF) { // if the recipient isn't this device or broadcast,
    //Serial.println("Packet recieved, incorrect address");return;
  }
  
//  Serial.println("Message ID: " + String(incomingMsgId));
//  Serial.println("Message length: " + String(incomingLength));
//  Serial.println("Recieved from " +String(sender,HEX) + " to " + String(recipient,HEX) + ":\n" + incoming);
//  Serial.print("Signal Strength: " + String((LoRa.packetRssi()/-120)*100) + "% ");
//  Serial.println("Signal Noise: " + String(((LoRa.packetSnr()-10)/30)*100) + "%");
//  Serial.println();
}
