#include <SPI.h>              // include libraries for LoRa Communications
#include <LoRa.h>             //
#include <Wire.h>                 //Sensor Data Transmission
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>     //GPS requires a serial port
#include "protocol.h"

//Take this many samples to average out. Higher values will lower resolution, but give smoother data
#define AVERAGE 127
#define PREPLENGTH 15000      //Duration in which the rocket is in prep mode before switching to normal

#define csPin 10              // LoRa radio chip select
#define resetPin 6            // LoRa radio reset
#define irqPin 1              // change for your board; must be a hardware interrupt pin
#define MPU 0x68
#define localAddress 0x57     // address of this device (ASCII W)
#define destination 0x45      // destination to send to (ASCII E)
#define GPSECHO  true

SoftwareSerial mySerial(4, 3);
Adafruit_GPS GPS(&mySerial);

const int interval = 250;     // interval between LoRa transmissions in milliseconds

String outgoing;              // outgoing message
byte msgCount = 0;            // count of outgoing messages
byte mode = 0;
long lastSendTime = 0;        // last send time
int16_t AAcX,AAcY,AAcZ,ATmp,AGyX,AGyY,AGyZ = 0;
float realTemp;

void setup() {
  Serial.begin(115200);               
  while (!Serial);
  //initalize LoRa communications
  LoRa.setPins(csPin, resetPin, irqPin);    // override the default CS, reset, and IRQ pins (optional)
  if (!LoRa.begin(915E6)) {                 // initialize at 915 MHz
    Serial.println(F("LoRa initialization failed, retrying...\n\n")); delay(5000);
    setup();
  } Serial.println(F("LoRa initialized successfully"));
  Initialize gyro/accelerometer
  Wire.begin();
  Wire.setClock(400000);
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  Wire.write(0x00);    
  Wire.endTransmission(true);
  //Initialize GPS
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  mySerial.println(PMTK_Q_RELEASE);
}

void loop() {
  char c = GPS.read();
  switch(mode){
    case 0:{ //curly bracket is to redeclare string: "message" in its own scope so the IDE doesnt get mad
      if(millis()>=PREPLENGTH){
        mode=2;
      }else{
        byte accelOK = gyroFunctional();
        byte gpsOK = gpsFunctional(); 
        byte message[] = {accelOK, gpsOK};
        sendMessage(0, message, 2); //change to proper frame later
        Serial.println("Accel: " + String(accelOK));
        Serial.println("GPS: " + String(gpsOK));
        delay(1000);
      }
      break;
    }case 1:{ 
      addData(MPU, AAcX, AAcY, AAcZ, ATmp, AGyX, AGyY, AGyZ);
      byte arr[16];
      encodeData(arr,AAcX,AAcY,AAcZ,ATmp,AGyX,AGyY,AGyZ);
      sendMessage(1, arr, sizeof(arr));                   //max 255 bytes, 4 reserved for frame
      AAcX = AAcY = AAcZ = ATmp = AGyX = AGyY = AGyZ = 0;
      break;
    }case 2:{ 
      //Accelerometer & Gyroscope
      addData(MPU, AAcX, AAcY, AAcZ, ATmp, AGyX, AGyY, AGyZ, AVERAGE);
      byte arr[16];
      encodeData(arr,AAcX,AAcY,AAcZ,ATmp,AGyX,AGyY,AGyZ);
      sendMessage(2, arr, sizeof(arr));                   //max 255 bytes, 4 reserved for frame
      //Serial monitoring
      //Serial.println(String(AAcX) +"\t"+ String(AAcY) +"\t"+ String(AAcZ));
      Serial.println(String(AAcX) +" "+ String(AAcY) +" "+ String(AAcZ) +" " + String(AGyX) +" "+ String(AGyY) +" "+ String(AGyZ) );
      AAcX = AAcY = AAcZ = ATmp = AGyX = AGyY = AGyZ = 0;
      break;
    }
  }
  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());
}

byte gyroFunctional(){
    addData(MPU, AAcX, AAcY, AAcZ, ATmp, AGyX, AGyY, AGyZ);    
    if(AAcX==-1 && AAcY==-1 && AAcZ==-1) return(0);
    AAcX = AAcY = AAcZ = ATmp = AGyX = AGyY = AGyZ = 0;
    return(255);
}

byte gpsFunctional(){
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      gpsFunctional();// we can fail to parse a sentence in which case we should just wait for another
  }
  Serial.print("\nTime: ");
  if (GPS.hour < 10) { Serial.print('0'); }
  Serial.print(GPS.hour, DEC); Serial.print(':');
  if (GPS.minute < 10) { Serial.print('0'); }
  Serial.print(GPS.minute, DEC); Serial.print(':');
  if (GPS.seconds < 10) { Serial.print('0'); }
  Serial.print(GPS.seconds, DEC); Serial.print('.');
  if (GPS.milliseconds < 10) {
    Serial.print("00");
  } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
    Serial.print("0");
  }
  if(GPS.seconds==GPS.milliseconds && GPS.milliseconds==0){
    Serial.println(String(GPS.seconds) + " " + String(GPS.milliseconds));
    if((byte)GPS.fix==1){
      return(1); //no Signal
    }else{
      return(0); //no time movement and no signal
    } 
  }else{
    return(255);
  }
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
