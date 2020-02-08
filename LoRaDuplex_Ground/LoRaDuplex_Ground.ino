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
    Serial.println("error: message length does not match length");return;
  }
  if (recipient != localAddress && recipient != 0xFF) { // if the recipient isn't this device or broadcast,
    Serial.println("Packet recieved, incorrect address");return;
  }
  
//  Serial.println("Message ID: " + String(incomingMsgId));
//  Serial.println("Message length: " + String(incomingLength));
//  Serial.println("Recieved from " +String(sender,HEX) + " to " + String(recipient,HEX) + "\n\n");
  Serial.println(incoming);
//  Serial.print("Signal Strength: " + String((LoRa.packetRssi()/-120)*100) + "% ");
//  Serial.println("Signal Noise: " + String(((LoRa.packetSnr()-10)/30)*100) + "%");
//  Serial.println();
}
