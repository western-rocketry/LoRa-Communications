#include "protocol.h"
#include <Arduino.h>
#include <Wire.h>

void parseData8(byte &val, byte b1){
  for(int i=0;i<8;i++){
    bitWrite(val,i,bitRead(byte(b1),i));
  }
}

void parseData16(int16_t &val, byte b1, byte b2){
  for(int i=0;i<8;i++){
    bitWrite(val,8+i,bitRead(b1,i));
    bitWrite(val,i,bitRead(b2,i));
  }
}

void parseData32(int32_t &val, byte b1, byte b2, byte b3, byte b4){
  for(int i=0;i<8;i++){
    bitWrite(val,24+i,bitRead(byte(b1),i));
    bitWrite(val,16+i,bitRead(byte(b2),i));
    bitWrite(val,8+i,bitRead(byte(b3),i));
    bitWrite(val,i,bitRead(byte(b4),i));
  }
}

void printInt(int b){
  for(int i = 15; i >= 0; i--)
    Serial.print(bitRead(b,i));
   Serial.println();
}
void printByte(byte b){
  for(int i = 7; i >= 0; i--)
    Serial.print(bitRead(b,i));
}
void addData(byte addr, int16_t &acx, int16_t &acy, int16_t &acz, int16_t &temp, int16_t &gyx, int16_t &gyy, int16_t &gyz){
  addData(addr,acx,acy,acz,temp,gyx,gyy,gyz,1.0);  
}
void addData(byte addr, int16_t &acx, int16_t &acy, int16_t &acz, int16_t &temp, int16_t &gyx, int16_t &gyy, int16_t &gyz, uint8_t average){
  for(int i=0;i<average;i++){  
    Wire.beginTransmission(addr);
    Wire.write(0x3B);  
    Wire.endTransmission(false);
    Wire.requestFrom(addr,14,true);  
    acx+= (Wire.read()<<8|Wire.read())/average;
    acy+= (Wire.read()<<8|Wire.read())/average;
    acz+= (Wire.read()<<8|Wire.read())/average;
    temp+=(Wire.read()<<8|Wire.read())/average;
    gyx+= (Wire.read()<<8|Wire.read())/average;
    gyy+= (Wire.read()<<8|Wire.read())/average;
    gyz+= (Wire.read()<<8|Wire.read())/average;
  }
}

void encodeData(byte *arr, int16_t &acx, int16_t &acy, int16_t &acz, int16_t &temp, int16_t &gyx, int16_t &gyy, int16_t &gyz){
  arr[0] = char(acx>>8); 
  arr[1] = char(acx%256);
  arr[2] = char(acy>>8);
  arr[3] = char(acy%256);
  arr[4] = char(acz>>8);
  arr[5] = char(acz%256);
  arr[6] = char((acx+acy+acz)%256); //Checksum
  arr[7] = char(gyx>>8);
  arr[8] = char(gyx%256);
  arr[9] = char(gyy>>8);
  arr[10] = char(gyy%256);
  arr[11] = char(gyz>>8);
  arr[12] = char(gyz%256);
  arr[13] = char((gyx+gyy+gyz)%256); //checksum
  arr[14] = char(temp>>8);
  arr[15] = char(temp%256);
}


void checkData(String message, int AAcX, int AAcY, int AAcZ, int AGyX, int AGyY, int AGyZ){
      //decode data
      int16_t acx,acy,acz,gyx,gyy,gyz;
      byte chk0,chk1;
      parseData16(acx,(message.charAt(0)),(message.charAt(1)));
      parseData16(acy,(message.charAt(2)),(message.charAt(3)));
      parseData16(acz,(message.charAt(4)),(message.charAt(5)));
      parseData8(chk0,(message.charAt(6)));
      parseData16(gyx,(message.charAt(7)),(message.charAt(8)));
      parseData16(gyy,(message.charAt(9)),(message.charAt(10)));
      parseData16(gyz,(message.charAt(11)),(message.charAt(12)));
      parseData8(chk1,(message.charAt(13)));
      
      //Check Data
      Serial.println("Data " + String(AAcX)+" "+String(AAcY)+" "+String(AAcZ)+" "+String(AGyX)+" "+String(AGyY)+" "+String(AGyZ));
      Serial.println("CKSM " + String(byte(AAcX+AAcY+AAcZ)%256) + " " +  String(byte(AGyX+AGyY+AGyZ)%256));
      Serial.print("AAcX Variable  ");
      Serial.println(AAcX);
      Serial.print("AAcX Binary    ");
      printInt(AAcX);
      Serial.print("AAcX Stored    ");
      printByte(char(AAcX>>8));
      printByte(char(AAcX%256));
      Serial.println();
      Serial.print("Msg Recieved   ");
      printByte(char(message.charAt(0)));
      printByte(char(message.charAt(1)));
      Serial.println(message);
      Serial.println(" - " + String(message.length()));
      for(int i=0;i<8;i++){
        bitWrite(acx,8+i,bitRead((byte)(message.charAt(0)),i));
        bitWrite(acx,i,bitRead((byte)(message.charAt(1)),i));
      }
      Serial.print("acx Decoded    ");
      printInt(acx);
      Serial.print("acx String     ");
      Serial.println(acx);
      Serial.print("CK Data ");
      Serial.print(acx); Serial.print(" ");
      Serial.print(acy); Serial.print(" ");
      Serial.print(acz); Serial.print(" ");
      Serial.print(gyx); Serial.print(" ");
      Serial.print(gyy); Serial.print(" ");
      Serial.println(gyz);
      Serial.println("CK CKSM " + String(chk0) + " " +  String(chk1)+"\n");
}
