#include "decode.h"
#include <Arduino.h>

void parseData8(byte &val, byte b1){
  for(int i=0;i<8;i++){
    bitWrite(val,i,bitRead(byte(b1),i));
  }
}

void parseData16(int16_t &val, byte b1, byte b2){
  for(int i=0;i<8;i++){
    bitWrite(val,i+8,bitRead(b1,i));
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

uint32_t parseTime(byte b1, byte b2, byte b3, byte b4){
  uint32_t val;
  for(int i=0;i<8;i++){
    bitWrite(val,i,bitRead(byte(b1),i));
    bitWrite(val,i+8,bitRead(byte(b2),i));
    bitWrite(val,i+16,bitRead(byte(b3),i));
    bitWrite(val,i+24,bitRead(byte(b4),i));
  }
  return(val);
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

void checkData(String message, int AAcX, int AAcY, int AAcZ, int AGyX, int AGyY, int AGyZ){
      //decode data
      int16_t acx,acy,acz,gyx,gyy,gyz;
      byte chk0,chk1;
      parseData16(gyx,(message.charAt(0)),(message.charAt(1)));
      parseData16(gyy,(message.charAt(2)),(message.charAt(3)));
      parseData16(gyz,(message.charAt(4)),(message.charAt(5)));
      parseData8(chk0,(message.charAt(6)));
      parseData16(acx,(message.charAt(7)),(message.charAt(8)));
      parseData16(acy,(message.charAt(9)),(message.charAt(10)));
      parseData16(acz,(message.charAt(11)),(message.charAt(12)));
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
      Serial.print("acx Recieved   ");
      printByte(char(message.charAt(0)));
      printByte(char(message.charAt(1)));
      Serial.println();
      for(int i=0;i<8;i++){
        bitWrite(acx,8+i,bitRead((byte)(message.charAt(0)),i));
        bitWrite(acx,i,bitRead((byte)(message.charAt(1)),i));
      }
      Serial.print("acx Decoded    ");
      printInt(acx);
      Serial.print("acx String     ");
      Serial.println(acx);
      Serial.print("CK Data ");
      Serial.print(gyx); Serial.print(" ");
      Serial.print(gyy); Serial.print(" ");
      Serial.print(gyz); Serial.print(" ");
      Serial.print(acx); Serial.print(" ");
      Serial.print(acy); Serial.print(" ");
      Serial.println(acz);
      Serial.println("CK CKSM " + String(chk0) + " " +  String(chk1)+"\n");
}
