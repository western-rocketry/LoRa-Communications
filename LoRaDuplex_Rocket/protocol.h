#include <Arduino.h>
#include <Adafruit_GPS.h>

typedef union{
  float num;
  uint8_t bytes[4];
} floatunion_t;

void parseData8(byte &val, byte b1);
void parseData16(int16_t &val, byte b1, byte b2);
void parseData32(int32_t &val, byte b1, byte b2, byte b3, byte b4);
void printInt(int b);
void printByte(byte b);
void addDataGyro(byte addr, int16_t &acx, int16_t &acy, int16_t &acz, int16_t &temp, int16_t &gyx, int16_t &gyy, int16_t &gyz);
void addDataGyro(byte addr, int16_t &acx, int16_t &acy, int16_t &acz, int16_t &temp, int16_t &gyx, int16_t &gyy, int16_t &gyz, uint8_t average);
void encodeData(byte *arr, int16_t acx, int16_t acy, int16_t acz, int16_t temp, int16_t gyx, int16_t gyy, int16_t gyz, byte gpsfix, byte gpsnum, byte gpsfail, floatunion_t gpsalt, floatunion_t gpslat, floatunion_t gpslong, floatunion_t gpsspeed, floatunion_t gpsangle, floatunion_t vdop, floatunion_t hdop, floatunion_t pdop);
void checkData(String message, int AAcX, int AAcY, int AAcZ, int AGyX, int AGyY, int AGyZ);
