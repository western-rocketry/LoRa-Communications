#include <Arduino.h>

void parseData8(byte &val, byte b1);
void parseData16(int16_t &val, byte b1, byte b2);
void parseData32(int32_t &val, byte b1, byte b2, byte b3, byte b4);
void printInt(int b);
void printByte(byte b);
void checkData(String message, int AAcX, int AAcY, int AAcZ, int AGyX, int AGyY, int AGyZ);