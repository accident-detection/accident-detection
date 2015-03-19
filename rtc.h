#ifndef RTC
#define RTC
#include "Arduino.h"
#include "Wire.h"
#define DS3231_I2C_ADDRESS 0x68

byte decToBcd(byte val);
byte bcdToDec(byte val);
void setTime(byte second, byte minute, byte hour,
              byte dayOfWeek, byte dayOfMonth, byte month,
              byte year);
String getTime();

#endif
