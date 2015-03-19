#include "rtc.h"

byte decToBcd(byte val)
{
  return( (val/10*16) + (val%10) );
}

byte bcdToDec(byte val)
{
  return( (val/16*10) + (val%16) );
}

void setTime(byte second, byte minute, byte hour,
              byte dayOfWeek, byte dayOfMonth, byte month,
              byte year)
{
  Wire.beginTransmission(DS3231_I2C_ADDRESS);

  Wire.write(0);
  Wire.write(decToBcd(second));
  Wire.write(decToBcd(minute));
  Wire.write(decToBcd(hour));
  Wire.write(decToBcd(dayOfWeek));
  Wire.write(decToBcd(dayOfMonth));
  Wire.write(decToBcd(month));
  Wire.write(decToBcd(year));

  Wire.endTransmission();
}

String getTime()
{
  String time = "";
  byte second, minute, hour, dayOfWeek,
        dayOfMonth, month, year;
  // Getting time from the DS3231

  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);

  // request seven bytes of data starting from register 00h
  second = bcdToDec(Wire.read() & 0x7f);
  minute = bcdToDec(Wire.read());
  hour = bcdToDec(Wire.read() & 0x3f);
  dayOfWeek = bcdToDec(Wire.read());
  dayOfMonth = bcdToDec(Wire.read());
  month = bcdToDec(Wire.read());
  year = bcdToDec(Wire.read());

  // Time
  time.concat(hour);
  time.concat(":");
  if (minute<10)
  {
    time.concat("0");
  }
  time.concat(minute);
  time.concat(":");
  if (second<10)
  {
    time.concat("0");
  }

  // Date
  time.concat(second);
  time.concat(" ");
  time.concat(dayOfMonth);
  time.concat("/");
  time.concat(month);
  time.concat("/");
  time.concat(year);

  return time;
}
