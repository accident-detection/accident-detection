#include "rtc/rtc.h"
#include "Wire.h"

void setup()
{
  Serial.begin(9600);
  Wire.begin();
}

void loop()
{
  Serial.println(getTime());
  delay(1000);
}
