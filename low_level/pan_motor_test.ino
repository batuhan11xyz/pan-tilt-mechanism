#include <Wire.h>
#include <AS5600.h>

const int PanPinCW = 11;
const int PanPinCCW = 10;


void setup()
{
  Serial.begin(9600);


}

void loop()
{


  analogWrite(PanPinCW, 0);
  analogWrite(PanPinCCW, LOW);
  delay(1000);
  analogWrite(PanPinCCW, 0);
  analogWrite(PanPinCW, LOW);
  delay(1000);


}
