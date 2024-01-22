#include <Wire.h>
#include <AS5600.h>


const int TiltPinCW = 9;
const int TiltPinCCW = 6;
float tilt_angle = 0;
static uint32_t lastTimeTilt = 0;


void setup()
{
  Serial.begin(9600);
  Wire.begin();
  analogWrite(TiltPinCCW, LOW);
  analogWrite(TiltPinCW, LOW);
  delay(2000);

}

void loop()
{

  analogWrite(TiltPinCW, 0);
  analogWrite(TiltPinCCW, LOW);
  delay(500);
  analogWrite(TiltPinCCW, 0);
  analogWrite(TiltPinCW, LOW);
  delay(500);


}
