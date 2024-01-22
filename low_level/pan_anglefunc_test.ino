
#include <Wire.h>
#include <AS5600.h>


float pan_angle = 0.0;
const int PanPinCW = 11;
const int PanPinCCW = 10;
float zeroposition = 0.0;

static uint32_t zeroposition2 = 0;
static uint32_t lastTimePan = 0;

AS5600 PanSensor;

void TCA9548A(uint8_t bus)
{
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}

void PanAngle()
{
  if (micros() - lastTimePan >= 50000)
  {
    TCA9548A(2);
    pan_angle = PanSensor.getCumulativePosition(); //receiving the previous position from the sensor
    pan_angle = pan_angle * 360.0 / 4096.0;
    lastTimePan = micros();
  }
}

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  TCA9548A(2);
  PanSensor.begin(0x36);
  analogWrite(PanPinCCW, LOW);
  analogWrite(PanPinCW, LOW);
  zeroposition = PanSensor.resetCumulativePosition(zeroposition2);
  delay(2000);
}

void loop()
{
  PanAngle();
 
  Serial.println(pan_angle, 8);
  Serial.print("\t");
  delay(10);

  analogWrite(PanPinCW, LOW);
  analogWrite(PanPinCCW, LOW);
  delay(1000);
  analogWrite(PanPinCCW, LOW);
  analogWrite(PanPinCW, LOW);
  delay(5000);


}
