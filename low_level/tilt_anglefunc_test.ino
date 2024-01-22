#include <Wire.h>
#include <AS5600.h>


float tilt_angle = 0.0;
const int TiltPinCW = 9;
const int TiltPinCCW = 6;
float zeroposition = 0.0;

static uint32_t zeroposition2 = 0;
static uint32_t lastTimeTilt = 0;

AS5600 TiltSensor;

void TCA9548A(uint8_t bus)
{
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}


void TiltAngle()
{
  if (micros() - lastTimeTilt >= 50000)
  {
    TCA9548A(7);
    tilt_angle = TiltSensor.getCumulativePosition(); //receiving the previous position from the sensor
    tilt_angle = tilt_angle * 360.0 / 4096.0;
    lastTimeTilt = micros();
  }

}


void setup()
{
  Serial.begin(9600);
  Wire.begin();
  TCA9548A(7);
  TiltSensor.begin(0x36);
  analogWrite(TiltPinCCW, LOW);
  analogWrite(TiltPinCW, LOW);
  zeroposition = TiltSensor.resetCumulativePosition(zeroposition2);
  delay(2000);

}

void loop()
{
  TiltAngle();
  
  Serial.println(tilt_angle, 8);
  Serial.print("\t");
  delay(10);


  analogWrite(TiltPinCW, LOW);
  analogWrite(TiltPinCCW, LOW);
  delay(1000);
  analogWrite(TiltPinCCW, LOW);
  analogWrite(TiltPinCW, LOW);
  delay(5000);



}
