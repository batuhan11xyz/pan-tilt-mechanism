#include <Wire.h>
#include <AS5600.h>

AS5600 TiltSensor;
float tilt_angle = 0.0; // detector
const int TiltPinCW = 9;
const int TiltPinCCW = 6;
float zeropositiontilt = 0.0;

static uint32_t zeroposition2tilt = 0;
static uint32_t lastTimeTilt = 0; // as 5600

//---------------Pan

float pan_angle = 0.0;
const int PanPinCW = 11;
const int PanPinCCW = 10;
float zeropositionpan = 0.0;

static uint32_t zeroposition2pan = 0;
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

void setup() {
  Serial.begin(9600);
  Wire.begin();

  TCA9548A(2);
  PanSensor.begin(0x36);
  zeropositionpan = PanSensor.resetCumulativePosition(zeroposition2pan);

  delay(100);

  TCA9548A(7);
  TiltSensor.begin(0x36);
  zeropositiontilt = TiltSensor.resetCumulativePosition(zeroposition2tilt);

  analogWrite(PanPinCCW, LOW);
  analogWrite(PanPinCW, LOW);
  analogWrite(TiltPinCCW, LOW);
  analogWrite(TiltPinCW, LOW);
  delay (2000);

}

void loop() {

  PanAngle();
  TiltAngle();

  delay(10);
  

  digitalWrite(PanPinCW, 0);
  digitalWrite(PanPinCCW, 0);
  digitalWrite(TiltPinCW, 0);
  digitalWrite(TiltPinCCW, 0);
  delay(100);
  digitalWrite(PanPinCCW, 0);
  digitalWrite(PanPinCW, 0);
  digitalWrite(TiltPinCCW, 0);
  digitalWrite(TiltPinCW, 0);
  delay(100);

  Serial.println(pan_angle, 4);
  Serial.print("\t");
  Serial.print("\t");
  Serial.println(tilt_angle, 4);
  Serial.println("-----------------------------------------------------------");

  delay(100);


}
