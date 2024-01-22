#include <Wire.h> 
#include <AS5600.h>


float pan_angle =0;
float tilt_angle =0;
const int TiltPinCW = 6;
static uint32_t lastTimePan = 0;
static uint32_t lastTimeTilt = 0;

AS5600 TiltSensor; 
AS5600 PanSensor; 
void TCA9548A(uint8_t bus)
{
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}

void PanAngle()
{
  if(micros() - lastTimePan >=50000)
  {
  TCA9548A(2);
  pan_angle = PanSensor.getCumulativePosition(); //receiving the previous position from the sensor
  pan_angle = pan_angle* 360/4096;
  lastTimePan = micros();
  }
 
}
void TiltAngle()
{
  if(micros() - lastTimeTilt >=50000)
  {
  TCA9548A(7);
  tilt_angle = TiltSensor.getCumulativePosition(); //receiving the previous position from the sensor 
  tilt_angle = tilt_angle * 360/4096;
  lastTimeTilt = micros();
  }
 
  
}
void setup() 
{
 Serial.begin(9600);
 Wire.begin();

TCA9548A(2);
PanSensor.begin(0x36); 

TCA9548A(7);
TiltSensor.begin(0x36);

}

void loop() 
{  
  PanAngle();
  TiltAngle();
 
 Serial.println(tilt_angle, 8);
 Serial.print("\t");
 Serial.print(micros() - lastTimeTilt);
 Serial.print("\t");
 Serial.print("\t");
 
 
}
