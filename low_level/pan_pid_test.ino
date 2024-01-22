#include <Wire.h>
#include <AS5600.h>

//---------------Pan
float Pan_angle = 0;
float pan_angle = 0;
static uint32_t lastTimePan = 0;

const int PanPinCCW = 10;
const int PanPinCW = 11;

float controlSignalPan = 0.0;

float targetAnglePan = 0.0;
float errorPan = 0.0;
float previousErrorPan = 0.0;

int PWMValuePan = 0;

float currentTimePan = 0.0;
const float PanKp = 25.0;
const float PanKd = 0.035;

float zeroposition = 0.0;

static uint32_t zeroposition2 = 0;

float previousTimePan = 0;

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
    pan_angle = pan_angle * 360 / 4096;
    lastTimePan = micros();
  }
}

void drivePanMotor()
{

  PWMValuePan = (int)fabs(controlSignalPan);
  if (PWMValuePan >= 255)
    PWMValuePan = 255;
  else if (PWMValuePan <= 10)
    PWMValuePan = 0;
  else
    PWMValuePan = PWMValuePan;

  if (controlSignalPan <= 0)
  {
    analogWrite(PanPinCCW, PWMValuePan);
  }
  else if (controlSignalPan >= 0)
  {
    analogWrite(PanPinCW, PWMValuePan);
  }

  else if (controlSignalPan == 0)
  {
    digitalWrite(PanPinCCW, LOW);
    digitalWrite(PanPinCW, LOW);
  }
}

void calculatePIDforPan ()
{
  //if(micros() - lastTimePanPID == 50000)

  currentTimePan = micros();
  auto deltaTimePan = (currentTimePan - previousTimePan) / 1e6;
  previousTimePan = currentTimePan;

  errorPan = targetAnglePan - pan_angle;

  auto edotPan = (errorPan - previousErrorPan) / deltaTimePan;

  controlSignalPan = (PanKp * errorPan) + (PanKd * edotPan) ;

  previousErrorPan = errorPan;


}
void setup() {
  Serial.begin(9600);
  Wire.begin();

  TCA9548A(2);
  PanSensor.begin(0x36);
  analogWrite(PanPinCCW, LOW);
  analogWrite(PanPinCW, LOW);
  zeroposition = PanSensor.resetCumulativePosition(zeroposition2);

  // put your setup code here, to run once:
  delay (2000);

}

void loop() {
  PanAngle();
  calculatePIDforPan ();
  drivePanMotor();
  


}
