#include <Wire.h>
#include <AS5600.h>

//-----------Tilt

float Tilt_angle = 0.0; // encoder
float tilt_angle = 0.0; // detector


float targetAngleTilt = 62.0;
static uint32_t lastTimeTilt = 0; // as 5600
static uint32_t lastTimeTiltPID = 0;


const int TiltPinCW = 9;
const int TiltPinCCW = 6;

float controlSignalTilt = 0.0;
float errorIntegralTilt;
float errorTilt = 0.0;

float currentTimeTilt;
const float TiltKp = 26.0;
const float TiltKd = 0.034;

int PWMValueTilt = 0;

float previousTimeTilt = 0.0;
float previousErrorTilt = 0.0;

float zeroposition = 0.0;

static uint32_t zeroposition2 = 0;

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
    tilt_angle = tilt_angle * 360 / 4096;
    lastTimeTilt = micros();
  }
}

void driveTiltMotor()
{

  PWMValueTilt = (int)fabs(controlSignalTilt);
  if (PWMValueTilt >= 255)
    PWMValueTilt = 255;
  else if (PWMValueTilt <= 10)
    PWMValueTilt = 0;
  else
    PWMValueTilt = PWMValueTilt;

  if (controlSignalTilt <= 0)
  {
    analogWrite(TiltPinCCW, PWMValueTilt);
  }
  else if (controlSignalTilt >= 0)
  {
    analogWrite(TiltPinCW, PWMValueTilt);
  }

  else if (controlSignalTilt == 0)
  {
    digitalWrite(TiltPinCCW, LOW);
    digitalWrite(TiltPinCW, LOW);
  }
}

void calculatePIDforTilt ()
{

  currentTimeTilt = micros();
  auto deltaTimeTilt = (currentTimeTilt - previousTimeTilt) / 1e6;
  previousTimeTilt = currentTimeTilt;

  errorTilt = targetAngleTilt - tilt_angle;

  auto edotTilt = (errorTilt - previousErrorTilt) / deltaTimeTilt;

  controlSignalTilt = (TiltKp * errorTilt) + (TiltKd * edotTilt) ;

  previousErrorTilt = errorTilt;

}


void setup() {
  Serial.begin(9600);
  Wire.begin();


  TCA9548A(7);
  TiltSensor.begin(0x36);
  analogWrite(TiltPinCCW, LOW);
  analogWrite(TiltPinCW, LOW);
  zeroposition = TiltSensor.resetCumulativePosition(zeroposition2);
  delay (2000);

}

void loop() {

  if (millis() - lastTimeTiltPID >= 1)
  {
    calculatePIDforTilt ();
    lastTimeTiltPID = millis();
    driveTiltMotor();
  }
  TiltAngle();
  calculatePIDforTilt ();
  driveTiltMotor();



}
