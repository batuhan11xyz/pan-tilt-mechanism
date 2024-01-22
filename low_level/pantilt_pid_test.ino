#include <Wire.h>
#include <AS5600.h>

AS5600 TiltSensor;
float tilt_angle = 0.0; // detector
const int TiltPinCW = 9;
const int TiltPinCCW = 6;
float zeropositiontilt = 0.0;

static uint32_t zeroposition2tilt = 0;
static uint32_t lastTimeTilt = 0; // as 5600

float targetAngleTilt = 0.0;
static uint32_t lastTimeTiltPID = 0;

float controlSignalTilt = 0;
float errorIntegralTilt;
float errorTilt = 0;

float currentTimeTilt;
const float TiltKp = 50.0;
const float TiltKd = 0.045;

int PWMValueTilt = 0;

float previousTimeTilt = 0;
float previousErrorTilt = 0;

//---------------Pan

float pan_angle = 0.0;
const int PanPinCW = 11;
const int PanPinCCW = 10;
float zeropositionpan = 0.0;

static uint32_t zeroposition2pan = 0;
static uint32_t lastTimePan = 0;

float controlSignalPan = 0;

float targetAnglePan = 0.0;
float errorPan = 0;

int PWMValuePan = 0;

float currentTimePan = 0;
const float PanKp = 50.0;
const float PanKd = 0.045;


float previousTimePan = 0;
float previousErrorPan = 0;


static uint32_t previousMillis = 0;

boolean condition1 = 0;
boolean condition2 = 0;

int counter = 0;
AS5600 PanSensor;

void TCA9548A(uint8_t bus)
{
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}

void PanAngle()
{
 
  
    TCA9548A(2);
    pan_angle = PanSensor.getCumulativePosition(); //receiving the previous position from the sensor
    pan_angle = pan_angle * 360.0 / 4096.0;
    
}


void TiltAngle()
{
  
    TCA9548A(7);
    tilt_angle = TiltSensor.getCumulativePosition(); //receiving the previous position from the sensor
    tilt_angle = tilt_angle * 360.0 / 4096.0;
    
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
  //if(micros() - lastTimePanPID == )

  currentTimeTilt = micros();
  auto deltaTimeTilt = (currentTimeTilt - previousTimeTilt) / 1e6;
  previousTimeTilt = currentTimeTilt;

  errorTilt = targetAngleTilt - tilt_angle;

  auto edotTilt = (errorTilt - previousErrorTilt) / deltaTimeTilt;

  controlSignalTilt = (TiltKp * errorTilt) + (TiltKd * edotTilt) ;

  previousErrorTilt = errorTilt;

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
  //if(micros() - lastTimePanPID == 10000)

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
  zeropositionpan = PanSensor.resetCumulativePosition(zeroposition2pan);

  delay(100);

  TCA9548A(7);
  TiltSensor.begin(0x36);
  zeropositiontilt = TiltSensor.resetCumulativePosition(zeroposition2tilt);

  analogWrite(PanPinCCW, LOW);
  analogWrite(PanPinCW, LOW);
  analogWrite(TiltPinCCW, LOW);
  analogWrite(TiltPinCW, LOW);
  PanAngle();
  TiltAngle();
  delay (2000);

}

void loop() {
  
  PanAngle();
  TiltAngle();

  // put your main code here, to run repeatedly:

  calculatePIDforTilt ();
  driveTiltMotor();

  calculatePIDforPan ();
  drivePanMotor();

  if (millis() - previousMillis >= 1000) {
    previousMillis = millis();

    if (condition1 == 0) {
      condition1 = 1;
      targetAnglePan =  10;
      targetAngleTilt = 10;
      Serial.println(tilt_angle);
      Serial.println(pan_angle);
      Serial.println("------------------1");//reg 1
    } else {
      condition1 = 0;
      targetAngleTilt = 5;
      targetAnglePan = 5;
      Serial.println(tilt_angle);
      Serial.println(pan_angle);
      Serial.println("------------------2"); //reg 3

    }
  }
}
