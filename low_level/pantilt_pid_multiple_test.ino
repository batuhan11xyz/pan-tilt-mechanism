#include <Wire.h>
#include <AS5600.h>
String veri_str1 ;
float intval = 0.0;
String veri_str2 ;
float intval2 = 0.0 ;
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
static uint32_t previousMillis1 = 0;
static uint32_t previousMillis2 = 0;
static uint32_t previousMillis3 = 0;
static uint32_t previousMillis4 = 0;
static uint32_t previousMillisdata = 0;


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

  controlSignalPan = (PanKp * errorPan) + (PanKd * edotPan)  ;

  previousErrorPan = errorPan;

}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  TCA9548A(2);
  PanSensor.begin(0x36);
  zeroposition2pan = PanSensor.resetCumulativePosition(zeroposition2pan);
  delay(100);

  TCA9548A(7);
  TiltSensor.begin(0x36);
  zeroposition2tilt = TiltSensor.resetCumulativePosition(zeroposition2tilt);

  analogWrite(PanPinCCW, LOW);
  analogWrite(PanPinCW, LOW);
  analogWrite(TiltPinCCW, LOW);
  analogWrite(TiltPinCW, LOW);
  intval = 0.0;
  intval2 = 0.0;
  veri_str1 = "";
  veri_str2 = "";

  delay (2000);

}

void loop() {
    if (Serial.available())
    {
      if (millis() - previousMillisdata >= 1) {
        Serial.println("Alınan veriler: ");
        veri_str1 = Serial.readStringUntil('\n');
        veri_str1.trim();
        intval = veri_str1.toFloat();
        //Serial.print(intval);
        Serial.print(previousErrorPan);
        veri_str2 = Serial.readStringUntil('\n');
        veri_str2.trim();
        intval2 = veri_str2.toFloat();
        Serial.print(" |||| ");
        //Serial.println(intval2);
        Serial.print(previousErrorTilt);
        PanAngle();
        TiltAngle();
        previousMillisdata = millis();

      }
      if (millis() - previousMillis >= 10) {
        if (intval > 0.0 and intval2 < 0.0) { // reg 1 motora göre
          if (millis() - previousMillis1 >= 1) {
            targetAnglePan = intval;
            targetAngleTilt = -intval2;
//            if (abs(errorPan) <= 1.0 and abs(errorTilt) <= 1.0) {
//              continue;
//            } else {
//              targetAnglePan = intval;
//              targetAngleTilt = -intval2;
//            }
            previousMillis1 = millis();
          }
          //      Serial.println("------------------0");
        } else if (intval < 0.0 and intval2 < 0.0) { // reg 2 motora göre
          if (millis() - previousMillis2 >= 1) {
            targetAnglePan = intval;
            targetAngleTilt = -intval2;

            previousMillis2 = millis();
          }
          //      Serial.println("------------------1");
        } else if (intval < 0.0 and intval2 > 0.0) { // reg 3
          if (millis() - previousMillis3 >= 1) {
            targetAnglePan = intval;
            targetAngleTilt = -intval2;
            previousMillis3 = millis();
          }
          //      Serial.println("------------------2");
        } else if (intval > 0.0 and intval2 > 0.0) { // reg 4
          if (millis() - previousMillis4 >= 1) {
            targetAnglePan = intval;
            targetAngleTilt = -intval2;
            
            previousMillis4 = millis();
          }
          //      Serial.println("------------------3");
        }
        previousMillis = millis();

      }
      if (millis() - lastTimeTiltPID >= 1)
      {
        calculatePIDforTilt ();
        driveTiltMotor();
        lastTimeTiltPID = millis();

      }
      calculatePIDforPan ();
      drivePanMotor();
    }
}


//reg 1 pan + tilt +
//reg 2 pan - tilt -
//reg 3 pan - tilt +
//reg 4 pan + tilt +
