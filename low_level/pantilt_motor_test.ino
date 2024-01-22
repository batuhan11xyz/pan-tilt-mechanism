
const int TiltPinCW = 9;
const int TiltPinCCW = 6;

const int PanPinCCW = 10;
const int PanPinCW = 11;

void setup() {
  Serial.begin(9600);

  analogWrite(PanPinCCW, LOW);
  analogWrite(PanPinCW, LOW);
  analogWrite(TiltPinCCW, LOW);
  analogWrite(TiltPinCW, LOW);

  // put your setup code here, to run once:
  delay (2000);
}

void loop() {

  analogWrite(PanPinCW, LOW);
  analogWrite(PanPinCCW, LOW);
  analogWrite(TiltPinCW,LOW);
  analogWrite(TiltPinCCW, LOW);
  delay(1000);
  analogWrite(PanPinCCW, LOW);
  analogWrite(PanPinCW, LOW);
  analogWrite(TiltPinCCW, LOW);
  analogWrite(TiltPinCW, LOW);
  delay(2000);

}
