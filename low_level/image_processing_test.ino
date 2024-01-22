

String data1 ;
float intval = 0.0;
String data2 ;
float intval2 = 0.0 ;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  data1 = Serial.readStringUntil('\n');
  intval = data1.toFloat();
  data2 = Serial.readStringUntil('\n');
  intval2 = data2.toFloat();

  Serial.println(intval2);
  Serial.print("/n");
  Serial.println(intval);
  Serial.print("/n");

}
