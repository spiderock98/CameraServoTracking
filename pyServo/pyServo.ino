#include <Servo.h>

Servo myservo;
int pos = 0;
String strPos = "0";

void setup() {
  // put your setup code here, to run once:
  myservo.attach(9);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  while(!Serial);
  if (Serial.available()){
    strPos = Serial.readStringUntil('\r');
    pos = strPos.toInt();
    //pos = map(pos, 30, 270, 0, 180);
    myservo.write(pos);
    delay(500);
  }
}
