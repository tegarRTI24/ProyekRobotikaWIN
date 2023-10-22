#include <Servo.h>
int pot = A0;
Servo myservo;

void setup() {
  Serial.begin(9600);
  pinMode(pot,INPUT);
  myservo.attach(9);
}

void loop() {
  int r=analogRead(pot);
  int valu=map(r,0,1023,0,180);
  myservo.write(valu);
  delay(15);
  Serial.println(valu);
}
