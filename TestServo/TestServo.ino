#include <Servo.h>
//int pot = A0;
Servo myservo;

void setup() {
  Serial.begin(9600);
  //pinMode(pot,INPUT);
  myservo.attach(9);
}

void loop() {
  //int r=analogRead(pot);
  //int valu=map(r,0,1023,0,180);
  //delay(1000)
  for(float i=0;i<=90;i+=1){
    myservo.write(i);
    delay(15);
  }
  for(float i=90;i>=0;i-=1){
    myservo.write(i);
    delay(15);
  }
//  myservo.write(90);
//  delay(15);
  //Serial.println(valu);
}
