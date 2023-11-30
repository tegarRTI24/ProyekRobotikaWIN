#include <ESP32Servo.h>
Servo myservo;
void setup() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);    // standard 50 hz servo
  myservo.attach(18, 500, 2400);
}

void loop() {
  for(float i=0;i<=90;i+=0.5){
    myservo.write(i);
    delay(15);
  }
  for(float i=90;i>=0;i-=0.5){
    myservo.write(i);
    delay(15);
  }

}
