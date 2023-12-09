#include <L298N.h>

const int IN1 = 47;
const int IN2 = 49;
const int IN3 = 51;
const int ENA = 45;
const int ENB = 46;
const int IN4 = 53;

float speedkuleft = 120;
float speedkuright = 120;

L298N motor1(ENA, IN1, IN2); // kiri
L298N motor2(ENB, IN3, IN4); // kanan

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN,OUTPUT);
}

void loop() {
  if(Serial.available()>0){
    char value = Serial.read();

    if(value == 'F'){
      maju();
      Serial.print("Maju");
    }if (value == 'B'){
      mundur();
      Serial.print("Mundur");
    }if (value == 'L'){
      kiri();
      Serial.print("Kiri");
    }if (value == 'R'){
      kanan();
      Serial.print("kanan");
    }if (value == 'S'){
      stopp();
      Serial.print("stop");
    }
  }

}


void maju(){
  motor1.forward();
  motor2.forward();
  motor1.setSpeed(speedkuleft);
  motor2.setSpeed(speedkuright);
}

void mundur() {
  motor1.backward();
  motor2.backward();
  motor1.setSpeed(speedkuleft);
  motor2.setSpeed(speedkuright);
}

void kanan() {
  motor1.forward();
  motor2.backward();
  motor1.setSpeed(speedkuleft);
  motor2.setSpeed(speedkuright);
}

void kiri() {
  motor1.backward();
  motor2.forward();
  motor1.setSpeed(speedkuleft);
  motor2.setSpeed(speedkuright);
}

void stopp() {
  motor1.stop();
  motor2.stop();
  motor1.setSpeed(0);
  motor2.setSpeed(0);
}
