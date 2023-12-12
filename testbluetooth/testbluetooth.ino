#include <L298N.h>

const int IN4 = 47;
const int IN3 = 49;
const int IN1 = 31;
const int ENA = 4;
const int ENB = 5;
const int IN2 = 33;
#define ENCRA 41
#define ENCRB 2
#define ENCLA 39
#define ENCLB 3

const int INA1 = 26;
const int INA2 = 28; //kanan
const int INA4 = 32;
const int ENAA = 6;
const int ENAB = 7;
const int INA3 = 30; //kiri

float speedkuleft = 150;
float speedkuright = 150;
int statusk,statusku;
Servo s1,s2,s3;

L298N motorkiri (ENB,IN3,IN4); //kiri
L298N motorkanan (ENA,IN1,IN2); //kanan

L298N motorkananku(ENAA, INA1, INA2); // kanan
L298N motorkiriku(ENAB, INA3, INA4); // kiri

void setup() {
  Serial.begin(9600);
  s1.attach(8);
  s2.attach(9);
  s3.attach(10);
  s1.write(13);
  delay(15);
  s2.write(90);
  delay(15);
  s3.write(153);
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
    if (value == 'W'){
      servo();
      Serial.print("servo nyala");
    }
    if (value == 'w'){
      servo2();
      Serial.print("servo nyala");
    }
    if (value == 'U'){
      motor();
      Serial.print("servo nyala");
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

void servo(){
  for(int i=90;i<135;i+=1){
     s2.write(i);
     delay(15);
  }
  //servo 1 ambil bola
  for(int i=13;i<88;i+=1){
     s1.write(i);
     delay(15);
  }
  //servo 1 angkat bola
  for(int i=88;i>13;i-=1){
     s1.write(i);
     delay(15);
  }
  statusk++;
  if (statusk == 1){
    exit(0);
  }
}

void servo2(){
  //servo 2 bukak bola
  for (int i  = 135; i >= 90; i -= 1) {
    s2.write(i);
    delay(15);
  }
  delay(2000);
  for (int i  = 90; i <= 135; i += 1) {
    s2.write(i);
    delay(15);
  }
  statusku++;
  if (statusku == 1){
    exit(0);
  }
}

void motor(){
  motorkiriku.forward();
  motorkiriku.setSpeed(40);
  motorkananku.backward();
  motorkananku.setSpeed(40);
}
