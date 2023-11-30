#include<L298N.h>

const int IN1 = 47;
const int IN2 = 49;
const int IN3 = 51;
const int ENA = 45;
const int ENB = 46;
const int IN4 = 53;

#include <QTRSensors.h>
#define num_sensor 8

int sensorValues[num_sensor]={A0,A1,A2,A3,A4,A5,A6,A7};

int speedkuleft = 170;
int speedkuright = 140;

L298N motor1 (ENA, IN1, IN2); //kiri
L298N motor2 (ENB, IN3, IN4); //kanan
int baca[num_sensor];
void setup() {
  Serial.begin(9600);
  
}

void loop() {
  bacaSensorLine();

}

void bacaSensorLine(){
  for(int i=0;i<num_sensor;i++){
    baca[i]=analogRead(sensorValues[i]);
  }
  Serial.print("Sensor : ");
  for(int i=0;i<num_sensor;i++){
    Serial.print(baca[i]);
    Serial.print('\t');
  }
  Serial.println(" ");

  if((baca[2]<=800) && (baca[3]>=900) && (baca[4]>=900) && (baca[5]<=800)){
    maju();
  }
  if((baca[2]<=800) && (baca[3]<=800) && (baca[4]>=900) && (baca[5]>=900)){
    kanan();
  }
  if((baca[2]>=900) && (baca[3]>=900) && (baca[4]<=800) && (baca[5]<=800)){
    kiri();
  }
  if((baca[2]<=800) && (baca[3]<=800) && (baca[4]<=800) && (baca[5]<=800)){
    stopp();
  }
}


void maju(){
  motor1.forward();
  motor2.forward();
  motor1.setSpeed(speedkuleft);
  motor2.setSpeed(speedkuright);
}

void mundur(){
  motor1.backward();
  motor2.backward();
  motor1.setSpeed(speedkuleft);
  motor2.setSpeed(speedkuright);
}

void kanan(){
  motor1.forward();
  motor2.backward();
  motor1.setSpeed(speedkuleft);
  motor2.setSpeed(speedkuright);
}

void kiri(){
  motor1.backward();
  motor2.forward();
  motor1.setSpeed(speedkuleft);
  motor2.setSpeed(speedkuright);
}

void stopp(){
  motor1.stop();
  motor2.stop();
  motor1.setSpeed(0);
  motor2.setSpeed(0);
}
