#include<Servo.h>

int trigPin = A0;
int echoPin = A1;
Servo myservo1,myservo2,myservo3,myservo4;
int statusmoved = 0;
long durasi;
int jarak,jarak_max=20;

void setup() {
  Serial.begin(9600);
  pinMode(echoPin,INPUT);
  pinMode(trigPin,OUTPUT);
  myservo1.attach(9);
  myservo2.attach(10);
  myservo3.attach(11);
  myservo4.attach(12);
  myservo1.write(53);
  myservo2.write(115);
  myservo3.write(60);
  myservo4.write(60);
}

void loop() {
  digitalWrite(trigPin,LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);
  durasi = pulseIn(echoPin,HIGH);
  jarak = durasi * 0.034/2;
  Serial.print("Jarak : ");
  Serial.print(jarak);
  Serial.println("     cm");
  delay(10);   
  if(jarak <= 5){
    s1jalan();
    delay(10);  
  }
}

void s1jalan(){
  for(float i=53;i<=158;i+=1.2){
    myservo1.write(i);
    delay(10);
  }
  for(float i=115;i<=155;i+=1.0){
    myservo2.write(i);
    delay(15);
  }
  for(float i=60;i<=100;i+=1.0){
    myservo3.write(i);
    delay(15);
  }
  for(float i=60;i<=95;i+=1.0){
    myservo4.write(i);
    delay(15);
  }
  for(float i=95;i>=60;i-=1.0){
    myservo4.write(i);
    delay(15);
  }
  for(float i=100;i>=60;i-=1.0){
    myservo3.write(i);
    delay(15);
  }
  for(float i=155;i>=115;i-=1.0){
    myservo2.write(i);
    delay(15);
  }
  for(float i=158;i>=53;i-=1.2){
    myservo1.write(i);
    delay(10);
  }
  statusmoved++;
  Serial.println(statusmoved);
  if(statusmoved>=4){
    myservo1.write(53);
    delay(15);
    myservo2.write(115);
    delay(100);
    myservo3.write(60);
    delay(100);
    myservo1.write(60);
    delay(100);
    exit(0);
  }
}

//s1:53-159
//s2:115-155
//s3:60-90
//s4:60-95
