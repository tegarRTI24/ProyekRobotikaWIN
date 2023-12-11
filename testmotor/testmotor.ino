#include <L298N.h>

const int IN1 = 47;
const int IN2 = 49;
const int IN4 = 31;
const int ENA = 45;
const int ENB = 5;
const int IN3 = 33;

const int INA1 = 24;
const int INA2 = 26;
const int INA4 = 30;
const int ENAA = 6;
const int ENAB = 7;
const int INA3 = 28;

const int encoderLeftA = 39;  // Encoder pin A for left motor
const int encoderLeftB = 37;  // Encoder pin B for left motor
const int encoderRightA = 41; // Encoder pin A for right motor
const int encoderRightB = 44; // Encoder pin B for right motor

volatile long encoderLeftCount = 0;   // Variable to store left encoder count
volatile long encoderRightCount = 0;  // Variable to store right encoder count

float speedkuleft = 130.5;
float speedkuright = 120;

L298N motor1(ENA, IN1, IN2); // kiri
L298N motor2(ENB, IN3, IN4); // kanan

L298N motor3(ENAA, INA1, INA2); // kiri
L298N motor4(ENAB, INA3, INA4); // kanan

void setup() {
  Serial.begin(9600);

  // Set up encoder pins as input
  pinMode(encoderLeftA, INPUT);
  pinMode(encoderLeftB, INPUT);
  pinMode(encoderRightA, INPUT);
  pinMode(encoderRightB, INPUT);

  // Attach interrupts for encoder readings
  attachInterrupt(digitalPinToInterrupt(encoderLeftA), updateEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRightA), updateEncoderRight, CHANGE);
}

void loop() {
//  Serial.println(encoderRightCount);
//  delay(100);
//  // Example: Stop after a certain number of encoder counts
//  if (encoderLeftCount >= 1000) {
//    stopp();
//  }
  maju();
  majuku();
}

void updateEncoderLeft() {
  if (digitalRead(encoderLeftA) == digitalRead(encoderLeftB)) {
    encoderLeftCount++;
  } else {
    encoderLeftCount--;
  }
}

void updateEncoderRight() {
  if (digitalRead(encoderRightA) == digitalRead(encoderRightB)) {
    encoderRightCount++;
  } else {
    encoderRightCount--;
  }
}

void maju() {
  motor1.forward();
  motor2.forward();
  motor1.setSpeed(speedkuleft);
  motor2.setSpeed(speedkuright);
}

void majuku() {
  motor3.forward();
  motor4.forward();
  motor3.setSpeed(100);
  motor4.setSpeed(100);
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
