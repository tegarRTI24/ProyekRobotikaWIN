#include<L298N.h>

const int IN1 = 47;
const int IN2 = 49;
const int IN3 = 51;
const int ENA = 45;
const int ENB = 46;
const int IN4 = 53;

int leftSpeedVal = 100;
float rightSpeedVal = 100;

L298N motor1 (ENA,IN1,IN2);
L298N motor2 (ENB,IN3,IN4);

#include <Wire.h>

const int MPU = 0x68;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;

int equilibriumSpeed = 248;

int c =0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  calculateError();
  delay(20);
  currentTime = micros();
}

void loop() {
  readAcceleration();
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX;
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY;

  previousTime = currentTime;
  currentTime = micros();
  elapsedTime = (currentTime - previousTime) / 1000000;
  readGyro();
  GyroX -= GyroErrorX;
  GyroY -= GyroErrorY;
  GyroZ -= GyroErrorZ;
  gyroAngleX += GyroX * elapsedTime;
  gyroAngleY += GyroY * elapsedTime;
  yaw += GyroZ * elapsedTime;
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;

  driveForward();
}

void driveForward() {
  maju();
  leftSpeedVal = equilibriumSpeed;
  rightSpeedVal = equilibriumSpeed;
  analogWrite(ENA, leftSpeedVal);
  analogWrite(ENB, rightSpeedVal);
}

void readAcceleration() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
}

void readGyro() {
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
}

void calculateError() {
  c = 0;
  while (c < 200) {
    readAcceleration();
    AccErrorX += (atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI);
    AccErrorY += (atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI);
    c++;
  }
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;

  while (c < 200) {
    readGyro();
    GyroErrorX += GyroX;
    GyroErrorY += GyroY;
    GyroErrorZ += GyroZ;
    c++;
  }
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
}

void maju(){
  motor1.forward();
  motor2.forward();
  motor1.setSpeed(leftSpeedVal);
  motor2.setSpeed(rightSpeedVal);
}

void mundur(){
  motor1.backward();
  motor2.backward();
  motor1.setSpeed(leftSpeedVal);
  motor2.setSpeed(rightSpeedVal);
}

void kanan(){
  motor1.forward();
  motor2.backward();
  motor1.setSpeed(leftSpeedVal);
  motor2.setSpeed(rightSpeedVal);
}

void kiri(){
  motor1.backward();
  motor2.forward();
  motor1.setSpeed(leftSpeedVal);
  motor2.setSpeed(rightSpeedVal);
}

void stopp(){
  motor1.stop();
  motor2.stop();
  motor1.setSpeed(0);
  motor2.setSpeed(0);
}
