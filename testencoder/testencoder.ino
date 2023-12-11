#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;
#include <L298N.h>
#include <Servo.h>
volatile long int encoder_posleft = 0;
volatile long int encoder_posright = 0;
volatile unsigned long lastDebounceTime1 = 0;
volatile unsigned long lastDebounceTime2 = 0;
unsigned long debounceDelay1 = 50;
unsigned long debounceDelay2 = 50;
unsigned long timeku=0;

#define IN1 47
#define IN2 49
#define ENA 45 //kanan
#define IN3 37 
#define IN4 35
#define ENB 5 //kiri
#define ENCRA 41
#define ENCRB 2
#define ENCLA 39
#define ENCLB 3

const int INA1 = 24;
const int INA2 = 26; //kanan
const int INA4 = 30;
const int ENAA = 6;
const int ENAB = 7;
const int INA3 = 28; //kiri

Servo sr1,sr2,sr3;
int sr1 = 8;
int sr2 = 9;
int sr3 = 10;

L298N motorkiri (ENB,IN3,IN4); //kiri
L298N motorkanan (ENA,IN1,IN2); //kanan

L298N motorkananku(ENAA, INA1, INA2); // kanan
L298N motorkiriku(ENAB, INA3, INA4); // kiri

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 20
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;  
uint16_t packetSize;
uint16_t fifoCount;  
uint8_t fifoBuffer[64];  

Quaternion q;

VectorFloat gravity; 
float euler[3];      
float ypr[3];


void belokKanan(int pwm_l, int pwm_r) {
  motorkanan.setSpeed(pwm_r);
  motorkiri.setSpeed(pwm_l);

  motorkanan.forward();
  motorkiri.forward();
}

void belokKiri(int pwm_l, int pwm_r) {
  motorkanan.setSpeed(pwm_r);
  motorkiri.setSpeed(pwm_l);

  motorkanan.forward();
  motorkiri.forward();
}


bool awal = true;
bool turning1 = false;
bool turning2 = false;
bool turning3 = false;

bool lurus1 = false, 
    lurus2 = false,
    lurus3 = false;

bool mundur = false;

bool maju = false;

int kecepatan_motorKiri;    // Sesuaikan kecepatan sesuai kebutuhan
int kecepatan_motorKanan;  // Sesuaikan kecepatan sesuai kebutuhan
int sudut;
int selisih_kecepatan;

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
unsigned long startTime = millis();
unsigned long duration = 2000; // 2 detik

void encoderleft() {
  unsigned long currentMillis1 = millis();

  if (currentMillis1 - lastDebounceTime1 > debounceDelay1) {
    lastDebounceTime1 = currentMillis1;

    if (digitalRead(ENCLB) == HIGH) {
      encoder_posleft++;
    } else {
      encoder_posleft--;
    }
  }
}

void encoderright() {
  unsigned long currentMillis2 = millis();

  if (currentMillis2 - lastDebounceTime2 > debounceDelay2) {
    lastDebounceTime2 = currentMillis2;

    if (digitalRead(ENCRB) == HIGH) {
      encoder_posright++;
    } else {
      encoder_posright--;
    }
  }
}

void setup() {
  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(115200);
  while (!Serial);

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);  // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    
    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  attachInterrupt(digitalPinToInterrupt(ENCRB), encoderright, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCLB), encoderleft, RISING);
}

void loop() {
  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
  // #ifdef OUTPUT_READABLE_YAWPITCHROLL
    if(awal == true){
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      int kecepatan_motorKiri = 144;
      int kecepatan_motorKanan = 120;
      int sudut = (ypr[0] * 180 / M_PI);
      Serial.print("sudut 0: ");
      Serial.println(sudut);
      int selisih_kecepatan = sudut * 4;  // Sesuaikan faktor pengganda sesuai kebutuhan
      motorkiri.forward();
      motorkanan.forward();
      motorkiri.setSpeed(kecepatan_motorKiri - selisih_kecepatan);
      motorkanan.setSpeed(kecepatan_motorKanan + selisih_kecepatan);
      
    // #endif
      if((encoder_posright >= 24 && encoder_posright <= 25)){
        awal = false;
        turning1 = true;
      }
    }
  //===========================================================================================
    if(turning1 == true){
      motorkiri.stop();
      motorkanan.stop();
      delay(2000);
      while (sudut > -63) {
      belokKiri(10, 120);
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          sudut = (ypr[0] * 180 / M_PI);
          Serial.print("sudut belok 1: ");
          Serial.println(sudut);
        }
      }
      
      motorkiri.stop();
      motorkanan.stop();
      encoder_posleft =0;
      encoder_posright =0;
      delay(2000);
      turning1 = false;
      lurus1 = true;
    }
    if(lurus1 == true){
          //====================================================
            motorkiri.forward();
            motorkanan.forward();
            motorkiri.setSpeed(134);
            motorkanan.setSpeed(120);
        //============================================================
          
          if(encoder_posright >= 18 && encoder_posright <= 18){
            // lurus1 = false;
            lurus1 = false;
            turning2 = true;
          }
        }
  }
     if(turning2 == true){
        motorkiri.stop();
        motorkanan.stop();
        delay(2000);
        while (sudut < -15) {
        belokKanan(120, 10);
          if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            sudut = (ypr[0] * 180 / M_PI);
            Serial.print("sudut belok 2: ");
            Serial.println(sudut);
          }
        }
        motorkiri.stop();
        motorkanan.stop();
        encoder_posleft =0;
        encoder_posright =0;
        delay(2000);
        turning2 = false;
        lurus2 = true;
      }
      if(lurus2 == true){
          //=====================================================
          if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            int kecepatan_motorKiri = 132;
            int kecepatan_motorKanan = 108;
            int sudut = (ypr[0] * 180 / M_PI);
            int selisih_kecepatan = sudut * 3;
            Serial.print("sudut belok 1: ");
            Serial.println(sudut);
            motorkiri.forward();
            motorkanan.forward();
            motorkiri.setSpeed(kecepatan_motorKiri - selisih_kecepatan);
            motorkanan.setSpeed(kecepatan_motorKanan + selisih_kecepatan);

            if((encoder_posright >= 20 && encoder_posright <= 21)){
              lurus2 = false;
              turning3 = true;
            }
          }
        //============================================================
        }
        if(turning3 == true){
          motorkiri.stop();
          motorkanan.stop();
          delay(2000);
          while (sudut <70) {
          belokKanan(130, 10);
            if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
              mpu.dmpGetQuaternion(&q, fifoBuffer);
              mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
              sudut = (ypr[0] * 180 / M_PI);
              Serial.print("terakhir: ");
              Serial.println(sudut);
            }
          }
          motorkiri.stop();
          motorkanan.stop();
          encoder_posleft =0;
          encoder_posright =0;
          delay(2000);
          turning3 = false;
          mundur=true;
        }

        if(mundur==true){
          if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            int kecepatan_motorKiri = 128;
            int kecepatan_motorKanan = 104;
            int sudut = (ypr[0] * 180 / M_PI);
            int selisih_kecepatan = sudut * 3;
            Serial.print("sudut mundur: ");
            Serial.println(sudut);
            motorkiri.forward();
            motorkanan.forward();
            motorkiri.setSpeed(kecepatan_motorKiri - selisih_kecepatan);
            motorkanan.setSpeed(kecepatan_motorKanan + selisih_kecepatan);

            if((encoder_posright >= 2 && encoder_posright <= 3)){
              mundur = false;
              maju = true;
              
            }
          }
        }

        if(maju==true){
          motorkiri.stop();
          motorkanan.stop();
          delay(2000);

          
          while (sudut >-35) {
          belokKiri(10, 130);
            if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
              mpu.dmpGetQuaternion(&q, fifoBuffer);
              mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
              sudut = (ypr[0] * 180 / M_PI);
              Serial.print("terakhir: ");
              Serial.println(sudut);
            }
          }
          motorkiri.stop();
          motorkanan.stop();
          encoder_posleft =0;
          encoder_posright =0;
          delay(2000);
          maju=false;
        }
  
  Serial.print(encoder_posleft);
  Serial.print("   ");
  Serial.println(encoder_posright);
}
