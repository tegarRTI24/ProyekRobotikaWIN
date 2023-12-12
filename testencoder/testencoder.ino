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

Servo s1,s2,s3;

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
bool turning4 = false;
bool turning5 = false;
bool turning6 = false;

bool lurus1 = false, 
    lurus2 = false,
    lurus3 = false,
    lurus4 = false,
    lurus5 = false;

bool maju = false,maju1=false;

bool servo = false;
bool servo2 = false;
bool motor = false;
bool mundur = false;
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
  s1.attach(8);
  s2.attach(9);
  s3.attach(10);
  s1.write(13);
  delay(15);
  s2.write(90);
  delay(15);
  s3.write(153);
  delay(15);
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
      int kecepatan_motorKiri = 170;
      int kecepatan_motorKanan = 172;
      int sudut = (ypr[0] * 180 / M_PI);
      Serial.print("sudut 0: ");
      Serial.println(sudut);
      int selisih_kecepatan = sudut * 9;  // Sesuaikan faktor pengganda sesuai kebutuhan
      motorkiri.forward();
      motorkanan.forward();
      motorkiri.setSpeed(kecepatan_motorKiri - selisih_kecepatan);
      motorkanan.setSpeed(kecepatan_motorKanan + selisih_kecepatan);
      
    // #endif
      if((encoder_posright >= 21 && encoder_posright <= 22)){
        awal = false;
       turning1 = true;
      }
    }
  //===========================================================================================
    if(turning1 == true){
      motorkiri.stop();
      motorkanan.stop();
      delay(2000);
      while (sudut > -67) {
      belokKiri(10, 170);
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
            motorkanan.setSpeed(137);
        //============================================================
          
          if(encoder_posright >= 9 && encoder_posright <= 10){
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
        while (sudut < -16) {
        belokKanan(140, 10);
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
            int kecepatan_motorKiri = 140;
            int kecepatan_motorKanan = 147;
            int sudut = (ypr[0] * 180 / M_PI);
            int selisih_kecepatan = sudut *5;
            Serial.print("sudut belok 1: ");
            Serial.println(sudut);
            motorkiri.forward();
            motorkanan.forward();
            motorkiri.setSpeed(kecepatan_motorKiri - selisih_kecepatan);
            motorkanan.setSpeed(kecepatan_motorKanan + selisih_kecepatan);

            if((encoder_posright >= 13 && encoder_posright <= 14)){
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
          while (sudut > -64) {
          belokKiri(10, 180);
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
          turning3=false;
          mundur=true;
        }
        if(mundur==true){                    
            motorkiri.backward();
            motorkanan.backward();
            motorkiri.setSpeed(150);
            motorkanan.setSpeed(168);          
            
            if((encoder_posright >= 18 && encoder_posright <= 18)){
              motorkiri.stop();
              motorkanan.stop();
              delay(2000);
              mundur = false;
              servo = true;
            }
          
        }
        if(servo == true){
                for(int i=90;i<=135;i+=1){
                  s2.write(i);
                  delay(15);
                }
                //servo 1 ambil bola
                for(float i=13;i<=90;i++){
                  s1.write(i);
                  delay(15);
                }
                //servo 1 angkat bola
                for(float i=90;i>=13;i--){
                  s1.write(i);
                  delay(15);
                }
                servo = false;
                maju1 =true;
        }
        if(maju1==true){                    
          if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            int kecepatan_motorKiri = 140;
            int kecepatan_motorKanan = 147;
            int sudut = (ypr[0] * 180 / M_PI);
            int selisih_kecepatan = sudut * 3;
            Serial.print("sudut mundur: ");
            Serial.println(sudut);
            motorkiri.forward();
            motorkanan.forward();
            motorkiri.setSpeed(kecepatan_motorKiri - selisih_kecepatan);
            motorkanan.setSpeed(kecepatan_motorKanan + selisih_kecepatan);           
            
            if((encoder_posright >= 28 && encoder_posright <= 29)){
              maju1 = false;
              maju = true;
            }
          }
        }

        if(maju==true){
          motorkiri.stop();
          motorkanan.stop();
          delay(2000);
          while (sudut <-4) {
          belokKiri(10, 180);
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
          turning4=true;
        }
     if(turning4 == true){
          motorkiri.stop();
          motorkanan.stop();
          delay(2000);
          while (sudut >-150) {
          belokKiri(10, 170);
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
          turning4=true;
          servo2=false;
     }
     if(servo2 == true){
          //servo 2 bukak bola
          for (int i  = 135; i >= 90; i -= 5) {
            s2.write(i);
            delay(15);
          }
          delay(2000);
          for (int i  = 90; i <= 135; i += 5) {
            s2.write(i);
            delay(15);
          }
          servo2= false;
          motor=true;
     }
     if(motor==true){
      //motor ngedrop bola
          motorkiriku.forward();
          motorkiriku.setSpeed(40);
          motorkananku.backward();
          motorkananku.setSpeed(40);
          delay(2000);
          motor=false;
          turning5=true;
     }

     if(turning5 == true){
          motorkiri.stop();
          motorkanan.stop();
          delay(2000);
          while (sudut >40) {
          belokKiri(10, 170);
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
          turning5=false;
          lurus5=true;
     }
     if(lurus5 == true){
          //=====================================================
          if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            int kecepatan_motorKiri = 140;
            int kecepatan_motorKanan = 147;
            int sudut = (ypr[0] * 180 / M_PI);
            int selisih_kecepatan = sudut *3;
            Serial.print("sudut belok 1: ");
            Serial.println(sudut);
            motorkiri.forward();
            motorkanan.forward();
            motorkiri.setSpeed(kecepatan_motorKiri - selisih_kecepatan);
            motorkanan.setSpeed(kecepatan_motorKanan + selisih_kecepatan);

            if((encoder_posright >= 10 && encoder_posright <= 11)){
              lurus5 = false;
              turning6 = true;
            }
          }
        //============================================================
        }
     
  
  Serial.print(encoder_posleft);
  Serial.print("   ");
  Serial.println(encoder_posright);
}
