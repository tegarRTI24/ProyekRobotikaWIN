#include <AFMotor.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

AF_DCMotor motor_kiri(1);
AF_DCMotor motor_kanan(2);

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 20  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13       // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define IrKiriDepan 22 
#define IrKiriBelakang 24 
#define IrKananDepan 48
#define IrKananBelakang 50
bool blinkState = false;

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

bool awal = true;
bool turning1 = false;
bool turning2 = false;
bool turning3 = false;

bool lurus1 = false, 
    lurus2 = false,
    lurus3 = false;

void belokKanan(int pwm_l, int pwm_r) {
  motor_kanan.setSpeed(pwm_r);
  motor_kiri.setSpeed(pwm_l);

  motor_kanan.run(BACKWARD);
  motor_kiri.run(BACKWARD);
}

void belokKiri(int pwm_l, int pwm_r) {
  motor_kanan.setSpeed(pwm_r);
  motor_kiri.setSpeed(pwm_l);

  motor_kanan.run(FORWARD);
  motor_kiri.run(FORWARD);
}

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

  //IR Sensor
  pinMode(IrKiriBelakang, INPUT);
  pinMode(IrKiriDepan, INPUT);
  pinMode(IrKananBelakang, INPUT);
  pinMode(IrKananDepan, INPUT);

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  if (!dmpReady) return;
  //lurus pertama=============================================================================
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
  // #ifdef OUTPUT_READABLE_YAWPITCHROLL
    if(awal == true){
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      int kecepatan_motorKiri = 100;
      int kecepatan_motorKanan = 120;
      int sudut = (ypr[0] * 180 / M_PI);
      Serial.print("sudut 0: ");
      Serial.println(sudut);
      int selisih_kecepatan = sudut * 5;  // Sesuaikan faktor pengganda sesuai kebutuhan
      motor_kiri.run(BACKWARD);
      motor_kanan.run(FORWARD);
      motor_kiri.setSpeed(kecepatan_motorKiri - selisih_kecepatan);
      motor_kanan.setSpeed(kecepatan_motorKanan + selisih_kecepatan);
      
    // #endif
      int irSensorValueKanan = digitalRead(IrKananBelakang);
      if(irSensorValueKanan == HIGH){
        awal = false;
        turning1 = true;
      }
    }
  
  //===========================================================================================
  if(turning1 == true){
    motor_kiri.run(RELEASE);
    motor_kanan.run(RELEASE);
    delay(500);
    while (sudut < 80) {
    belokKanan(150, 170);
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        sudut = (ypr[0] * 180 / M_PI);
        Serial.print("sudut belok 1: ");
        Serial.println(sudut);
      }
    }
    motor_kiri.run(RELEASE);
    motor_kanan.run(RELEASE);
    delay(1000);
    turning1 = false;
    lurus1 = true;
  }

      if(lurus1 == true){
        //=====================================================
          motor_kiri.run(BACKWARD);
          motor_kanan.run(FORWARD);
          motor_kiri.setSpeed(100);
          motor_kanan.setSpeed(120);
      //============================================================
        int irSensorValueKiri = digitalRead(IrKiriBelakang);
        if(irSensorValueKiri == HIGH){
          // lurus1 = false;
          lurus1 = false;
          turning2 = true;
        }
      }
    }

      if(turning2 == true){
        while (sudut > 10) {
        belokKiri(150, 170);
          if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            sudut = (ypr[0] * 180 / M_PI);
            Serial.print("sudut belok 2: ");
            Serial.println(sudut);
          }
        }
        motor_kiri.run(RELEASE);
        motor_kanan.run(RELEASE);
        delay(2000);
        turning2 = false;
        lurus2 = true;
      }

        if(lurus2 == true){
          //=====================================================
          if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            int kecepatan_motorKiri = 100;
            int kecepatan_motorKanan = 120;
            int sudut = (ypr[0] * 180 / M_PI);
            int selisih_kecepatan = sudut * 5;
            Serial.print("sudut belok 1: ");
            Serial.println(sudut);
            motor_kiri.run(BACKWARD);
            motor_kanan.run(FORWARD);
            motor_kiri.setSpeed(kecepatan_motorKiri - selisih_kecepatan);
            motor_kanan.setSpeed(kecepatan_motorKanan + selisih_kecepatan);

            int kiri = digitalRead(IrKiriBelakang);
            // int kanan = digitalRead(IrKananBelakang);
            if(kiri == HIGH){
              lurus2 = false;
              turning3 = true;
            }
          }
        //============================================================
        }
        if(turning3 == true){
          while (sudut > -80) {
          belokKiri(150, 180);
            if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
              mpu.dmpGetQuaternion(&q, fifoBuffer);
              mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
              sudut = (ypr[0] * 180 / M_PI);
              Serial.print("terakhir: ");
              Serial.println(sudut);
            }
          }
          motor_kiri.run(FORWARD);
          motor_kanan.run(BACKWARD);
          delay(200);
          motor_kiri.run(RELEASE);
          motor_kanan.run(RELEASE);
          turning3 = false;

        }
}
  // while(true){

  // }
