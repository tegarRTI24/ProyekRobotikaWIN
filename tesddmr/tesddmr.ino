#include <Arduino.h>

#define ENCA1 2 
#define ENCA2 41 

#define ENCB1 3
#define ENCB2 39

#define PWMA 45
#define INA2 47
#define INA1 49

#define PWMB 46
#define INB2 51
#define INB1 53

float speedA, speedB;
float ratio = 0.03293;
const int pulseInterval = 10;
volatile int CountA = 0, CountB = 0;
int posA = 0, posB = 0;
long waktu_sebelumA = 0, waktu_sebelumB = 0;
float Error_sebelumA = 0, Error_sebelumB = 0;
float Error_integralA = 0, Error_integralB = 0;

int targetA = 4 / ratio;
int targetB = 4 / ratio;

void readEncoderA() {
  static unsigned long previousTimeA = 0;
  static int previousCountA = 0;
  if (millis() - previousTimeA >= pulseInterval) {
    previousTimeA = millis();

    int pulseCountA = CountA - previousCountA;
    previousCountA = CountA;
    float distanceTraveledA = pulseCountA * ratio;
    speedA = distanceTraveledA / (pulseInterval / 1000.0);

    // Hitung jumlah pulse per 10ms
    if (digitalRead(ENCA1)) {
      if (digitalRead(ENCA2))
        CountA++;
      else
        CountA--;
    } else {
      if (digitalRead(ENCA2))
        CountA--;
      else
        CountA++;
    }
  }
}

void readEncoderB() {
  static unsigned long previousTimeB = 0;
  static int previousCountB = 0;
  if (millis() - previousTimeB >= pulseInterval) {
    previousTimeB = millis();

    int pulseCountB = CountB - previousCountB;
    previousCountB = CountB;
    float distanceTraveledB = pulseCountB * ratio;
    speedB = distanceTraveledB / (pulseInterval / 1000.0);

    // Hitung jumlah pulse per 10ms
    if (digitalRead(ENCB1)) {
      if (digitalRead(ENCB2))
        CountB--;
      else
        CountB++;
    } else {
      if (digitalRead(ENCB2))
        CountB++;
      else
        CountB--;
    }
  }
}

void setMotorA(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void setMotorB(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void belokKanan(unsigned char pwm_r, unsigned char pwm_l) {
  analogWrite(PWMA, pwm_l);
  analogWrite(PWMB, pwm_r);

  digitalWrite(INA1, LOW);
  digitalWrite(INA2, HIGH);

  digitalWrite(INB1, HIGH);
  digitalWrite(INB2, LOW);
}

void belokKiri(unsigned char pwm_r, unsigned char pwm_l) {
  analogWrite(PWMA, pwm_l);
  analogWrite(PWMB, pwm_r);

  digitalWrite(INA1, HIGH);
  digitalWrite(INA2, LOW);

  digitalWrite(INB1, LOW);
  digitalWrite(INB2, HIGH);
}

void PID_A() {
  float kp_A = 4.0;
  float kd_A = 0.025;
  float ki_A = 0.02;
  long waktu_sekarangA = micros();
  float delta_waktuA = ((float)(waktu_sekarangA - waktu_sebelumA)) / (1.0e6);
  waktu_sebelumA = waktu_sekarangA;
  noInterrupts();
  posA = CountA;
  interrupts();
  int ErrorA = posA - targetA;
  float derivativeA = (ErrorA - Error_sebelumA) / (delta_waktuA);
  Error_integralA = Error_integralA + ErrorA * delta_waktuA;
  float SumA = kp_A * ErrorA + kd_A * derivativeA + ki_A * Error_integralA;
  float pwma = fabs(SumA);

  if (pwma > 150) {
    pwma = 150;
  }
  int dirA = 1;
  if (SumA < 0) {
    dirA = -1;
  }

  setMotorA(dirA, pwma, PWMA, INA1, INA2);
  Error_sebelumA = ErrorA;
}

void PID_B() {
  float kp_B = 4.0;
  float kd_B = 0.025;
  float ki_B = 0.02;
  long waktu_sekarangB = micros();
  float delta_waktuB = ((float)(waktu_sekarangB - waktu_sebelumB)) / (1.0e6);
  waktu_sebelumB = waktu_sekarangB;
  noInterrupts();
  posB = CountB;
  interrupts();
  int ErrorB = posB - targetB;
  float derivativeB = (ErrorB - Error_sebelumB) / (delta_waktuB);
  Error_integralB = Error_integralB + ErrorB * delta_waktuB;
  float SumB = kp_B * ErrorB + kd_B * derivativeB + ki_B * Error_integralB;
  float pwmb = fabs(SumB);

  if (pwmb > 150) {
    pwmb = 150;
  }
  int dirB = 1;
  if (SumB < 0) {
    dirB = -1;
  }
  setMotorB(dirB, pwmb, PWMB, INB1, INB2);
  Error_sebelumB = ErrorB;
}

void setup() {
  Serial.begin(9600);
  pinMode(ENCA1, INPUT);
  pinMode(ENCA2, INPUT);
  pinMode(ENCB1, INPUT);
  pinMode(ENCB2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA1), readEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB1), readEncoderB, CHANGE);

  pinMode(PWMA, OUTPUT);
  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(INB2, OUTPUT);
}

void loop() {
  Serial.print(posA);
  Serial.print("   ");
  Serial.println(targetA);
  for (int i = 0; i < 12000; i++) {
      PID_A();
      PID_B();
  }
  if(posA>=targetA){
    belokKanan(0, 0);
  }
  // Add your motor control logic here
  while(true){
    
  }
}
