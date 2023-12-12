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

#include <L298N.h>
volatile long int encoder_posleft = 0;
volatile long int encoder_posright = 0;
volatile unsigned long lastDebounceTime1 = 0;
volatile unsigned long lastDebounceTime2 = 0;
unsigned long debounceDelay1 = 50;
unsigned long debounceDelay2 = 50;
unsigned long timeku=0;
L298N motorkiri (ENB,IN3,IN4); //kiri
L298N motorkanan (ENA,IN1,IN2); //kanan

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
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(ENCRB), encoderright, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCLB), encoderleft, RISING);

}

void loop() {
  motorkiri.forward();
  motorkiri.setSpeed(120);
  motorkanan.forward();
  motorkanan.setSpeed(147);
  Serial.print(encoder_posleft);
  Serial.print(" ");
  Serial.println(encoder_posright);
  if(encoder_posright >= 14){
    motorkanan.setSpeed(0);
    motorkiri.setSpeed(0);
  }
}
