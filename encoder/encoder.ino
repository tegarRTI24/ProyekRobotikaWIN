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
#define IN3 51
#define IN4 53
#define ENB 46 //kiri
#define ENCRA 41
#define ENCRB 2
#define ENCLA 39
#define ENCLB 3

void maju() {
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  analogWrite(ENA,104);
  analogWrite(ENB,128);
}

void kanan(){
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  analogWrite(ENA,10);
  analogWrite(ENB,120);
}

void kiri(){
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  analogWrite(ENA,120);
  analogWrite(ENB,10);
}

void stopp(){
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  analogWrite(ENA,10);
  analogWrite(ENB,10);
}

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
  maju();
  if(encoder_posright >= 22 && encoder_posleft >= 22) {
    kiri();
    if(encoder_posright >= 29 && encoder_posleft >= 26) {
      maju();
      if(encoder_posright >= 31 && encoder_posleft >= 30) {
        kanan();
        if(encoder_posright >= 33 && encoder_posleft >= 42) {
          maju();
          if(encoder_posright >= 35 && encoder_posleft >= 45) {
            stopp();
          }
        }
      }  
    }
  }
  
  Serial.print(encoder_posleft);
  Serial.print("   ");
  Serial.println(encoder_posright);

}
