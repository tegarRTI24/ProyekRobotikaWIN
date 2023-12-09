volatile long int encoder_posleft = 0;
volatile long int encoder_posright = 0;
volatile unsigned long lastDebounceTime1 = 0;
volatile unsigned long lastDebounceTime2 = 0;
unsigned long debounceDelay1 = 50;
unsigned long debounceDelay2 = 50;
unsigned long timeku=0;

#define IN1 47
#define IN2 49
#define ENA 45
#define IN3 51
#define IN4 53
#define ENB 46
#define ENCRA 41
#define ENCRB 2
#define ENCLA 39
#define ENCLB 3


void setup() {
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(ENCRB), encoderright, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCLB), encoderleft, RISING);
}

void loop() {
  maju();
  if(encoder_posright >= 30) {
    stopp();
    
  }
//  if(encoder_posright >= 101) {
//      maju();
//      
//    }
//    if(encoder_posright >=150) {
//        kanan();
//        
//      }
//      if(encoder_posleft >=151) {
//          maju();
//          
//        }
//        if(encoder_posright >= 200) {
//            stopp();
//          }
  Serial.print(encoder_posleft);
  Serial.print("   ");
  Serial.println(encoder_posright);
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

void maju() {
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  analogWrite(ENA,170);
  analogWrite(ENB,200);
}

void kiri(){
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  analogWrite(ENA,150);
  analogWrite(ENB,150);
  Serial.print("kiri  ");
}

void kanan(){
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  analogWrite(ENA,150);
  analogWrite(ENB,150);
  Serial.print("kanan  ");
}

void stopp(){
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
  analogWrite(ENA,0);
  analogWrite(ENB,0);
}
