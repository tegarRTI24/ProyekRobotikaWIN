#include <Servo.h>

Servo s1;

void setup() {
  Serial.begin(9600);
  s1.attach(8);
  s1.write(13);
  delay(15);
}

//s3 153 - 90
//s1 13 - 88
//s2 90-135

void loop() {
  for (int i  = 13; i <=88; i += 1) {
    s1.write(i);
    delay(15);
  }
  for (int i  = 88; i >= 13; i -= 1) {
    s1.write(i);
    delay(15);
  }
  while(true){
    
  }
}
