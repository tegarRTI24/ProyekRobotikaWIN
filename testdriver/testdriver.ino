const int in3=7;
const int in4=8;
const int ENB=9;

void setup() {
  Serial.begin(9600);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
}

void loop() {
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
  analogWrite(ENB,150);

}
