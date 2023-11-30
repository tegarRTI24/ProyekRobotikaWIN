void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN,OUTPUT);
}

void loop() {
  if(Serial.available()>0){
    char value = Serial.read();

    if(value == 'F'){
      digitalWrite(LED_BUILTIN,HIGH);
      Serial.print("Nyala");
    }if (value == 'B'){
      digitalWrite(LED_BUILTIN,LOW);
      Serial.print("Mati");
    }
  }

}
