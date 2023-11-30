#define S0 8
#define S1 9
#define S2 10
#define S3 11
#define OutputSensor 12

int f_red = 0;
int f_green = 0;
int f_blue = 0;

void setup()
{
Serial.begin(9600);
pinMode(S0, OUTPUT);
pinMode(S1, OUTPUT);
pinMode(S2, OUTPUT);
pinMode(S3, OUTPUT);

pinMode(OutputSensor, INPUT);

digitalWrite(S0, HIGH);
digitalWrite(S1, LOW);
}

void loop()
{
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  f_red = pulseIn(OutputSensor, LOW);
//  Serial.print("R = ");
//  Serial.println(f_red);
  delay(500);
  
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  f_green = pulseIn(OutputSensor, LOW);
//  Serial.print("G = ");
//  Serial.println(f_green);
  delay(500);
  
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  f_blue = pulseIn(OutputSensor, LOW);
//  Serial.print("B = ");
//  Serial.println(f_blue);
  delay(500);
  //orange
  if(f_red >= 85 && f_red <= 100 && f_green >= 150 && f_green <= 170 && f_blue >=140 && f_blue <=160){
    Serial.println("Orange");
  }
}
