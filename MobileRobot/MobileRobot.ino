#include<L298N.h>
#include<QTRSensors.h>
#include<BluetoothSerial.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//waiting pin esp32 to fix pin 
const int IN1 = 2;    
const int IN2 = 3;    
const int IN3 = 4;
const int ENA = 5;
const int ENB = 6;     
const int IN4 = 7;

const int offsetA = 1;
const int offsetB = 1;

L298N motor1 (ENA, IN1, IN2);
L298N motor1 (ENB, IN3, IN4);

QTRSensors qtr;
BluetoothSerial SerialBT;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
int threshold[SensorCount];

float kp=0;
float ki=0;
float kd=0;

uint8_t multiP = 1;
uint8_t multiI  = 1;
uint8_t multiD = 1;
uint8_t Kpfinal;
uint8_t Kifinal;
uint8_t Kdfinal;
float Pvalue;
float Ivalue;
float Dvalue;

boolean onoff = false;

int val, cnt = 0, v[3];

uint16_t position;
int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 180;

void setup() {
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){34,35,32,33,25,26,27,14}, SensorCount);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
  
  Serial.begin(115200);
  SerialBT.begin();
  Serial.println("Bluetooth Started! Ready to pair...");
  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    threshold[i] = (qtr.calibrationOn.minimum[i] + qtr.calibrationOn.maximum[i])/2;
    Serial.print(threshold[i]);
    Serial.print("  ");
  }
  Serial.println();

  delay(1000);
}

void loop() {
  if (SerialBT.available()){
    while(SerialBT.available() == 0);
    valuesread();
    processing();
  }
  if (onoff == true){
    robot_control();
  }
  else if(onoff == false){
    motor1.stop();
    motor2.stop();
  }
}

void robot_control(){
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 4000 (for a white line, use readLineWhite() instead)
  position = qtr.readLineBlack(sensorValues);
  error = 2000 - position;
  while(sensorValues[0]>=980 && sensorValues[1]>=980 && sensorValues[2]>=980 && sensorValues[3]>=980 && sensorValues[4]>=980 && sensorValues[5]>=980 && sensorValues[6]>=980 && sensorValues[7]>=980){ // A case when the line follower leaves the line
    if(previousError>0){       //Turn left if the line was to the left before
      motor_drive(-180,180);
    }
    else{
      motor_drive(180,-180); // Else turn right
    }
    position = qtr.readLineBlack(sensorValues);
  }
  
  PID_Linefollow(error);
  //PID_Linefollow(error);
}
void PID_Linefollow(int error){
    P = error;
    I = I + error;
    D = error - previousError;
    
    Pvalue = (Kp/pow(10,multiP))*P;
    Ivalue = (Ki/pow(10,multiI))*I;
    Dvalue = (Kd/pow(10,multiD))*D; 

    float PIDvalue = Pvalue + Ivalue + Dvalue;
    previousError = error;

    lsp = lfspeed - PIDvalue;
    rsp = lfspeed + PIDvalue;

    if (lsp > 200) {
      lsp = 200;
    }
    if (lsp < -200) {
      lsp = -200;
    }
    if (rsp > 200) {
      rsp = 200;
    }
    if (rsp < -200) {
      rsp = -200;
    }
    motor_drive(lsp,rsp);
}
//This void delimits each instruction.
//The  Arduino knows that for each instruction it will receive 2 bytes.
void valuesread()  {
  val = SerialBT.read();
  cnt++;
  v[cnt] = val;
  if (cnt == 2)
    cnt = 0;
}

//In this void the the 2 read values are assigned.
void  processing() {
  int a = v[1];
  if (a == 1) {
    Kp = v[2];
  }
  if (a == 2) {
    multiP = v[2];
  }
  if (a == 3) {
    Ki = v[2];
  }
  if (a == 4) {
    multiI = v[2];
  }
  if (a == 5) {
    Kd  = v[2];
  }
  if (a == 6) {
    multiD = v[2];
  }
  if (a == 7)  {
    onoff = v[2];
  }
}
void motor_drive(int left, int right){
  
  if(right>0)
  {
    motor2.setSpeed(right);
    motor2.forward();
  }
  else 
  {
    motor2.setSpeed(right);
    motor2.backward();
  }
  
 
  if(left>0)
  {
    motor1.setSpeed(left);
    motor1.forward();
  }
  else 
  {
    motor1.setSpeed(left);
    motor1.backward();
  }
}
