#include <Wire.h>
#include <SoftwareSerial.h>
#include "PID_v1.h"

int minVal = 205, maxVal = 402;
const int MPU_addr=0x68; int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

double x;
double y;
double z;

int xAng, yAng, zAng;

const int pwm = 3;

const int in_1 = 5; const int in_2 = 6;
int currAng;
int currPow;

SoftwareSerial HC12(10, 11); // HC-12 TX Pin, HC-12 RX Pin

const int trigPin = 3;
const int echoPin = 4;
long duration;
long distance;

double input = 0;
double output = 0;
double setpoint = 0;

double Kp = 40;
double Ki = 0;
double Kd = 0.001;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

const int rollLength = 5;
int rollingMedian[rollLength];
int currentMedianArrayLocation = 0;

int TIMES_SKIP = 10;
int TICKS = 0;

void sort(int a[], int n) {
   int i, j, min, temp;
   for (i = 0; i < n - 1; i++) {
      min = i;
      for (j = i + 1; j < n; j++)
         if (a[j] < a[min])
            min = j;
      temp = a[i];
      a[i] = a[min];
      a[min] = temp;
   }
}

double getMedian(int a[], int n) 
{ 
    sort(a, a+n); 
  
    if (n % 2 != 0) 
       return (double)a[n/2]; 
      
    return (double)(a[(n-1)/2] + a[n/2])/2.0; 
} 

void setup(){

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
 
  pinMode(13, OUTPUT);

  pinMode(pwm,OUTPUT); pinMode(in_1,OUTPUT); pinMode(in_2,OUTPUT);

  Serial.begin(9600);
  HC12.begin(9600);

  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255, 255);  
}
 
void loop(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
 
  xAng = map(AcX,minVal,maxVal,-90,90);
  yAng = map(AcY,minVal,maxVal,-90,90);
  zAng = map(AcZ,minVal,maxVal,-90,90);

  x = RAD_TO_DEG * (atan2(-yAng, -zAng));
  y = RAD_TO_DEG * (atan2(-xAng, -zAng));
  z = RAD_TO_DEG * (atan2(-yAng, -xAng));

  if (TICKS % TIMES_SKIP == 0){
    TICKS = 0;
    rollingMedian[currentMedianArrayLocation] = y;

    int cpyarr[rollLength];
    for (int i = 0; i < rollLength; i++){
      cpyarr[i] = rollingMedian[i];
    }
    currAng = getMedian(cpyarr, rollLength); 

    currentMedianArrayLocation = (currentMedianArrayLocation + 1) % 5;
  }

  input = currAng;
  pid.Compute();
  currPow = output;

//  if (currPow > 255){
//    currPow = 255;
//  } else if (currPow < -255){
//    currPow = -255;
//  }

  analogWrite(pwm,abs(currPow));
  if (currPow < 0){
    digitalWrite(in_1,LOW); digitalWrite(in_2,HIGH);
  } else {
    digitalWrite(in_1,HIGH); digitalWrite(in_2,LOW);
  }

  Serial.print(currAng);
//  Serial.print("ANG = "  ); Serial.println(currAng); 
//  Serial.print(" Motor Power = "); Serial.println(currPow);
// 
//  while (HC12.available()) {        // If HC-12 has data
//    Serial.write(HC12.read());      // Send the data to Serial monitor
//  }
////  while (Serial.available()) {      // If Serial monitor has data
////    HC12.write(Serial.read());      // Send that data to HC-12
////  }

//  delay(300);
  TICKS++;
}
