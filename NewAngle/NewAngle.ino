#include <Wire.h>
#include <SoftwareSerial.h>
#include "PID_v1.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

//int minVal = 205, maxVal = 402;
//const int MPU_addr=0x68; int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

// MPU control/status vars
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//double x;
//double y;
//double z;

//int xAng, yAng, zAng;

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

double Kp = 37;
double Ki = 0;
double Kd = 0;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

void setup(){

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
//
//  Wire.begin();
//  Wire.beginTransmission(MPU_addr);
//  Wire.write(0x6B);
//  Wire.write(0);
//  Wire.endTransmission(true);
 
  pinMode(13, OUTPUT);

  pinMode(pwm,OUTPUT); pinMode(in_1,OUTPUT); pinMode(in_2,OUTPUT);

  Serial.begin(9600);
  HC12.begin(9600);

  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255, 255);  

  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1688);

  mpu.setDMPEnabled(true);
  attachInterrupt(0, dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();
  dmpReady = true;
  packetSize = mpu.dmpGetFIFOPacketSize();
  
}
 
void loop(){
//  Wire.beginTransmission(MPU_addr);
//  Wire.write(0x3B);
//  Wire.endTransmission(false);
//  Wire.requestFrom(MPU_addr,14,true);
//  AcX=Wire.read()<<8|Wire.read();
//  AcY=Wire.read()<<8|Wire.read();
//  AcZ=Wire.read()<<8|Wire.read();
// 
//  xAng = map(AcX,minVal,maxVal,-90,90);
//  yAng = map(AcY,minVal,maxVal,-90,90);
//  zAng = map(AcZ,minVal,maxVal,-90,90);
//
//  x = RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
//  y = RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
//  z = RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize){
      //no mpu data - performing PID calculations and output to motors     
      pid.Compute();   
      
      //Print the value of Input and Output on serial monitor to check how it is working.
      Serial.print(input); Serial.print(" =>"); Serial.println(output);
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024){
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));
  }
  else if (mpuIntStatus & 0x02){ //DMP data
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      mpu.dmpGetQuaternion(&q, fifoBuffer); //get value for q
      mpu.dmpGetGravity(&gravity, &q); //get value for gravity
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //get value for ypr
 }

  currAng = ypr[3] - 178; //TODO NEED TO CHECK WHICH ONE IS THIS (ypr[x]) --------------------------------------------

  input = currAng;
  pid.Compute();
  currPow = output;
  
  if (currPow > 255){
    currPow = 255;
  }

  analogWrite(pwm,abs(currPow));
  if (currPow < 0){
    digitalWrite(in_1,LOW); digitalWrite(in_2,HIGH);
  } else {
    digitalWrite(in_1,HIGH); digitalWrite(in_2,LOW);
  }

  Serial.print("ANG = "); Serial.print(currAng); Serial.print(" Motor Power = "); Serial.println(currPow);
// 
//  while (HC12.available()) {        // If HC-12 has data
//    Serial.write(HC12.read());      // Send the data to Serial monitor
//  }
////  while (Serial.available()) {      // If Serial monitor has data
////    HC12.write(Serial.read());      // Send that data to HC-12
////  }

//  delay(300);
}
