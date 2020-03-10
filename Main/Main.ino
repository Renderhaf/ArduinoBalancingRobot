//This is a library that helps use the I2C protocol
#include <Wire.h>
//This is a library that helps use the Radio
#include <SoftwareSerial.h>
//This is a library that helps use the PID calculations efficiently
#include "PID_v1.h"

//The following are values used in oreder to get information from the Gyro/Accelarometer
int minVal = 205, maxVal = 402;

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;s

double x;
double y;
double z;

int xAng, yAng, zAng;

//These values are called the RobotMap, and define the different ports of the arduino
const int pwm = 3;
const int in_1 = 5; 
const int in_2 = 6;
const int MPU_addr=0x68;
SoftwareSerial HC12(10, 11); // HC-12 TX Pin, HC-12 RX Pin
#define distanceSensorLocation A3

//These are interger values used in the loop function 
int currAng;
int currPow;

long duration;
long distance;

//These are values used in the PID control class:
//input - the raw integer sensor value that the PID looper recives
//output - a integer address that the PID puts the output into
//setpoint - the point that the PID tries to get the input to

double input = 0;
double output = 0;
double setpoint = 0;

//These are the PID gains:
//kP - Propotional gains (kP * E = P)
//kI - Intgeral gains (kI * sigma(E) = I)
//kD - Derivative gains (kD * (E-(lastE)) * -1 = D)
//then P + I + D = PID
double Kp = 30;
double Ki = 0;
double Kd = 0.003;

//Defines the PID class for calulating the PID values from the input and gains
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

//These are values used for the rolling median
const int rollLength = 5;
int rollingMedian[rollLength];
int currentMedianArrayLocation = 0;

//These are values used for the sampling rate decrease
int TIMES_SKIP = 10;
int TICKS = 0;

//This is the value recived from the radio, and is used to control robot movements
int radioVal = 0;

//These are the meanings of the values recived from the radio
#define RADIO_BACK_VALUE 0
#define RADIO_FORWARD_VALUE 1
#define RADIO_NEUTRAL_VALUE 2

//Value recived from the distance sensor
float distSensor;

//A function that sorts an array in place by finding the smalling value of a[i:], putting it in at i, then increasing i (insertion sort)
//@param a is an int array start location
//@param n is the length of the int array
void sort(int a[], int n) {
   int i, j, minn, temp;
   for (i = 0; i < n - 1; i++) {
      minn = i;
      for (j = i + 1; j < n; j++)
         if (a[j] < a[minn])
            minn = j;
      temp = a[i];
      a[i] = a[minn];
      a[minn] = temp;
   }
}

//A mutetor function that returns the median value of an array (but destroys the array in the proccess)
//The function sorts the array, then if its of an even length, it returns the avarage of the two median values, and if its of an odd length, it just returns the median value
double getMedian(int a[], int n) { 
    sort(a, a+n); 
  
    if (n % 2 != 0) 
       return (double)a[n/2]; 
      
    return (double)(a[(n-1)/2] + a[n/2])/2.0; 
} 


void setup(){
  //Setting up the MPU (Gyro) using I2C
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  //This sets a value of a register in the gyro to 0, thus making it "wake up"
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  //Setting up the motor contorller
  pinMode(pwm,OUTPUT); 
  pinMode(in_1,OUTPUT); 
  pinMode(in_2,OUTPUT);

  //Setting up the serial and the radio transmitting
  Serial.begin(9600);
  HC12.begin(9600);

  //Setting up some of the PID settings
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255, 255); 

  //Initializing some of the values
  distSensor = 0;
  radioVal = 2;
}
 
void loop(){
  Wire.beginTransmission(MPU_addr);
  //Telling the Gyro which register we want to start with
  Wire.write(0x3B);
  Wire.endTransmission(false);
  
  //Asking for 14 register values, then doing some bitwise operations on them to get the X,Y,and Z acceleration values
  Wire.requestFrom(MPU_addr,14,true);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();

  //Turning the raw gyro values into angles (in radians)
  xAng = map(AcX,minVal,maxVal,-90,90);
  yAng = map(AcY,minVal,maxVal,-90,90);
  zAng = map(AcZ,minVal,maxVal,-90,90);

  //Converting the angles into degrees
  x = RAD_TO_DEG * (atan2(-yAng, -zAng));
  y = RAD_TO_DEG * (atan2(-xAng, -zAng));
  z = RAD_TO_DEG * (atan2(-yAng, -xAng));

  //Applying the sample rate decrease
  if (TICKS % TIMES_SKIP == 0){
    TICKS = 0;
    //putting the raw sensor angle into the rolling median data sample
    rollingMedian[currentMedianArrayLocation] = y;

    //making a new array out of the rolling median array (because the median function is a mutator)
    int cpyarr[rollLength];
    for (int i = 0; i < rollLength; i++){
      cpyarr[i] = rollingMedian[i];
    } 

    //make the current angle the median of the raw angles
    currAng = getMedian(cpyarr, rollLength); 

    // increase the median smaple data location and make it roll over
    currentMedianArrayLocation = (currentMedianArrayLocation + 1) % rollLength;
  }

  //put the current median angle as the PID input, then get the PID output
  input = currAng;
  pid.Compute();
  currPow = output;

  //Read from the distance sensor, and if the detects a close proximity object, it sets the current motor output to 0
  distSensor = analogRead(distanceSensorLocation);
  if (distSensor < 600){
    currPow = 0;
  }

  //if data is being sent over the radio, set the radioVal variable to it
  if (HC12.available()){
    radioVal = HC12.read();
  }

  //if the data sent from the radio reciver is forward or back, set the robot power to forward or back
  if (radioVal != RADIO_NEUTRAL_VALUE){
    if (radioVal == RADIO_BACK_VALUE){
      currPow = 150;
    } else ir (radioVal == RADIO_FORWARD_VALUE){
      currPow = -150;
    }
  }

  //Write the current motor output to the motor
  analogWrite(pwm,abs(currPow));
  if (currPow < 0){
    digitalWrite(in_1,LOW); digitalWrite(in_2,HIGH);
  } else {
    digitalWrite(in_1,HIGH); digitalWrite(in_2,LOW);
  }
  
  //Add 1 to the tick counter - used for sample rate decreasing 
  TICKS++;
}
