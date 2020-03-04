#include <SoftwareSerial.h>
//Remote

SoftwareSerial HC12(10, 11); // HC-12 TX Pin, HC-12 RX Pin
double xval, yval;
int sendVal;

void setup() {
  Serial.begin(9600);             // Serial port to computer
  HC12.begin(9600);               // Serial port to HC12
}
void loop() {
  xval = analogRead(A0);
  yval = analogRead(A1);

  if (xval > 768){
    sendVal = 0;
  } else if (xval < 256) {
    sendVal = 1;
  } else {
    sendVal = 2;
  }

  HC12.write(sendVal);

  Serial.print(yval); Serial.println(sendVal);
}
