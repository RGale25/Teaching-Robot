#include <Wire.h>
//#include <FaBo9Axis_MPU9250.h>

#include "MPU9250.h"
#include <PID_v1.h>

//FaBo9Axis IMU;
MPU9250 IMU(Wire,0x68);

int fA;
int fB;

int motorAspeed = 100;
int motorBspeed = 100;

unsigned long newTimeA;
unsigned long prevTimeA;

unsigned int npwA;
unsigned int lpwA;

unsigned long newTimeB;
unsigned long prevTimeB;

unsigned int npwB;
unsigned int lpwB;

int currentOutputA;
int currentOutputB;

float pitch = 0;
float roll = 0;
float yaw = 0;

float gx, gy, gz;

int mSpeed = 100;

int i;
unsigned long timePassed;
unsigned int interval = 10; //0.1 seconds


double Setpoint = 5, Input, Output;

double Kp= 1, Ki= 0, Kd=0;

PID gyroPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(2, INPUT_PULLUP);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(11, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(2), incFA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), incFB, CHANGE);

  IMU.begin();

  delay(2000);
  lpwA = 0;
  prevTimeA = 0;
  lpwB = 0;
  prevTimeB = 0;
  i = 0;
  
  analogWrite(5, mSpeed);
  analogWrite(6, 0);
  analogWrite(9, mSpeed);
  analogWrite(11, 0);
  
 gyroPID.SetMode(AUTOMATIC);

}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(5, motorAspeed);
  analogWrite(6, 0);
  analogWrite(9, motorBspeed);
  analogWrite(11, 0);

  //set the prevTime = millis() before the loop
  //set the lpw = 0; before the while loop
  
  timePassed = millis();
  if (timePassed >= (i * interval)) {
    gyroCheck();
    /*Serial.print("A : ");
    Serial.print(fA);
    Serial.print("  B : ");
    Serial.println(fB);*/
    i ++;
  }
  
}

void gyroCheck () {
  int p = - 5;
  int e;
  
  //IMU.readGyroXYZ(&gx,&gy,&gz);
  IMU.readSensor();
  gz = IMU.getGyroZ_rads();
  
  //yaw = yaw + (gz) * 0.01;
  e = 0 - gz;
  //if ( e < - 0.5 || e > 0.5) {
  motorBspeed += (p * e);
  //}
  Serial.print(gz);
  Serial.print("   ");
  Serial.println(p * e);
  /*Input = yaw;
  
  gyroPID.Compute();
  motorBspeed += Output;
  Serial.println(Output);*/
}

  void incFA () {
    if (digitalRead(2) == HIGH && currentOutputA == 0) {
      calculateFA();
      currentOutputA = 1;
    } else if (digitalRead(2) == LOW && currentOutputA == 1){
      calculateFA();
      currentOutputA = 0;
    }
  }

  void incFB () {
    if (digitalRead(3) == HIGH && currentOutputB == 0) {
      calculateFB();
      currentOutputB = 1;
    } else if (digitalRead(3) == LOW && currentOutputB == 1){
      calculateFB();
      currentOutputB = 0;
    }
  }



  void calculateFA () {
    newTimeA = millis();
    //Serial.println(newTime);
    npwA = newTimeA - prevTimeA;
    //Serial.println(lpw);
    fA = (1000 / ((npwA + lpwA) / 2));
    prevTimeA = newTimeA;
    lpwA = npwA;
  }

  void calculateFB () {
    newTimeB = millis();
    //Serial.println(newTime);
    npwB = newTimeB - prevTimeB;
    //Serial.println(lpw);
    fB = (1000 / ((npwB + lpwB) / 2));
    prevTimeB = newTimeB;
    lpwB = npwB;
  }
