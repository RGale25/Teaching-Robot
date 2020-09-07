#include <Wire.h>
#include <FaBo9Axis_MPU9250.h>
#include <PID_v1.h>

FaBo9Axis IMU;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;

float gx, gy, gz;

int encoderA = 2;
int encoderB = 3;
int ina = 5;
int inb = 6;
int inc = 9;
int ind = 11;

int motorAspeed;
int motorBspeed;

int storeA;
int storeB;

int currentOutputA;
int currentOutputB;

unsigned long newTimeA;
unsigned long prevTimeA;

unsigned int npwA;
unsigned int lpwA;

unsigned long newTimeB;
unsigned long prevTimeB;

unsigned int npwB;
unsigned int lpwB;

int gyroPeriod = 100;
int encoderPeriod = 50;

int i;

int freqA;
int freqB;

int ticker;

double Setpoint = 0, Input, Output;
double Kp=-2, Ki=0, Kd=0;

PID gyroPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  // put your setup code here, to run once:
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  pinMode(ina, OUTPUT);
  pinMode(inb, OUTPUT);
  pinMode(inc, OUTPUT);
  pinMode(ind, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoderA), incFA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), incFB, CHANGE);
  Serial.begin(115200);

  IMU.begin();
  gyroPID.SetMode(AUTOMATIC);

  delay(5000);

  i = 0;
  storeA = 130;
  storeB = 140;
  
  //resetTicker();
  //moveForward(100);
  
  while ( i < 4 ) {
    resetTicker();
    moveForward(27);
    delay(1500);
    resetTicker();
    moveBackward(7);
    delay(1500);
    Serial.println("here");
    resetTicker();
    rotateClockwise(90);
    delay(1500);
    i++;
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}

void moveForward (int distance) {
  int requiredSlots;
  unsigned long timeNow;

  motorAspeed = storeA;
  motorBspeed = storeB;

  requiredSlots = floor(distance / 0.534);
  timeNow = millis();

  lpwA = 0;
  lpwB = 0;
  prevTimeA = millis();
  prevTimeB = prevTimeA;
  
  resetMotors();
  resetTicker();
  
  while (ticker < requiredSlots) {
    //this controls the forwards movement
    analogWrite(ina, motorAspeed); 
    digitalWrite(inb, LOW);
    analogWrite(inc, motorBspeed);
    digitalWrite(ind, LOW);

    Serial.println(ticker);
    
    if (millis() >= timeNow + encoderPeriod) { //completes task every ecnoder period
      compareEncoders();
      timeNow += encoderPeriod;
      
    }
    
    if (millis() >= timeNow + gyroPeriod) { //completes task every gyro period
      gyroCheck();
      timeNow += gyroPeriod;
    }
  }
  resetMotors();
}

void moveBackward (int distance) {
  int requiredSlots;
  unsigned long timeNow;

  motorAspeed = storeA;
  motorBspeed = storeB;

  requiredSlots = floor(distance / 0.534);
  timeNow = millis();

  lpwA = 0;
  lpwB = 0;
  prevTimeA = millis();
  prevTimeB = prevTimeA;
  
  resetMotors();
  
  while (ticker < requiredSlots) { //this controls the backwards movement
    digitalWrite(ina, LOW);
    analogWrite(inb, motorAspeed); 
    digitalWrite(inc, LOW);
    analogWrite(ind, motorBspeed);

    Serial.println(ticker);
    
    if (millis() >= timeNow + encoderPeriod) { //completes task every ecnoder period
      compareEncoders();
      timeNow += encoderPeriod;
      
    }
    
    if (millis() >= timeNow + gyroPeriod) { //completes task every gyro period
      gyroCheck();
      timeNow += gyroPeriod;
    }
  }
  resetMotors();
}

void rotateClockwise (int deg) {
   int ticks;
   
   storeA = motorAspeed;
   storeB = motorBspeed;
   
   motorAspeed = 130;
   motorBspeed = 140;

   //ticks = floor(deg / 9.81);
   ticks = floor(deg / 4.905);

   resetMotors();
   
   while (ticker < ticks - 5) {
      digitalWrite(ina, LOW);
      analogWrite(inb, motorAspeed);
      analogWrite(inc, motorBspeed);
      digitalWrite(ind, LOW);

      Serial.println(ticker);
   }

   resetMotors();
}

void rotateAntiClockwise (int deg) {
   int ticks;
   
   storeA = motorAspeed;
   storeB = motorBspeed;
   
   motorAspeed = 130;
   motorBspeed = 140;

   //ticks = floor(deg / 9.81);
   ticks = floor(deg / 4.905);
   
   resetMotors();
   Serial.println(ticks);
   while (ticker < ticks) {
      analogWrite(ina, motorAspeed);
      digitalWrite(inb, LOW);
      digitalWrite(inc, LOW);
      analogWrite(ind, motorBspeed);
      Serial.println(ticker);
   }

   resetMotors();
}

void gyroCheck () {
  int p = -2;
  int e;
  
  IMU.readGyroXYZ(&gx,&gy,&gz);
  
  yaw = yaw + (gz) * 0.1;

  Input = yaw;
  Serial.println(yaw);
  gyroPID.Compute();
  motorBspeed += Output;
  
}

void compareEncoders() {
  int p = 0.75;
  int e;

  e = freqA - freqB;
  motorBspeed = motorBspeed + (p * e);
}

void incFA() {
  if (digitalRead(encoderA) == HIGH && currentOutputA == 0) {
      calculateFA;
      ticker ++;
      currentOutputA = 1;
    } else if (digitalRead(encoderA) == LOW && currentOutputA == 1){
      calculateFA;
      ticker ++;
      currentOutputA = 0;
    }
}

void incFB() {
  if (digitalRead(encoderB) == HIGH && currentOutputB == 0) {
      calculateFB();
      currentOutputB = 1;
    } else if (digitalRead(encoderB) == LOW && currentOutputB == 1){
      calculateFB();
      currentOutputB = 0;
    }
}

void calculateFA() {
    newTimeA = millis();
    npwA = newTimeA - prevTimeA;
    freqA = (1000 / ((npwA + lpwA) / 2));
    prevTimeA = newTimeA;
    lpwA = npwA;
}

void calculateFB() {
    newTimeB = millis();
    npwB = newTimeB - prevTimeB;
    freqB = (1000 / ((npwB + lpwB) / 2));
    prevTimeB = newTimeB;
    lpwB = npwB;
}


void resetMotors() {
  digitalWrite(ina, LOW);
  digitalWrite(inb, LOW);
  digitalWrite(inc, LOW);
  digitalWrite(ind, LOW);
}

void resetTicker() {
  ticker = 0;
}

void resetYaw() {
  pitch = 0;
  roll = 0;
  yaw = 0;
}
