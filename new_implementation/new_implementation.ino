#include <Wire.h>
#include <FaBo9Axis_MPU9250.h>

FaBo9Axis IMU;

unsigned long timer = 0;
float timeStep = 0.01;

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

int gyroPeriod = 100;
int encoderPeriod = 100;

int i;

int freqA;
int freqB;

int ticker;

void setup() {
  // put your setup code here, to run once:
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  pinMode(ina, OUTPUT);
  pinMode(inb, OUTPUT);
  pinMode(inc, OUTPUT);
  pinMode(ind, OUTPUT);
  Serial.begin(115200);

  IMU.begin();

  delay(5000);

  i = 0;
  storeA = 130;
  storeB = 130;
  
  //resetTicker();
  //moveForward(100);
  
  while ( i < 4 ) {
    resetTicker();
    moveForward(27);
    delay(1500);
    resetTicker();
    moveBackward(7);
    delay(1500);
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
  int period = 100;
  unsigned long timeNow;
  int n = 1;

  motorAspeed = storeA;
  motorBspeed = storeB;

  resetMotors();
  
  //requiredSlots = floor(distance / 1.068); //number of slots required to travel = distance / distance per slot
  requiredSlots = floor(distance / 0.534);
  timeNow = millis();
  
  while (ticker < requiredSlots) {
    //this controls the forwards movement
    analogWrite(ina, motorAspeed); 
    digitalWrite(inb, LOW);
    analogWrite(inc, motorBspeed);
    digitalWrite(ind, LOW);

    Serial.println(motorBspeed);
    checkEncoders();
    
    
    /*if (timeNow >= (n * encoderPeriod)) {
      compareEncoders();
      //Serial.print(motorAspeed);
      //Serial.println(motorBspeed);
      n ++;
    }
    
    if (millis() >= timeNow + period) { //not sure about this may not work. idea is to check gyro every 100ms
      Serial.println(yaw);
      timeNow += period;
      gyroCheck();
    } */
  }
  resetMotors();
}

void moveBackward (int distance) {
  int requiredSlots;
  int period = 10;
  unsigned long timeNow;

  motorAspeed = storeA;
  motorBspeed = storeB;

  resetMotors();

  timeNow = millis();

  //requiredSlots = floor(distance / 1.068); //number of slots required to travel = distance / distance per slot
  requiredSlots = floor(distance / 0.534);
  
  while (ticker < requiredSlots) {
    //this controls the forwards movement
    analogWrite(inb, 110); 
    digitalWrite(ina, LOW);
    analogWrite(ind, 110);
    digitalWrite(inc, LOW);

    checkEncoders();
    
    /*if (millis() >= timeNow + period) { //not sure about this may not work. idea is to check gyro every 100ms
      timeNow += period;
      gyroCheck();
    } */
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
   
   while (ticker < ticks - 4) {
      digitalWrite(ina, LOW);
      analogWrite(inb, motorAspeed);
      analogWrite(inc, motorBspeed);
      digitalWrite(ind, LOW);

      checkEncoders();
   }

   resetMotors();
}

void rotateAntiClockwise (int deg) {
   int ticks = 9;
   motorAspeed = 110;
   motorBspeed = 120;

   resetMotors();

   while (ticker < ticks) {
      analogWrite(ina, motorAspeed);
      digitalWrite(inb, LOW);
      digitalWrite(inc, LOW);
      analogWrite(ind, motorBspeed);
   }

   resetMotors();
}

void gyroCheck () {

  IMU.readGyroXYZ(&gx,&gy,&gz);

  // Calculate Pitch, Roll and Yaw
  pitch = pitch + (gy) * timeStep;
  roll = roll + (gx) * timeStep;
  yaw = yaw + (gz) * timeStep;

  if ( yaw  <= 1 && yaw >= -1 ) {
  }
  else {
    if ( yaw < 0 ) {
      motorBspeed -= 10;
    }
    else {
      motorBspeed += 10;
    }
    
  }
}

void checkEncoders() {
  if (digitalRead(encoderA) == HIGH && currentOutputA == 0) {
    freqA ++;
    ticker ++;
    currentOutputA = 1;
  } else if (digitalRead(encoderA) == LOW && currentOutputA == 1){
    freqA ++;
    ticker ++;
    currentOutputA = 0;
  }

  if (digitalRead(encoderB) == HIGH && currentOutputB == 0) {
    freqB ++;
    currentOutputB = 1;
  } else if (digitalRead(encoderB) == LOW && currentOutputB == 1){
    freqB ++;
    currentOutputB = 0;
  }
}

void compareEncoders() {
  int p;
  if (freqA == freqB) {
    //do nothing
    p = 0;
  } else {
    p = (freqA - freqB);
  }

  motorBspeed = motorBspeed + p;
  Serial.print(freqA);
  Serial.println(freqB);
   
  resetFreqs();
}

void resetMotors() {
  digitalWrite(ina, LOW);
  digitalWrite(inb, LOW);
  digitalWrite(inc, LOW);
  digitalWrite(ind, LOW);
}

void resetFreqs() {
  freqA = 0;
  freqB = 0;
}

void resetTicker() {
  ticker = 0;
}

void resetYaw() {
  pitch = 0;
  roll = 0;
  yaw = 0;
}
