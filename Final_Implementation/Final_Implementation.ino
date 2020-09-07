int encoderA = 2;
int encoderB = 3;
int ina = 5;
int inb = 6;
int inc = 9;
int ind = 11;

int motorAspeed;
int motorBspeed;

int currentOutputA;
int currentOutputB;

int i;

int freqA;
int freqB;

void setup() {
  // put your setup code here, to run once:
  pinMode(encoderA, INPUT);
  //attachInterrupt(digitalPinToInterrupt(encoderA), incrementFreqA, CHANGE);
  pinMode(encoderB, INPUT);
  //attachInterrupt(digitalPinToInterrupt(encoderB), incrementFreqB, CHANGE);
  pinMode(ina, OUTPUT);
  pinMode(inb, OUTPUT);
  pinMode(inc, OUTPUT);
  pinMode(ind, OUTPUT);

  delay(5000);

  i = 0;
  
  while ( i < 4 ) {
    resetFreqs();
    moveForward(27);
    delay(1000);
    resetFreqs();
    moveBackward(7);
    resetFreqs();
    delay(1000);
    rotateClockwise(90);
    delay(1000);
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

  motorAspeed = 130;
  motorBspeed = 140;

  resetMotors();
  
  //requiredSlots = floor(distance / 1.068); //number of slots required to travel = distance / distance per slot
  requiredSlots = floor(distance / 0.534);

  while (freqA < requiredSlots) {
    //this controls the forwards movement
    analogWrite(ina, motorAspeed); 
    digitalWrite(inb, LOW);
    analogWrite(inc, motorBspeed);
    digitalWrite(ind, LOW);

    if (digitalRead(encoderA) == HIGH && currentOutputA == 0) {
    freqA ++;
    //Serial.println(currentOutput); 
    currentOutputA = 1;
  } else if (digitalRead(encoderA) == LOW && currentOutputA == 1){
    freqA ++;
    currentOutputA = 0;
    //Serial.println("low"); 
  }
    
    /*if (millis() >= timeNow + period) { //not sure about this may not work. idea is to check gyro every 100ms
      timeNow += period;
      gyroCheck();
    }*/
  }
  resetMotors();
}

void moveBackward (int distance) {
  int requiredSlots;
  int period = 100;
  unsigned long timeNow;

  motorAspeed = 130;
  motorBspeed = 150;

  resetMotors();

  //requiredSlots = floor(distance / 1.068); //number of slots required to travel = distance / distance per slot
  requiredSlots = floor(distance / 0.534);
  
  while (freqA < requiredSlots) {
    //this controls the forwards movement
    analogWrite(inb, motorAspeed); 
    digitalWrite(ina, LOW);
    analogWrite(ind, motorBspeed);
    digitalWrite(inc, LOW);

    if (digitalRead(encoderA) == HIGH && currentOutputA == 0) {
    freqA ++;
    //Serial.println(currentOutput); 
    currentOutputA = 1;
  } else if (digitalRead(encoderA) == LOW && currentOutputA == 1){
    freqA ++;
    currentOutputA = 0;
    //Serial.println("low"); 
  }
    
    /*if (millis() >= timeNow + period) { //not sure about this may not work. idea is to check gyro every 100ms
      timeNow += period;
      gyroCheck();
    } */
  }
  resetMotors();
}

void rotateClockwise (int deg) {
   int ticks;
   motorAspeed = 130;
   motorBspeed = 140;

   //ticks = floor(deg / 9.81);
   ticks = floor(deg / 4.905);

   resetMotors();
   
   while (freqA < ticks - 4) {
      digitalWrite(ina, LOW);
      analogWrite(inb, motorAspeed);
      analogWrite(inc, motorBspeed);
      digitalWrite(ind, LOW);

      if (digitalRead(encoderA) == HIGH && currentOutputA == 0) {
        freqA ++;
        //Serial.println(currentOutput); 
        currentOutputA = 1;
      } else if (digitalRead(encoderA) == LOW && currentOutputA == 1){
        freqA ++;
        currentOutputA = 0;
        //Serial.println("low"); 
      }
   }

   resetMotors();
}

void rotateAntiClockwise (int deg) {
   int ticks = 9;
   motorAspeed = 110;
   motorBspeed = 120;

   resetMotors();

   while (freqA < ticks) {
      analogWrite(ina, motorAspeed);
      digitalWrite(inb, LOW);
      digitalWrite(inc, LOW);
      analogWrite(ind, motorBspeed);
   }

   resetMotors();
}

void gyroCheck () {
/*  if ( yaw  <= 7 && yaw >= -7 ) {
        analogWrite(ina, motorAspeed);
        digitalWrite(inb, LOW);
        digitalWrite(ind, LOW);
        analogWrite(inc, motorBspeed);
  }
  else {
    if ( yaw < 0 ) {
        analogWrite(ina, (motorAspeed + 15));
        digitalWrite(inb,LOW);
        digitalWrite(inc, LOW);
        analogWrite(ind, (motorBspeed + 15));
  }

  else {
        digitalWrite(ina, LOW);
        analogWrite(inb, (motorAspeed + 15));
        analogWrite(inc, (motorBspeed + 15));
        digitalWrite(ind, LOW);
  }
    
  }*/
}

void incrementFreqA() {
  freqA += 1;
}

void incrementFreqB() {
  freqB += 1;
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
