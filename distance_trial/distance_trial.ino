int encoderA = 2;
int encoderB = 3;
int ina = 5;
int inb = 6;
int inc = 9;
int ind = 11;

int motorAspeed;
int motorBspeed;

int freqA;
int freqB;

void setup() {
  // put your setup code here, to run once:
  pinMode(encoderA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderA), incrementFreqA, CHANGE);
  pinMode(encoderB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderB), incrementFreqB, CHANGE);
  pinMode(ina, OUTPUT);
  pinMode(inb, OUTPUT);
  pinMode(inc, OUTPUT);
  pinMode(ind, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

}

void moveForward (int distance) {
  int requiredSlots;

  motorAspeed = 140;
  motorBspeed = 155;

  resetMotors();

  requiredSlots = floor(distance / 0.97); //number of slots required to travel = distance / distance per slot

  while (freqA < requiredSlots) {
    //this controls the forwards movement
  }
}

void moveBackward (int distance) {
  int requiredSlots;

  motorAspeed = 140;
  motorBspeed = 155;

  resetMotors();

  requiredSlots = floor(distance / 0.97); //number of slots required to travel = distance / distance per slot

  while (freqA < requiredSlots) {
    //this controls the forwards movement
  }
}

void rotateAntiClockwise (int deg) {
   int ticks = 38;
   motorAspeed = 120;
   motorBspeed = 135;

   resetMotors();
   
   while (freqA < ticks) {
      digitalWrite(ina, LOW);
      analogWrite(inb, motorAspeed);
      analogWrite(inc, motorBspeed);
      digitalWrite(ind, LOW);
   }
}

void rotateClockwise (int deg) {
   int ticks = 38;
   motorAspeed = 120;
   motorBspeed = 135;

   resetMotors();

   while (freqA < ticks) {
      analogWrite(ina, motorAspeed);
      digitalWrite(inb, LOW);
      digitalWrite(inc, LOW);
      analogWrite(ind, motorBspeed);
   }
}

void gyroCheck () {
  if ( yaw  <= 7 && yaw >= -7 ) {
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
    
  }
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
