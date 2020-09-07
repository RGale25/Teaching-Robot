const int interruptPin1 = 2;
const int interruptPin2 = 3;

const int n1Pin = 5;
const int n2Pin = 6;
const int n3Pin = 9;
const int n4Pin = 11;

int freq1;
int freq2;

int i;

int motorPWM1; //sets the analog value to use for motor
int motorPWM2;

unsigned long timePassed;
unsigned int interval = 1000;

void setup() {
  // put your setup code here, to run once:
  pinMode(interruptPin1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin1), incrementFreq1, RISING);
  pinMode(interruptPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), incrementFreq2, RISING);

  pinMode(n1Pin, OUTPUT);
  pinMode(n2Pin, OUTPUT);
  pinMode(n3Pin, OUTPUT);
  pinMode(n4Pin, OUTPUT);

  motorPWM1 = 140;
  motorPWM2 = 155;

  analogWrite(n1Pin, motorPWM1);
  digitalWrite(n2Pin, LOW);
  digitalWrite(n4Pin, LOW);
  analogWrite(n3Pin, motorPWM2);
  
  i = 0;
  freq1 = 0;
  freq2 = 0;

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  timePassed = millis();
  if (timePassed >= (i * interval)) {
    Serial.print("LFrequecy : ");
    Serial.print(freq1);
    Serial.print(  "      RFrequecy : ");
    Serial.println(freq2);
    freq1 = 0;
    freq2 = 0;
    i ++;
  }
}

void incrementFreq1() {
  freq1 ++;
}
void incrementFreq2() {
  freq2 ++;
}
