unsigned long pulseWidth;
unsigned long newPulse;
unsigned long timePassed;
int freq;
unsigned int interval = 1000;
int i;

int MotorOut = 10;
int EncoderIn = 6;
int PulseOut = 9;

void setup() {
  pinMode(MotorOut, OUTPUT); //set pin modes
  pinMode(EncoderIn, INPUT);
  pinMode(PulseOut, OUTPUT);
  
  analogWrite(MotorOut, 50); //sends pwm to motor (pin 10)
  
  Serial.begin(9600); //begin serial communication
  
  freq = 0; //initialize counter
  i = 1;
}

void loop() {
  pulseWidth  = pulseIn(10, HIGH); //returns time of pulse High
  sendPulse();
  freq ++;
  
  timePassed = millis();
  if (timePassed >= (i * interval)) {
    Serial.println(freq); 
    freq = 0;
    i ++;
  }
}

void sendPulse() {
  newPulse = pulseWidth * 5; //divide pulse high time by 5
  digitalWrite(PulseOut, HIGH);
  delayMicroseconds(newPulse);
  digitalWrite(PulseOut, LOW);
  delayMicroseconds(newPulse);
}
