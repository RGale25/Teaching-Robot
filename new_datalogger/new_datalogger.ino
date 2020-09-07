int encoderA = 2;
int encoderB = 3;
int ina = 5;
int inb = 6;
int inc = 9;
int ind = 11;

int freqA;
int freqB;

int mSpeed = 180;

unsigned long timePassed;
unsigned int interval = 1000;

int i;


void setup() {
  // put your setup code here, to run once:
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  
  pinMode(ina, OUTPUT);
  pinMode(inb, OUTPUT);
  pinMode(inc, OUTPUT);
  pinMode(ind, OUTPUT);

  i = 0;

  analogWrite(ina, mSpeed); 
  digitalWrite(inb, LOW);
  analogWrite(inc, mSpeed );
  digitalWrite(ind, LOW);

  Serial.begin(9600);
  delay(2000);
  attachInterrupt(digitalPinToInterrupt(encoderA), incrementFreqA, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderB), incrementFreqB, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  timePassed = millis();
  if (timePassed >= (i * interval)) {
    Serial.print("A Frequecy : ");
    Serial.print(freqA );
    Serial.print(  "      B Frequecy : ");
    Serial.println(freqB);
    resetFreqs();
    i ++;
  }

}

void incrementFreqA() {
  freqA += 1;
}

void incrementFreqB() {
  freqB += 1;
}

void resetFreqs() {
  freqA = 0;
  freqB = 0;
}
