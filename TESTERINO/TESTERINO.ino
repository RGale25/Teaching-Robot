int freq = 0;

int i = 0;
unsigned long timePassed;
unsigned int interval = 1000;

void setup() {
  // put your setup code here, to run once:
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(3), incrementFreq, RISING);

  pinMode(9, OUTPUT);
  pinMode(11, OUTPUT);

  analogWrite(9, 140);
  digitalWrite(11, LOW);

  Serial.begin(9600);
}
  
void loop() {
  // put your main code here, to run repeatedly:
  timePassed = millis();
  if (timePassed >= (i * interval)) {
    Serial.print("LFrequecy : ");
    Serial.println(freq);
    freq = 0;
    i ++;
  }

}

void incrementFreq() {
  freq += 1;
}
