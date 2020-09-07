int currentOutput;
int freq;
int i;
bool sent;
unsigned long timePassed;
unsigned long pulseStart;
unsigned long pulseStop;
unsigned long pulseWidth;
unsigned int interval = 1000; //1 seconds

void setup() {
  // put your setup code here, to run once
  pinMode(10, OUTPUT);
  pinMode(2, INPUT);
  freq = 0;
  i = 0;
  sent = false;
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(10,90);
  
  //should detect a change in output, if rising edge increment freq
  if (digitalRead(2) == HIGH && currentOutput == 0) {
    freq ++;
    currentOutput = 1;
    pulseStart = millis();
    
  } else if (digitalRead(2) == LOW && currentOutput == 1){
    freq ++;
    currentOutput = 0; 
    pulseStop = millis();
  }
  pulseWidth = pulseStop - pulseStart;
  timePassed = millis();
  
  if (timePassed >= (i * interval)) {
    Serial.println(pulseWidth); 
    freq = 0;
    i ++;
  }
  
}
