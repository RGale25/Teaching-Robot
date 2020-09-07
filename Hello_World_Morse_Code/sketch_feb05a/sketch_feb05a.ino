
int currentOutput;
int freq;
unsigned long timePassed;
unsigned int interval = 5000; //5 seconds

void setup() {
  // put your setup code here, to run once
  pinMode(10, OUTPUT);
  pinMode(2, INPUT);
  freq = 0;
  
  Serial.begin(9600);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(10,0);

  //should detect a change in output, if rising edge increment freq
  if (digitalRead(2) == HIGH && currentOutput == 0) {
    freq ++;
    currentOutput = 1;
  } else if (digitalRead(2) == LOW && currentOutput == 1){
    currentOutput = 0;
  }
  
  timePassed = millis();
  if (timePassed >= interval) {
    Serial.println(freq); 
  }
  
}
