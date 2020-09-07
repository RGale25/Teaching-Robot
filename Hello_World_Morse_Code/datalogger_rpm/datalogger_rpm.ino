
int currentOutput;
int freq;
int i;
unsigned long timePassed;
unsigned int interval = 1000; //1 seconds

void setup() {
  // put your setup code here, to run once
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(2, INPUT);
  freq = 0;
  i = 0;
  analogWrite(5,100);
  digitalWrite(6, LOW);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  
  //should detect a change in output, if rising edge increment freq
  if (digitalRead(2) == HIGH && currentOutput == 0) {
    freq ++;
    //Serial.println(currentOutput); 
    currentOutput = 1;
  } else if (digitalRead(2) == LOW && currentOutput == 1){
    currentOutput = 0;
    //Serial.println("low"); 
  }
  
  timePassed = millis();
  if (timePassed >= (i * interval)) {
    Serial.println(freq);
    freq = 0;
    i ++;
  }
  
}
