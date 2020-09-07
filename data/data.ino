
int currentOutputA;
int currentOutputB;
int freqA;
int freqB;
int i;
unsigned long timePassed;
unsigned int interval = 1000; //1 seconds

int m = 255;

void setup() {
  // put your setup code here, to run once
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  freqA = 0;
  freqB = 0;
  i = 0;
  analogWrite(5,110);
  digitalWrite(6, LOW);
  analogWrite(9,120);
  digitalWrite(11, LOW);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  
  //should detect a change in output, if rising edge increment freq
  if (digitalRead(2) == HIGH && currentOutputA == 0) {
    freqA ++;
    //Serial.println(currentOutput); 
    currentOutputA = 1;
  } else if (digitalRead(2) == LOW && currentOutputA == 1){
    currentOutputA = 0;
    //Serial.println("low"); 
  }

  if (digitalRead(3) == HIGH && currentOutputB == 0) {
    freqB ++;
    //Serial.println(currentOutput); 
    currentOutputB = 1;
  } else if (digitalRead(3) == LOW && currentOutputB == 1){
    currentOutputB = 0;
    //Serial.println("low"); 
  }
  
  timePassed = millis();
  if (timePassed >= (i * interval)) {
    Serial.print("Frequency A : ");
    Serial.print(freqA);
    Serial.print("       Frequency B : ");
    Serial.println(freqB);
    freqA = 0;
    freqB = 0;
    i ++;
  }
  
}
