int ina = 5;
int inb = 6;
int inc = 9;
int ind = 11;
int motorAspeed = 110;
int motorBspeed = 110;
int freq = 0;
int currentOutput;

void setup() {
  // put your setup code here, to run once:
  pinMode(3, INPUT);
  pinMode(ina, OUTPUT);
  pinMode(inb, OUTPUT);
  pinMode(inc, OUTPUT);
  pinMode(ind, OUTPUT);
  Serial.begin(9600);
  Serial.println("Starting in 5");
  delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (freq >= 19) {
    digitalWrite(ina, LOW);
    digitalWrite(inb, LOW);
    digitalWrite(inc, LOW);
    digitalWrite(ind, LOW);
    Serial.println(freq);
    //delay(2000);
    //freq = 0;
    
  } else {
    analogWrite(ina, motorAspeed);
    digitalWrite(inb, LOW);
    analogWrite(ind, motorBspeed);
    digitalWrite(inc, LOW);
  }

  if (digitalRead(3) == HIGH && currentOutput == 0) {
    freq ++;
    //Serial.println(currentOutput); 
    currentOutput = 1;
  } else if (digitalRead(3) == LOW && currentOutput == 1){
    currentOutput = 0;
    //Serial.println("low"); 
  }

  
}
