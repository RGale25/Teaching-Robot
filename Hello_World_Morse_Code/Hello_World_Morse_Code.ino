void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  //H
  short_burst();
  short_burst();
  short_burst();
  short_burst();

  delay(2000);
  
  //E
  short_burst();

  delay(2000);
  
  //L
  short_burst();
  long_burst();
  short_burst();
  short_burst();

  delay(2000);

  //L
  short_burst();
  long_burst();
  short_burst();
  short_burst();

  delay(2000);

  //O
  long_burst();
  long_burst();
  long_burst();

  delay(2000);

  //W
  short_burst();
  long_burst();
  long_burst();

  delay(2000);
  
  //O
  long_burst();
  long_burst();
  long_burst();

  delay(2000);
  
  //R
  short_burst();
  long_burst();
  short_burst();

  delay(2000);
  
  //L
  short_burst();
  long_burst();
  short_burst();
  short_burst();

  delay(2000);
  
  //D
  long_burst();
  short_burst();
  short_burst();

  delay(2000);
}

void short_burst() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}

void long_burst() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(2000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(2000);
}
