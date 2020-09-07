#define Motor_PWM 10

void setup() {
  // put your setup code here, to run once:
  
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(Motor_PWM, HIGH);
  delay(1000);
  digitalWrite(Motor_PWM, LOW);
  delay(1000);
  
}
