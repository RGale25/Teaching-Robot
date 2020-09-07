#include <Wire.h>
#include <FaBo9Axis_MPU9250.h>

FaBo9Axis IMU;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;

int ina = 5;
int inb = 6;
int inc = 9;
int ind = 11;

int motorAspeed = 130;
int motorBspeed = 130;

void setup() 
{
  Serial.begin(115200);
  Serial.println("RESET");
  Serial.println();

  Serial.println("configuring device.");

  if (IMU.begin()) {
    Serial.println("configured FaBo 9Axis I2C Brick");
  } else {
    Serial.println("device error");
    while(1);
  }
  
  pinMode(ina, OUTPUT);
  pinMode(inb, OUTPUT);
  pinMode(inc, OUTPUT);
  pinMode(ind, OUTPUT);

  delay(5000); // use the delay to place barney on the floor
  

  
}

void loop()
{
  timer = millis();
  float gx,gy,gz;
  
  IMU.readGyroXYZ(&gx,&gy,&gz);

  // Calculate Pitch, Roll and Yaw
  pitch = pitch + (gy) * timeStep;
  roll = roll + (gx) * timeStep;
  yaw = yaw + (gz) * timeStep;

  // Output raw
  Serial.print(" Pitch = ");
  Serial.print(pitch);
  Serial.print(" Roll = ");
  Serial.print(roll);  
  Serial.print(" Yaw = ");
  Serial.println(yaw);

  // Wait to full timeStep period
  delay((timeStep*1000) - (millis() - timer));

  if ( yaw  <= 7 && yaw >= -7 ) {
        analogWrite(ina, 0);
        digitalWrite(inb, LOW);
        digitalWrite(ind, LOW);
        analogWrite(inc, 0);
  }
  else {
    if ( yaw < 0 ) {
        analogWrite(ina, (motorAspeed + 15));
        digitalWrite(inb,LOW);
        digitalWrite(inc, LOW);
        analogWrite(ind, (motorBspeed + 15));
  }

  else {
        digitalWrite(ina, LOW);
        analogWrite(inb, (motorAspeed + 15));
        analogWrite(inc, (motorBspeed + 15));
        digitalWrite(ind, LOW);
  }
    
  }
}
