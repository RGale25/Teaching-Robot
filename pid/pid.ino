#include <Wire.h>
#include <FaBo9Axis_MPU9250.h>
#include <PID_v1.h>

float pitch = 0;
float roll = 0;
float yaw = 0;

float gx, gy, gz;

FaBo9Axis IMU;

double Setpoint = 0, Input , Output;

double Kp= 1, Ki=0, Kd=0;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPID2(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);

int e;
int p = 1;



int i;
unsigned long timePassed;
int interval = 1000;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  IMU.begin();
  myPID.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
}

void loop() {
  // put your main code here, to run repeatedly:
  timePassed = millis();
  if (timePassed >= (i * interval)) {
    
    IMU.readGyroXYZ(&gx,&gy,&gz);
    yaw = yaw + (gz) * 0.1;

    Input = yaw;
    /*
    e = Setpoint - Input;
    Output = p * e;*/
    Serial.println(yaw);
    Serial.print("Set Point = "); Serial.print(Setpoint); Serial.print("  Input = "); Serial.print(Input); Serial.print("  Output = "); Serial.println(Output);
    myPID.Compute();
    myPID2.Compute();
    i ++;
  }

}
