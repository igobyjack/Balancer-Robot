#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <math.h>

MPU6050 mpu;

//GLOBALS
int16_t ax, ay, az;
int16_t dx, dy, dz;
unsigned long currTime, prevTime = 0;
float pitch, dt, previousError, error, PID, Speed;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;

/*****************************/

// PID CONSTANTS

float kp = 20;
float ki = 0.1;
float kd = 0.5;

// TARGET ANGLE
float target = 1.9;

/*****************************/

// Motor control pins
int rightPin1 = 4;  // Right direction
int rightPin2 = 5;  // Right direction
int leftPin1 = 6;   // Left direction
int leftPin2 = 7;   // Left direction
int ERA = 9;        // Right motor speed
int ERB = 10;       // Left motor speed

void Back()
{

  //right motor
  digitalWrite(rightPin1, HIGH);
  digitalWrite(rightPin2, LOW);
  analogWrite(ERA, Speed);

  //left motor
  digitalWrite(leftPin1, LOW);
  digitalWrite(leftPin2, HIGH);
  analogWrite(ERB, Speed);


}

void Forward()
{
  
  //right motor
  digitalWrite(rightPin1, LOW);
  digitalWrite(rightPin2, HIGH);
  analogWrite(ERA, Speed);


  //left motor
  digitalWrite(leftPin1, HIGH);
  digitalWrite(leftPin2, LOW);
  analogWrite(ERB, Speed);

}

void Rest()
{
  //Right motor
  digitalWrite(rightPin1, LOW);
  digitalWrite(rightPin2, LOW);
  analogWrite(ERA, 0);

  //Left motor
  digitalWrite(leftPin1, LOW);
  digitalWrite(leftPin2, LOW);
  analogWrite(ERB, 0);
}

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  
  // MPU boot delay
  delay(1000);

  mpu.initialize();
  mpu.setXAccelOffset(-1602);
  mpu.setYAccelOffset(-445);
  mpu.setZAccelOffset(4643);
  mpu.setXGyroOffset(103);
  mpu.setYGyroOffset(-31);
  mpu.setZGyroOffset(-32);
}

void loop()
{

  currTime = millis();
  dt = (currTime - prevTime) / 1000.0; // conversion to seconds
  prevTime = currTime;
  
  // Read accelerometer data
  mpu.getAcceleration(&ax, &ay, &az);

  mpu.getRotation(&dx, &dy, &dz);

  // Print raw accelerometer values for debugging
  // Divide by 65.5 for deg/sec value
  // Serial.print("\tY ang: ");
  // Serial.print((float)dy/131);
  // Serial.print("\tX acc: ");
  // Serial.println((float)ax/16384);


  float accPitch = atan2((float)ax, (float)az) * RAD_TO_DEG;
  // 131 correction at sensitivity 0
  float gyroRate = (float)dy / 131.0;


  // Complementary filter 
  pitch = 0.995 * (pitch + gyroRate * dt) + 0.005 * accPitch;

  // total error
  error = target - pitch;

  // proportional error
  pid_p = kp * error;

  // integral error
  pid_i += (ki * error);
  pid_i = constrain(pid_i, -100, 100);

  //derivative error
  pid_d = kd * ((error - previousError) / dt);

  //final PID value
  PID = pid_p + pid_i + pid_d;

  Speed = constrain(abs(PID), 0, 255); 
  if (Speed > 0 && Speed < 50) Speed = 0; 

  // negative = back
  // positive = forward

  if (abs(error) < 1.0) {  
    Rest();
    pid_i = 0;
    Speed = 0;
  } else {
    if (PID < 0) Back();
    else Forward();
  }

  //Updated error
  previousError = error;

  // Debugging output
  // Serial.print("Pitch: "); Serial.print(pitch);
  // Serial.print("\tPID: "); Serial.print(PID);
  // Serial.print("\tSpeed: "); Serial.println(Speed);



}
