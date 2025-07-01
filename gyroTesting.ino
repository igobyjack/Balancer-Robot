#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <math.h>

MPU6050 mpu;

int16_t ax, ay, az;
int16_t dx, dy, dz;
unsigned long currTime, prevTime = 0;
float pitch, dt;

void setup()
{
  Wire.begin();
  Serial.begin(115200);

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
  // Dont use anymore!! will crash
  // Divide by 65.5 for deg/sec value
  // Serial.print("\tY ang: ");
  // Serial.print((float)dy/131);
  // Serial.print("\tX acc: ");
  // Serial.println((float)ax/16384);

  // no correction needed here since were operating with all units in raw values
  float accPitch = atan2((float)ax, (float)az) * RAD_TO_DEG;
  // 131 correction at sensitivity 0 
  // used for PID so need to correct
  float gyroRate = (float)dy / 131.0;

  // Complementary filter
  // higher alpha gives more trust to gyro
  // lower alpha gives more trust to accelerometer
  // alpha = 0.98 is a good value
  // pitch = alpha * (pitch + gyroRate * dt) + (1- alpha) * accPitch
  pitch = 0.98 * (pitch + gyroRate * dt) + 0.02 * accPitch;


  Serial.println(pitch);

}
