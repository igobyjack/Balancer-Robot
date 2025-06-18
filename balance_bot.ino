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
  // Divide by 65.5 for deg/sec value
  // Serial.print("\tY ang: ");
  // Serial.print((float)dy/131);
  // Serial.print("\tX acc: ");
  // Serial.println((float)ax/16384);


  float accPitch = atan2((float)ax, (float)az) * RAD_TO_DEG;
  // 131 correction at sensitivity 0
  float gyroRate = (float)dy / 131.0;


  // Complementary filter
  pitch = 0.98 * (pitch + gyroRate * dt) + 0.02 * accPitch;

  Serial.println(pitch);

}
