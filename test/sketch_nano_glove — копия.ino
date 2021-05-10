#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "MPU6050.h"


#include <SoftwareSerial.h>

float angleX = 0;
float angleY = 0;
float angleZ = 0;

MPU6050 mpu1(0x69);
MPU6050 mpu2(0x69);
MPU6050 mpu3(0x69);
MPU6050 mpu4(0x69);
MPU6050 mpu5(0x69);
MPU6050 mpu6(0x69);

int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup()
{
  pinMode(13, OUTPUT);

  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(38400);

  mpu1.initialize();
  mpu2.initialize();
  mpu3.initialize();
  mpu4.initialize();
  mpu5.initialize();
  mpu6.initialize();

  initDMP();
}

void loop()
{
  if (Serial.available())
  {

    switch (Serial.read())
    {
    // mpu connection state
    case 0:
    {
      Serial.println(mpu1.testConnection() ? "MPU6050 1 OK" : "MPU6050 1 FAIL");
      Serial.println(mpu2.testConnection() ? "MPU6050 2 OK" : "MPU6050 2 FAIL");
      Serial.println(mpu3.testConnection() ? "MPU6050 3 OK" : "MPU6050 3 FAIL");
      Serial.println(mpu4.testConnection() ? "MPU6050 4 OK" : "MPU6050 4 FAIL");
      Serial.println(mpu5.testConnection() ? "MPU6050 5 OK" : "MPU6050 5 FAIL");
      Serial.println(mpu6.testConnection() ? "MPU6050 6 OK" : "MPU6050 6 FAIL");
    }
    // sensitivity setting
    case 1:
    {
      set_sensitivity(MPU6050_ACCEL_FS_2, MPU6050_GYRO_FS_250);

      Serial.println("Set sensitivity OK");
    }
    //calibration
    case 2:
    {
      calibration(15);

      Serial.println("Calibration OK");
    }
    //send_offsets
    case 3:
    {
      send_offsets();
    }
    //send data to call
    case 4:
    {
      send_mpu_data();
    }
    case 5:
    {
      getAngles();

      Serial.print(angleX);
      Serial.print('\t');
      Serial.print(angleY);
      Serial.print('\t');
      Serial.println(angleZ);

      delay(20);

      /*
      send_data();

      Serial.println();
      delay(1000);
      */
    }
    }
  }
}

/*
void set_sensitivity()
{
  mpu1.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu1.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  mpu2.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu2.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  mpu3.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu3.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  mpu4.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu4.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  mpu5.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu5.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  mpu6.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu6.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
}
*/

void set_sensitivity(uint8_t accel, uint8_t gyro)
{
  mpu1.setFullScaleAccelRange(accel);
  mpu1.setFullScaleGyroRange(gyro);

  mpu2.setFullScaleAccelRange(accel);
  mpu2.setFullScaleGyroRange(gyro);

  mpu3.setFullScaleAccelRange(accel);
  mpu3.setFullScaleGyroRange(gyro);

  mpu4.setFullScaleAccelRange(accel);
  mpu4.setFullScaleGyroRange(gyro);

  mpu5.setFullScaleAccelRange(accel);
  mpu5.setFullScaleGyroRange(gyro);

  mpu6.setFullScaleAccelRange(accel);
  mpu6.setFullScaleGyroRange(gyro);
}

void calibration(int iterations) // up to 15
{
  mpu1.CalibrateAccel(iterations);
  mpu1.CalibrateGyro(iterations);

  mpu2.CalibrateAccel(iterations);
  mpu2.CalibrateGyro(iterations);

  mpu3.CalibrateAccel(iterations);
  mpu3.CalibrateGyro(iterations);

  mpu4.CalibrateAccel(iterations);
  mpu4.CalibrateGyro(iterations);

  mpu5.CalibrateAccel(iterations);
  mpu5.CalibrateGyro(iterations);

  mpu6.CalibrateAccel(iterations);
  mpu6.CalibrateGyro(iterations);
}

void send_offsets()
{
  ax = mpu1.getXAccelOffset();
  ay = mpu1.getYAccelOffset();
  az = mpu1.getZAccelOffset();
  gx = mpu1.getXGyroOffset();
  gy = mpu1.getYGyroOffset();
  gz = mpu1.getZGyroOffset();
  serial_send();

  ax = mpu2.getXAccelOffset();
  ay = mpu2.getYAccelOffset();
  az = mpu2.getZAccelOffset();
  gx = mpu2.getXGyroOffset();
  gy = mpu2.getYGyroOffset();
  gz = mpu2.getZGyroOffset();
  serial_send();

  ax = mpu3.getXAccelOffset();
  ay = mpu3.getYAccelOffset();
  az = mpu3.getZAccelOffset();
  gx = mpu3.getXGyroOffset();
  gy = mpu3.getYGyroOffset();
  gz = mpu3.getZGyroOffset();
  serial_send();

  ax = mpu4.getXAccelOffset();
  ay = mpu4.getYAccelOffset();
  az = mpu4.getZAccelOffset();
  gx = mpu4.getXGyroOffset();
  gy = mpu4.getYGyroOffset();
  gz = mpu4.getZGyroOffset();
  serial_send();

  ax = mpu5.getXAccelOffset();
  ay = mpu5.getYAccelOffset();
  az = mpu5.getZAccelOffset();
  gx = mpu5.getXGyroOffset();
  gy = mpu5.getYGyroOffset();
  gz = mpu5.getZGyroOffset();
  serial_send();

  ax = mpu6.getXAccelOffset();
  ay = mpu6.getYAccelOffset();
  az = mpu6.getZAccelOffset();
  gx = mpu6.getXGyroOffset();
  gy = mpu6.getYGyroOffset();
  gz = mpu6.getZGyroOffset();
  serial_send();
}

void send_mpu_data()
{
  mpu1.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  serial_send();

  mpu2.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  serial_send();

  mpu3.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  serial_send();

  mpu4.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  serial_send();

  mpu5.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  serial_send();

  mpu6.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  serial_send();
}

void serial_send()
{
  Serial.print(ax);
  Serial.print('\t');
  Serial.print(ay);
  Serial.print('\t');
  Serial.print(az);
  Serial.print('\t');
  Serial.print(gx);
  Serial.print('\t');
  Serial.print(gy);
  Serial.print('\t');
  Serial.println(gz);
}

// НУЖНЫЕ ПЕРЕМЕННЫЕ
const float toDeg = 360.0 / M_PI;
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
// инициализация
void initDMP()
{
  devStatus = mpu1.dmpInitialize();
  mpu1.setDMPEnabled(true);
  mpuIntStatus = mpu1.getIntStatus();
  packetSize = mpu1.dmpGetFIFOPacketSize();
}
// получение углов в angleX, angleY, angleZ
void getAngles()
{
  //if (mpu1.dmpGetCurrentFIFOPacket(fifoBuffer))
  //{
    mpu1.dmpGetQuaternion(&q, fifoBuffer);
    mpu1.dmpGetGravity(&gravity, &q);
    mpu1.dmpGetYawPitchRoll(ypr, &q, &gravity);
    angleX = ypr[2] * toDeg;
    angleY = ypr[1] * toDeg;
    angleZ = ypr[0] * toDeg;
  //}
}
