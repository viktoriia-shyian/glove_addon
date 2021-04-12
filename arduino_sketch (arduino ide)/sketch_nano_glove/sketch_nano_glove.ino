#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#include <SoftwareSerial.h>

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
  Serial.begin(38400);

  mpu1.initialize();
  mpu2.initialize();
  mpu3.initialize();
  mpu4.initialize();
  mpu5.initialize();
  mpu6.initialize();
}

void loop()
{
  if (Serial.available())
  {

    switch ()
    {
    // connection state
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
      set_sensitivity();
      set_sensitivity(MPU6050_ACCEL_FS_2, MPU6050_GYRO_FS_250);
    }
    //calibration
    case 2:
    {
      calibration();
    }
    //send data to call
    case 3:
    {
      send_data();
    }
    case 4:
    {
    }
    default:
    {
      send_data();
      Serial.println();
      delay(1000);
    }
    }
  }
}

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

void set_sensitivity(uint8_t accel, uint8_t gyro)
{
  mpu1.setFullScaleAccelRange(accel);
  mpu1.setFullScaleGyroRange(gyro);
}

void send_data()
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

void calibration()
{
}
