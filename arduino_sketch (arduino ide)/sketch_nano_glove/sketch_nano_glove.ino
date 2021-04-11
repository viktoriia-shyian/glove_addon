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

  mpu1.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu1.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
}

void loop()
{
  if (Serial.available())
  {

    switch ()
    {
    //check connection
    case 0:
    {
      Serial.println(mpu1.testConnection() ? "MPU6050 1 OK" : "MPU6050 1 FAIL");
      Serial.println(mpu2.testConnection() ? "MPU6050 2 OK" : "MPU6050 2 FAIL");
      Serial.println(mpu3.testConnection() ? "MPU6050 3 OK" : "MPU6050 3 FAIL");
      Serial.println(mpu4.testConnection() ? "MPU6050 4 OK" : "MPU6050 4 FAIL");
      Serial.println(mpu5.testConnection() ? "MPU6050 5 OK" : "MPU6050 5 FAIL");
      Serial.println(mpu6.testConnection() ? "MPU6050 6 OK" : "MPU6050 6 FAIL");
    }
    //calibration
    case 1:
    {
      calibration();
    }
    //send data to call
    case 2:
    {
      send_data();
    }
    case 3:
    {
    }
    }

    //get data
    send_data();
    Serial.println();
    delay(1000);

    if (state == 0)
    {
      Serial.println("\nReading sensors for first time...");
      meansensors();
      state++;
      delay(1000);
    }
    if (state == 1)
    {
      Serial.println("\nCalculating offsets...");
      calibration();
      state++;
      delay(1000);
    }
    if (state == 2)
    {
      meansensors();
      Serial.println("\nFINISHED!");
      Serial.print("\nSensor readings with offsets:\t");
      Serial.print(mean_ax);
      Serial.print("\t");
      Serial.print(mean_ay);
      Serial.print("\t");
      Serial.print(mean_az);
      Serial.print("\t");
      Serial.print(mean_gx);
      Serial.print("\t");
      Serial.print(mean_gy);
      Serial.print("\t");
      Serial.println(mean_gz);
      Serial.print("Your offsets:\t");
      Serial.print(ax_offset);
      Serial.print(", ");
      Serial.print(ay_offset);
      Serial.print(", ");
      Serial.print(az_offset);
      Serial.print(", ");
      Serial.print(gx_offset);
      Serial.print(", ");
      Serial.print(gy_offset);
      Serial.print(", ");
      Serial.println(gz_offset);
      Serial.println("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
      Serial.println("Check that your sensor readings are close to 0 0 16384 0 0 0");
      Serial.println("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)");
      while (1)
        ;
    }
  }
}

void send_data()
{

  mpu1.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  send_data();

  mpu2.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  send_data();

  mpu3.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  send_data();

  mpu4.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  send_data();

  mpu5.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  send_data();

  mpu6.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  send_data();

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

void meansensors()
{
  long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;
  while (i < (buffersize + 101))
  { // read raw accel/gyro measurements from device mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); if (i > 100 && i <= (buffersize + 100)) { //First 100 measures are discarded
    buff_ax = buff_ax + ax;
    buff_ay = buff_ay + ay;
    buff_az = buff_az + az;
    buff_gx = buff_gx + gx;
    buff_gy = buff_gy + gy;
    buff_gz = buff_gz + gz;
  }
  if (i == (buffersize + 100))
  {
    mean_ax = buff_ax / buffersize;
    mean_ay = buff_ay / buffersize;
    mean_az = buff_az / buffersize;
    mean_gx = buff_gx / buffersize;
    mean_gy = buff_gy / buffersize;
    mean_gz = buff_gz / buffersize;
  }
  i++;
  delay(2);
}
}

void calibration()
{
  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (16384 - mean_az) / 8;
  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;
  while (1)
  {
    int ready = 0;
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);
    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);
    meansensors();
    Serial.println("...");
    if (abs(mean_ax) <= acel_deadzone)
      ready++;
    else
      ax_offset = ax_offset - mean_ax / acel_deadzone;
    if (abs(mean_ay) <= acel_deadzone)
      ready++;
    else
      ay_offset = ay_offset - mean_ay / acel_deadzone;
    if (abs(16384 - mean_az) <= acel_deadzone)
      ready++;
    else
      az_offset = az_offset + (16384 - mean_az) / acel_deadzone;
    if (abs(mean_gx) <= gyro_deadzone)
      ready++;
    else
      gx_offset = gx_offset - mean_gx / (gyro_deadzone + 1);
    if (abs(mean_gy) <= gyro_deadzone)
      ready++;
    else
      gy_offset = gy_offset - mean_gy / (gyro_deadzone + 1);
    if (abs(mean_gz) <= gyro_deadzone)
      ready++;
    else
      gz_offset = gz_offset - mean_gz / (gyro_deadzone + 1);
    if (ready == 6)
      break;
  }
}
