#include <SoftwareSerial.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define interruptPin 2
#define LED_PIN 13

MPU6050 mpu1(0x69);
MPU6050 mpu2(0x69);
MPU6050 mpu3(0x69);
MPU6050 mpu4(0x69);
MPU6050 mpu5(0x69);
MPU6050 mpu6(0x69);

int16_t ax, ay, az;
int16_t gx, gy, gz;

// indicates whether MPU interrupt pin has gone high
volatile bool mpuInterrupt = false;

// MPU control/status vars
// holds actual interrupt status byte from MPU
uint8_t mpuIntStatus1;
uint8_t mpuIntStatus2;
uint8_t mpuIntStatus3;
uint8_t mpuIntStatus4;
uint8_t mpuIntStatus5;
uint8_t mpuIntStatus6;

// return status after each device operation (0 = success, !0 = error)
uint8_t devStatus;

// expected DMP packet size (default is 42 bytes)
uint16_t packetSize1;
uint16_t packetSize2;
uint16_t packetSize3;
uint16_t packetSize4;
uint16_t packetSize5;
uint16_t packetSize6;

uint16_t fifoCount1;
uint16_t fifoCount2;
uint16_t fifoCount3;
uint16_t fifoCount4;
uint16_t fifoCount5;
uint16_t fifoCount6;

uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float Yaw, Pitch, Roll;

static unsigned long _ETimer = 0;
int state = 0;
int timer = 10;

void setup()
{
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(2000000);

  Serial.println("i2cSetup");
  i2cSetup();
  Serial.println("mpu6050Connect");
  mpu6050Connect();
  Serial.println("Setup complete");
  pinMode(LED_PIN, OUTPUT);
}

void loop()
{
  if (Serial.available() > 0)
  {
    int input = Serial.parseInt();

    switch (input)
    {
    case 0:
      Serial.println(mpu1.testConnection() ? "MPU6050 1 OK" : "MPU6050 1 FAIL");
      Serial.println(mpu2.testConnection() ? "MPU6050 2 OK" : "MPU6050 2 FAIL");
      Serial.println(mpu3.testConnection() ? "MPU6050 3 OK" : "MPU6050 3 FAIL");
      Serial.println(mpu4.testConnection() ? "MPU6050 4 OK" : "MPU6050 4 FAIL");
      Serial.println(mpu5.testConnection() ? "MPU6050 5 OK" : "MPU6050 5 FAIL");
      Serial.println(mpu6.testConnection() ? "MPU6050 6 OK" : "MPU6050 6 FAIL");
      break;
    case 1:
      while (!Serial.available() > 0)
      {
      }
      int input_accel = Serial.parseInt();
      uint8_t accel;
      bool is_allowable = 1;

      while (!Serial.available() > 0)
      {
      }
      int input_gyro = Serial.parseInt();
      uint8_t gyro;

      switch (input_accel)
      {
      case 2:
        accel = MPU6050_ACCEL_FS_2;
        break;
      case 4:
        accel = MPU6050_ACCEL_FS_4;
        break;
      case 8:
        accel = MPU6050_ACCEL_FS_8;
        break;
      case 16:
        accel = MPU6050_ACCEL_FS_16;
        break;
      default:
        is_allowable = false;
        break;
      }

      switch (input_gyro)
      {
      case 250:
        accel = MPU6050_GYRO_FS_250;
        break;
      case 500:
        accel = MPU6050_GYRO_FS_500;
        break;
      case 1000:
        accel = MPU6050_GYRO_FS_1000;
        break;
      case 2000:
        accel = MPU6050_GYRO_FS_2000;
        break;
      default:
        is_allowable = false;
        break;
      }

      if (is_allowable)
      {
        setSensitivity(accel, gyro);
        Serial.println("Set sensitivity OK");
      }
      else
      {
        Serial.println("Set sensitivity FAIL");
      }
      break;
    case 2:
      while (!Serial.available() > 0)
      {
      }
      int input_iterations = Serial.parseInt();
      if (0 < input_iterations && input_iterations <= 15)
      {
        calibrate(input_iterations);
        Serial.println("Calibrate OK");
      }
      else
      {
        Serial.println("Calibrate FAIL");
      }
      break;
    case 3:
      sendOffsets();
      break;
    case 4:
      sendRawData();
      break;
    case 5:
      while (!getDMP())
      {
        delay(timer);
      }
      break;
    case 6:
      state = 1;
      Serial.println("Start timer OK");
      break;
    case 7:
      state = 0;
      Serial.println("Stop timer OK");
      break;
    }

    if (state)
    {
      if (millis() - _ETimer >= (timer))
      {
        _ETimer += (timer);
        mpuInterrupt = true;
      }
      if (mpuInterrupt)
      {
        // wait for MPU interrupt or extra packet(s) available
        getDMP();
      }
    }
  }
}

void i2cSetup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
}

void dmpDataReady()
{
  mpuInterrupt = true;
}

void mpu6050Connect()
{
  // initialize device
  mpu1.initialize();
  mpu2.initialize();
  mpu3.initialize();
  mpu4.initialize();
  mpu5.initialize();
  mpu6.initialize();

  // load and configure the DMP
  devStatus = mpu1.dmpInitialize();
  checkDevStatus(1);
  devStatus = mpu2.dmpInitialize();
  checkDevStatus(2);
  devStatus = mpu3.dmpInitialize();
  checkDevStatus(3);
  devStatus = mpu4.dmpInitialize();
  checkDevStatus(4);
  devStatus = mpu5.dmpInitialize();
  checkDevStatus(5);
  devStatus = mpu6.dmpInitialize();
  checkDevStatus(6);

  calibrate(15);

  Serial.println(F("Enabling DMP..."));
  mpu1.setDMPEnabled(true);
  mpu2.setDMPEnabled(true);
  mpu3.setDMPEnabled(true);
  mpu4.setDMPEnabled(true);
  mpu5.setDMPEnabled(true);
  mpu6.setDMPEnabled(true);

  // enable Arduino interrupt detection
  Serial.println(F("Enabling interrupt detection (Arduino external interrupt pin 2 on the Uno)..."));
  Serial.print("mpu1.getInterruptDrive=  ");
  Serial.println(mpu1.getInterruptDrive());
  Serial.print("mpu2.getInterruptDrive=  ");
  Serial.println(mpu2.getInterruptDrive());
  Serial.print("mpu3.getInterruptDrive=  ");
  Serial.println(mpu3.getInterruptDrive());
  Serial.print("mpu4.getInterruptDrive=  ");
  Serial.println(mpu4.getInterruptDrive());
  Serial.print("mpu5.getInterruptDrive=  ");
  Serial.println(mpu5.getInterruptDrive());
  Serial.print("mpu6.getInterruptDrive=  ");
  Serial.println(mpu6.getInterruptDrive());
  attachInterrupt(digitalPinToInterrupt(interruptPin), dmpDataReady, RISING);

  mpuIntStatus1 = mpu1.getIntStatus();
  mpuIntStatus2 = mpu2.getIntStatus();
  mpuIntStatus3 = mpu3.getIntStatus();
  mpuIntStatus4 = mpu4.getIntStatus();
  mpuIntStatus5 = mpu5.getIntStatus();
  mpuIntStatus6 = mpu6.getIntStatus();

  // get expected DMP packet size for later comparison
  packetSize1 = mpu1.dmpGetFIFOPacketSize();
  packetSize2 = mpu2.dmpGetFIFOPacketSize();
  packetSize3 = mpu3.dmpGetFIFOPacketSize();
  packetSize4 = mpu4.dmpGetFIFOPacketSize();
  packetSize5 = mpu5.dmpGetFIFOPacketSize();
  packetSize6 = mpu6.dmpGetFIFOPacketSize();
  delay(1000); // let it stabalize

  // clear fifo buffer
  mpu1.resetFIFO();
  mpu2.resetFIFO();
  mpu3.resetFIFO();
  mpu4.resetFIFO();
  mpu5.resetFIFO();
  mpu6.resetFIFO();

  mpu1.getIntStatus();
  mpu2.getIntStatus();
  mpu3.getIntStatus();
  mpu4.getIntStatus();
  mpu5.getIntStatus();
  mpu6.getIntStatus();

  // wait for next interrupt
  mpuInterrupt = false;
}

void checkDevStatus(int mpu_num)
{
  static int MPUInitCntr = 0;

  if (devStatus != 0)
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)

    char *StatStr[5]{"No Error", "initial memory load failed", "DMP configuration updates failed", "3", "4"};

    MPUInitCntr++;

    Serial.print(F("MPU connection Try #"));
    Serial.println(MPUInitCntr);
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(StatStr[devStatus]);
    Serial.println(F(")"));

    if (MPUInitCntr >= 10)
      return;
    delay(1000);
    mpu6050Connect();
    return;
  }
}

void setSensitivity(uint8_t accel, uint8_t gyro)
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

void calibrate(int iterations) // up to 15
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

void sendOffsets()
{
  ax = mpu1.getXAccelOffset();
  ay = mpu1.getYAccelOffset();
  az = mpu1.getZAccelOffset();
  gx = mpu1.getXGyroOffset();
  gy = mpu1.getYGyroOffset();
  gz = mpu1.getZGyroOffset();
  serialSendRawData();

  ax = mpu2.getXAccelOffset();
  ay = mpu2.getYAccelOffset();
  az = mpu2.getZAccelOffset();
  gx = mpu2.getXGyroOffset();
  gy = mpu2.getYGyroOffset();
  gz = mpu2.getZGyroOffset();
  serialSendRawData();

  ax = mpu3.getXAccelOffset();
  ay = mpu3.getYAccelOffset();
  az = mpu3.getZAccelOffset();
  gx = mpu3.getXGyroOffset();
  gy = mpu3.getYGyroOffset();
  gz = mpu3.getZGyroOffset();
  serialSendRawData();

  ax = mpu4.getXAccelOffset();
  ay = mpu4.getYAccelOffset();
  az = mpu4.getZAccelOffset();
  gx = mpu4.getXGyroOffset();
  gy = mpu4.getYGyroOffset();
  gz = mpu4.getZGyroOffset();
  serialSendRawData();

  ax = mpu5.getXAccelOffset();
  ay = mpu5.getYAccelOffset();
  az = mpu5.getZAccelOffset();
  gx = mpu5.getXGyroOffset();
  gy = mpu5.getYGyroOffset();
  gz = mpu5.getZGyroOffset();
  serialSendRawData();

  ax = mpu6.getXAccelOffset();
  ay = mpu6.getYAccelOffset();
  az = mpu6.getZAccelOffset();
  gx = mpu6.getXGyroOffset();
  gy = mpu6.getYGyroOffset();
  gz = mpu6.getZGyroOffset();
  serialSendRawData();
}

void sendRawData()
{
  mpu1.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  serialSendRawData();

  mpu2.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  serialSendRawData();

  mpu3.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  serialSendRawData();

  mpu4.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  serialSendRawData();

  mpu5.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  serialSendRawData();

  mpu6.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  serialSendRawData();
}

void serialSendRawData()
{
  Serial.print(ax);
  Serial.print(';');
  Serial.print(ay);
  Serial.print(';');
  Serial.print(az);
  Serial.print(';');
  Serial.print(gx);
  Serial.print(';');
  Serial.print(gy);
  Serial.print(';');
  Serial.println(gz);
}

bool getDMP()
{
  mpuInterrupt = false;

  // count of all bytes currently in FIFO
  fifoCount1 = mpu1.getFIFOCount();
  fifoCount2 = mpu2.getFIFOCount();
  fifoCount3 = mpu3.getFIFOCount();
  fifoCount4 = mpu4.getFIFOCount();
  fifoCount5 = mpu5.getFIFOCount();
  fifoCount6 = mpu6.getFIFOCount();

  if ((!fifoCount1) || (fifoCount1 % packetSize1) || (!fifoCount2) || (fifoCount2 % packetSize2) || (!fifoCount3) || (fifoCount3 % packetSize3) ||
      (!fifoCount4) || (fifoCount4 % packetSize4) || (!fifoCount5) || (fifoCount5 % packetSize5) || (!fifoCount6) || (fifoCount6 % packetSize6))
  {                             // reset failed
    digitalWrite(LED_PIN, LOW); // turn off the blinking light
    mpu1.resetFIFO();           // clear the buffer and start over
    mpu2.resetFIFO();
    mpu3.resetFIFO();
    mpu4.resetFIFO();
    mpu5.resetFIFO();
    mpu6.resetFIFO();
  }
  else
  {
    // get the packets until have the latest
    while (fifoCount1 >= packetSize1)
    {
      mpu1.getFIFOBytes(fifoBuffer, packetSize1);
      fifoCount1 -= packetSize1;
    }
    mpuMath(1);

    while (fifoCount2 >= packetSize2)
    {
      mpu2.getFIFOBytes(fifoBuffer, packetSize2);
      fifoCount2 -= packetSize2;
    }
    mpuMath(2);

    while (fifoCount3 >= packetSize3)
    {
      mpu3.getFIFOBytes(fifoBuffer, packetSize3);
      fifoCount3 -= packetSize3;
    }
    mpuMath(3);

    while (fifoCount4 >= packetSize4)
    {
      mpu4.getFIFOBytes(fifoBuffer, packetSize4);
      fifoCount4 -= packetSize4;
    }
    mpuMath(4);

    while (fifoCount5 >= packetSize5)
    {
      mpu5.getFIFOBytes(fifoBuffer, packetSize5);
      fifoCount5 -= packetSize5;
    }
    mpuMath(5);

    while (fifoCount6 >= packetSize6)
    {
      mpu6.getFIFOBytes(fifoBuffer, packetSize6);
      fifoCount6 -= packetSize6;
    }
    mpuMath(6);
    return true;
  }
  return false;
}

void mpuMath(int num)
{
  if (num == 1)
  {
    mpu1.dmpGetQuaternion(&q, fifoBuffer);
    mpu1.dmpGetGravity(&gravity, &q);
    mpu1.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
  else if (num == 2)
  {
    mpu2.dmpGetQuaternion(&q, fifoBuffer);
    mpu2.dmpGetGravity(&gravity, &q);
    mpu2.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
  else if (num == 3)
  {
    mpu3.dmpGetQuaternion(&q, fifoBuffer);
    mpu3.dmpGetGravity(&gravity, &q);
    mpu3.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
  else if (num == 4)
  {
    mpu4.dmpGetQuaternion(&q, fifoBuffer);
    mpu4.dmpGetGravity(&gravity, &q);
    mpu4.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
  else if (num == 5)
  {
    mpu5.dmpGetQuaternion(&q, fifoBuffer);
    mpu5.dmpGetGravity(&gravity, &q);
    mpu5.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
  else if (num == 6)
  {
    mpu6.dmpGetQuaternion(&q, fifoBuffer);
    mpu6.dmpGetGravity(&gravity, &q);
    mpu6.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }

  Yaw = (ypr[2] * 180.0 / M_PI);
  Pitch = (ypr[1] * 180.0 / M_PI);
  Roll = (ypr[0] * 180.0 / M_PI);

  Serial.println(String(Yaw) + ";" + String(Pitch) + ";" + String(Roll));
}
