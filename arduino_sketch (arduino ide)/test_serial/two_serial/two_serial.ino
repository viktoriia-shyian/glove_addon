#include <SoftwareSerial.h>
#include <regex>

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  if (Serial.available() > 0)
  {
    String input = Serial.readString();

    if (input == "0")
    {
      Serial.println("input=0");
    }
    else if (input == "1" regex_match ("softwareTesting", regex("(soft)(.*)") ))
    {
      set_sensitivity(MPU6050_ACCEL_FS_2, MPU6050_GYRO_FS_250);
      Serial.println("Set sensitivity OK");
    }
    else if (input == "2")
    {
      calibration(15);
      Serial.println("Calibration OK");
    }
    else if (input == "3")
    {
      send_offsets();
    }
    else if (input == "4")
    {
      send_raw_data();
    }
    else if (input == "5")
    {
      timer();
    }
    else if (input == "6")
    {
      state = 1;
    }
  }
}
