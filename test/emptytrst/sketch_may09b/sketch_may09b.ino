#include <SoftwareSerial.h>

void setup() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
  Serial.begin(38400);
}

void loop() {
  // put your main code here, to run repeatedly:

  if (Serial.available() > 0){
    

  String c = Serial.readString();
  Serial.println("you wrote"+c);
  }
}
