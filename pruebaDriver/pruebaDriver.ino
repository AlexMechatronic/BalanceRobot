#include "driverL293.h"

MotorHandle_s motorL = {.pin1 = 26/*IN1*/, .pin2=32/*IN2*/, .enable =10/*A*/};
MotorHandle_s motorR = {.pin1 = 28/*IN3*/, .pin2=30/*IN4*/, .enable =9/*B*/};

void setup()
{
  Serial.begin(9600);
  driverInit(&motorL);
  Serial.print("Pin1: ");
  Serial.println(motorL.pin1);
  Serial.print("pin2: ");
  Serial.println(motorL.pin2);
  Serial.print("Enable: ");
  Serial.println(motorL.enable);
  //delay(2000);
}

void loop()
{
  for(uint8_t i=76; i<255; i++)
  {
    driverMForward(&motorR,i);
    delay(1000);
    Serial.println(i);
  }
}

