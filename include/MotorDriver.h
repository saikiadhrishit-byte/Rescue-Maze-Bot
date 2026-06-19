#pragma once

#include <Arduino.h>

class MotorDriver
{
public:
  MotorDriver(uint8_t pwmLeft, uint8_t dirLeft, uint8_t pwmRight, uint8_t dirRight, uint8_t standbyPin);
  void begin();
  void setPower(int leftPower, int rightPower);
  void stop();

private:
  uint8_t pwmLeft;
  uint8_t dirLeft;
  uint8_t pwmRight;
  uint8_t dirRight;
  uint8_t standbyPin;
};
