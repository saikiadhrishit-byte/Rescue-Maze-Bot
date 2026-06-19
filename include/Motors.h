#pragma once

#include <Arduino.h>
#include "RobotConfig.h"

namespace Hardware
{

class Motors
{
public:
  Motors();
  void begin();
  void setPower(int fl, int fr, int rl, int rr);
  void setDrivePower(int left, int right); // Skid steer interface
  void stop();
  void brake();

private:
  static int constrainPower(int value);
  void driveMotor(uint8_t pwmPin, uint8_t dirPin, int power);
};

} // namespace Hardware
