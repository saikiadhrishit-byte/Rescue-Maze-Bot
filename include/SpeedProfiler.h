#pragma once

#include <Arduino.h>

namespace Motion
{

class SpeedProfiler
{
public:
  SpeedProfiler(float maxAccel = 300.0f, float maxDecel = 400.0f);
  void reset(float initialSpeed = 0.0f);
  
  // Updates current speed towards target speed given time delta (dt in seconds)
  float update(float targetSpeed, float dtSec);
  
  float getSpeed() const { return currentSpeed; }

private:
  float currentSpeed;
  float accelLimit;
  float decelLimit;
};

} // namespace Motion
