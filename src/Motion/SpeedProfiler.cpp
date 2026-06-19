#include "SpeedProfiler.h"

namespace Motion
{

SpeedProfiler::SpeedProfiler(float maxAccel, float maxDecel)
    : currentSpeed(0.0f), accelLimit(maxAccel), decelLimit(maxDecel)
{
}

void SpeedProfiler::reset(float initialSpeed)
{
  currentSpeed = initialSpeed;
}

float SpeedProfiler::update(float targetSpeed, float dtSec)
{
  if (dtSec <= 0.0f) return currentSpeed;

  float diff = targetSpeed - currentSpeed;
  if (abs(diff) < 0.1f)
  {
    currentSpeed = targetSpeed;
    return currentSpeed;
  }

  if (diff > 0.0f)
  {
    // Accelerating
    float step = accelLimit * dtSec;
    if (step >= diff)
      currentSpeed = targetSpeed;
    else
      currentSpeed += step;
  }
  else
  {
    // Decelerating (diff is negative)
    float step = decelLimit * dtSec;
    if (step >= abs(diff))
      currentSpeed = targetSpeed;
    else
      currentSpeed -= step;
  }

  return currentSpeed;
}

} // namespace Motion
