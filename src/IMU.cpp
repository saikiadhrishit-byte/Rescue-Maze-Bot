#include "IMU.h"
#include <Wire.h>

IMU::IMU()
    : headingDegrees(0.0f), initialized(false)
{
}

bool IMU::begin()
{
  if (!imu.begin(BNO08x_DEFAULT_ADDRESS, Wire, PIN_BNO085_INT, -1))
  {
    return false;
  }

  if (!imu.enableRotationVector(10))
  {
    return false;
  }

  initialized = true;
  return true;
}

bool IMU::update()
{
  if (!initialized)
    return false;

  if (!imu.serviceBus())
    return false;

  if (!imu.getSensorEvent())
    return false;

  float qi, qj, qk, qr, radAccuracy;
  uint8_t accuracy;
  imu.getQuat(qi, qj, qk, qr, radAccuracy, accuracy);
  headingDegrees = normalizeAngle(quaternionToYaw(qi, qj, qk, qr));
  return true;
}

float IMU::getHeading() const
{
  return headingDegrees;
}

bool IMU::isReady() const
{
  return initialized;
}

float IMU::quaternionToYaw(float i, float j, float k, float r)
{
  // Standard conversion from quaternion (x, y, z, w) to yaw angle
  float siny = 2.0f * (r * k + i * j);
  float cosy = 1.0f - 2.0f * (j * j + k * k);
  return atan2f(siny, cosy) * 180.0f / PI;
}

float IMU::normalizeAngle(float degrees)
{
  float value = fmod(degrees, 360.0f);
  if (value < 0)
    value += 360.0f;
  return value;
}
