#pragma once

#include "RobotConfig.h"
#include "SparkFun_BNO08x_Arduino_Library.h"

class IMU
{
public:
  IMU();
  bool begin();
  bool update();
  float getHeading() const;
  bool isReady() const;

private:
  BNO08x imu;
  float headingDegrees;
  bool initialized;
  static float quaternionToYaw(float i, float j, float k, float r);
  static float normalizeAngle(float degrees);
};
