#pragma once

#include <Arduino.h>
#include "Motors.h"
#include "Encoders.h"
#include "IMU.h"
#include "PID.h"
#include "SpeedProfiler.h"

namespace Motion
{

enum class MotionType
{
  Idle,
  MovingForward,
  Turning,
  Braking
};

class DriveController
{
public:
  DriveController(Hardware::Motors &motors, Hardware::Encoders &encoders, IMU &imu);
  void begin();
  void update();

  // Non-blocking motion triggers
  void startMoveForward(float distanceMm, int targetSpeed = 130);
  void startTurn(float relativeAngleDeg, int targetSpeed = 100);
  void startTurnToHeading(float absoluteHeadingDeg, int targetSpeed = 100);
  void stopImmediate();
  void startBrake();

  void setHeadingCorrection(double corr) { headingCorrection = corr; }

  static float wrapAngle(float angle);

  bool isMotionComplete() const { return motionState == MotionType::Idle; }
  MotionType getMotionState() const { return motionState; }

private:
  Hardware::Motors &motors;
  Hardware::Encoders &encoders;
  IMU &imu;

  MotionType motionState;
  
  float targetHeading;
  int32_t targetLeftTicks;
  int32_t targetRightTicks;
  int baseTargetSpeed;
  double headingCorrection; // Dynamic external steering offset

  PIDController headingPid;
  SpeedProfiler speedProfiler;
  
  uint32_t lastUpdateMs;
};

} // namespace Motion
