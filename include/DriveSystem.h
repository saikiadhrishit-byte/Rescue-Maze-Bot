#pragma once

#include "RobotConfig.h"
#include "IMU.h"
#include <Encoder.h>
#include <PID_v1.h>

class DriveSystem
{
public:
  DriveSystem(uint8_t leftA,
              uint8_t leftB,
              uint8_t rightA,
              uint8_t rightB,
              uint8_t leftPwm,
              uint8_t leftDir,
              uint8_t rightPwm,
              uint8_t rightDir,
              uint8_t standbyPin);

  bool begin(IMU *imu);
  void stop();
  void setMotorPower(int left, int right);
  void driveHeading(float targetHeading, int speed = 130);
  void moveForward(float distance_mm, int speed = 140);
  void turnLeft90();
  void turnRight90();
  void rotateToHeading(float targetHeading);
  void update();
  void resetEncoders();
  int32_t leftEncoderCount();
  int32_t rightEncoderCount();

private:
  Encoder leftEncoder;
  Encoder rightEncoder;
  uint8_t pinLeftPwm;
  uint8_t pinLeftDir;
  uint8_t pinRightPwm;
  uint8_t pinRightDir;
  uint8_t pinStandby;
  IMU *imu;

  double headingInput;
  double headingOutput;
  double headingSetpoint;
  PID headingPid;
  static int constrainPower(int value);
  static float wrapAngle(float degrees);
};
