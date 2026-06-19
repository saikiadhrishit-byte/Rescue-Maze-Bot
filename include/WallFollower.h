#pragma once

#include "RobotConfig.h"
#include "RangeFinder.h"
#include "DriveSystem.h"
#include <PID_v1.h>

class WallFollower
{
public:
  WallFollower(RangeFinder &rangeFinder, DriveSystem &drive)
      : rangeFinder(rangeFinder), drive(drive), pid(&input, &output, &setpoint, 1.5, 0.0, 0.4, DIRECT), lastCorrection(0)
  {
    pid.SetOutputLimits(-80, 80);
    pid.SetMode(AUTOMATIC);
  }

  void maintainCenter(int speed = 130)
  {
    rangeFinder.update();
    float left = static_cast<float>(rangeFinder.distance(SIDE_LEFT));
    float right = static_cast<float>(rangeFinder.distance(SIDE_RIGHT));
    if (!rangeFinder.isClear(SIDE_LEFT, WALL_THRESHOLD_MM) || !rangeFinder.isClear(SIDE_RIGHT, WALL_THRESHOLD_MM))
    {
      drive.setMotorPower(speed, speed);
      return;
    }

    float error = left - right;
    input = error;
    pid.Compute();
    int correction = static_cast<int>(output);
    drive.setMotorPower(speed - correction, speed + correction);
    lastCorrection = correction;
  }

private:
  RangeFinder &rangeFinder;
  DriveSystem &drive;
  double input = 0.0;
  double output = 0.0;
  PID pid{&input, &output, &setpoint, 1.5, 0.0, 0.4, DIRECT};
  double setpoint = 0.0;
  int lastCorrection;
};
