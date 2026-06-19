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
    const ToFMeasurements &measurements = rangeFinder.getMeasurements();
    bool leftValid = measurements.valid[SIDE_LEFT];
    bool rightValid = measurements.valid[SIDE_RIGHT];
    float left = leftValid ? static_cast<float>(measurements.distances[SIDE_LEFT]) : 0.0f;
    float right = rightValid ? static_cast<float>(measurements.distances[SIDE_RIGHT]) : 0.0f;
    bool leftWall = leftValid && left < WALL_THRESHOLD_MM;
    bool rightWall = rightValid && right < WALL_THRESHOLD_MM;

    if (!leftWall && !rightWall)
    {
      drive.setMotorPower(speed, speed);
      return;
    }

    float error = 0.0f;
    if (leftWall && rightWall)
    {
      error = right - left;
    }
    else if (leftWall)
    {
      error = left - WALL_THRESHOLD_MM;
    }
    else if (rightWall)
    {
      error = right - WALL_THRESHOLD_MM;
    }

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
