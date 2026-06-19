#pragma once

#include <Arduino.h>

class PIDController
{
public:
  PIDController(double kp, double ki, double kd);
  void setTunings(double kp, double ki, double kd);
  void setSetpoint(double setpoint);
  void reset();
  double compute(double measurement);

private:
  double kp;
  double ki;
  double kd;
  double setpoint;
  double integral;
  double lastError;
  uint32_t lastTime;
};
