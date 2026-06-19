#include "PID.h"

PIDController::PIDController(double kp, double ki, double kd)
    : kp(kp), ki(ki), kd(kd), setpoint(0.0), integral(0.0), lastError(0.0), lastTime(0)
{
}

void PIDController::setTunings(double p, double i, double d)
{
  kp = p;
  ki = i;
  kd = d;
}

void PIDController::setSetpoint(double sp)
{
  setpoint = sp;
}

void PIDController::reset()
{
  integral = 0.0;
  lastError = 0.0;
  lastTime = millis();
}

double PIDController::compute(double measurement)
{
  uint32_t now = millis();
  double dt = (now - lastTime) / 1000.0;
  if (dt <= 0.0) dt = 0.001; // Prevent division by zero
  lastTime = now;

  double error = setpoint - measurement;
  
  // Clamping integral to avoid windup
  integral += error * dt;
  integral = constrain(integral, -100.0, 100.0);

  double derivative = (error - lastError) / dt;
  lastError = error;

  return (kp * error) + (ki * integral) + (kd * derivative);
}
