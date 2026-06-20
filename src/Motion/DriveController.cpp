#include "DriveController.h"

namespace Motion
{

DriveController::DriveController(Hardware::Motors &motors, Hardware::Encoders &encoders, IMU &imu)
    : motors(motors), encoders(encoders), imu(imu),
      motionState(MotionType::Idle),
      targetHeading(0.0f), targetLeftTicks(0), targetRightTicks(0),
      baseTargetSpeed(0), directionSign(1.0f), headingCorrection(0.0),
      headingPid(2.2, 0.05, 0.8), // Using the legacy tuned parameters
      speedProfiler(400.0f, 500.0f), // Accel/decel limits (ticks/s^2 equivalent)
      lastUpdateMs(0)
{
}

void DriveController::begin()
{
  headingPid.setSetpoint(0.0); // Setpoint is 0 error
  lastUpdateMs = millis();
}

void DriveController::startMoveForward(float distanceMm, int targetSpeed)
{
  encoders.reset();
  imu.update();
  
  targetHeading = imu.getHeading();
  targetLeftTicks = abs(static_cast<int32_t>(distanceMm / DISTANCE_PER_TICK_MM));
  targetRightTicks = targetLeftTicks;
  baseTargetSpeed = targetSpeed;
  directionSign = (distanceMm >= 0.0f) ? 1.0f : -1.0f;
  headingCorrection = 0.0;

  headingPid.reset();
  speedProfiler.reset(0.0f);
  
  motionState = MotionType::MovingForward;
  lastUpdateMs = millis();
}

void DriveController::startTurn(float relativeAngleDeg, int targetSpeed)
{
  encoders.reset();
  imu.update();

  targetHeading = wrapAngle(imu.getHeading() + relativeAngleDeg);
  targetLeftTicks = 0; // Turn completion is determined by heading
  targetRightTicks = 0;
  baseTargetSpeed = targetSpeed;
  directionSign = 1.0f;
  headingCorrection = 0.0;

  headingPid.reset();
  speedProfiler.reset(0.0f);

  motionState = MotionType::Turning;
  lastUpdateMs = millis();
}

void DriveController::startTurnToHeading(float absoluteHeadingDeg, int targetSpeed)
{
  encoders.reset();
  imu.update();

  targetHeading = wrapAngle(absoluteHeadingDeg);
  targetLeftTicks = 0;
  targetRightTicks = 0;
  baseTargetSpeed = targetSpeed;
  directionSign = 1.0f;
  headingCorrection = 0.0;

  headingPid.reset();
  speedProfiler.reset(0.0f);

  motionState = MotionType::Turning;
  lastUpdateMs = millis();
}

void DriveController::stopImmediate()
{
  motors.stop();
  motionState = MotionType::Idle;
}

void DriveController::startBrake()
{
  baseTargetSpeed = 0;
  motionState = MotionType::Braking;
  lastUpdateMs = millis();
}

void DriveController::update()
{
  if (motionState == MotionType::Idle)
  {
    motors.stop();
    return;
  }

  uint32_t now = millis();
  float dtSec = (now - lastUpdateMs) / 1000.0f;
  lastUpdateMs = now;

  imu.update();
  float currentHeading = imu.getHeading();
  float headingError = wrapAngle(targetHeading - currentHeading);

  if (motionState == MotionType::MovingForward)
  {
    int32_t leftTicks = abs(encoders.getLeftAverage());
    int32_t rightTicks = abs(encoders.getRightAverage());

    // Check if target distance is reached
    if (leftTicks >= targetLeftTicks || rightTicks >= targetRightTicks)
    {
      stopImmediate();
      return;
    }

    // Speed ramping (Trapezoidal Ramping Down)
    int32_t remainingLeft = targetLeftTicks - leftTicks;
    int32_t remainingRight = targetRightTicks - rightTicks;
    int32_t remainingTicks = min(remainingLeft, remainingRight);

    float currentTargetSpeed = baseTargetSpeed;
    if (remainingTicks < DECEL_START_TICKS && baseTargetSpeed > MIN_DRIVE_POWER)
    {
      float ratio = static_cast<float>(remainingTicks) / DECEL_START_TICKS;
      currentTargetSpeed = MIN_DRIVE_POWER + ratio * (baseTargetSpeed - MIN_DRIVE_POWER);
    }

    // Speed profiler ramps velocity to prevent slip
    float profiledSpeed = speedProfiler.update(currentTargetSpeed, dtSec);
    float speed = profiledSpeed * directionSign;

    // PID heading correction (setpoint = 0, input = -error) corrected for direction
    double correction = headingPid.compute(-headingError) * directionSign;

    // Apply external steering correction (e.g. wall following centering) only if moving forward
    if (directionSign > 0.0f)
    {
      correction += headingCorrection;
    }

    // Apply differential drive skid steer outputs
    int leftPower = static_cast<int>(speed + correction);
    int rightPower = static_cast<int>(speed - correction);

    motors.setDrivePower(leftPower, rightPower);
  }
  else if (motionState == MotionType::Turning)
  {
    // Check if we are close to the target heading
    if (abs(headingError) < 1.5f)
    {
      stopImmediate();
      return;
    }

    // PID correction for turning (spin-in-place)
    double correction = headingPid.compute(-headingError);

    // Limit correction to prevent violent spin
    correction = constrain(correction, -baseTargetSpeed, baseTargetSpeed);
    
    // Scale turn power using speed profiler
    float profiledSpeed = speedProfiler.update(abs(correction), dtSec);
    int turnPower = static_cast<int>(profiledSpeed);

    // Overcome static friction: ensure turnPower is at least MIN_TURN_POWER
    if (turnPower < MIN_TURN_POWER)
    {
      turnPower = MIN_TURN_POWER;
    }

    if (correction > 0)
    {
      // Turn right (left motor forward, right motor backward)
      motors.setDrivePower(turnPower, -turnPower);
    }
    else
    {
      // Turn left (left motor backward, right motor forward)
      motors.setDrivePower(-turnPower, turnPower);
    }
  }
  else if (motionState == MotionType::Braking)
  {
    float profiledSpeed = speedProfiler.update(0.0f, dtSec);
    if (profiledSpeed <= 5.0f)
    {
      stopImmediate();
    }
    else
    {
      // Slow down in current heading
      double correction = headingPid.compute(-headingError) * directionSign;
      float speed = profiledSpeed * directionSign;
      int leftPower = static_cast<int>(speed + correction);
      int rightPower = static_cast<int>(speed - correction);
      motors.setDrivePower(leftPower, rightPower);
    }
  }
}

float DriveController::wrapAngle(float angle)
{
  while (angle > 180.0f)  angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

} // namespace Motion
