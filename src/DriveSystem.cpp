#include "DriveSystem.h"
#include <Arduino.h>

DriveSystem::DriveSystem(uint8_t leftA,
                         uint8_t leftB,
                         uint8_t rightA,
                         uint8_t rightB,
                         uint8_t leftPwm,
                         uint8_t leftDir,
                         uint8_t rightPwm,
                         uint8_t rightDir,
                         uint8_t standbyPin)
    : leftEncoder(leftA, leftB),
      rightEncoder(rightA, rightB),
      pinLeftPwm(leftPwm),
      pinLeftDir(leftDir),
      pinRightPwm(rightPwm),
      pinRightDir(rightDir),
      pinStandby(standbyPin),
      imu(nullptr),
      headingInput(0.0),
      headingOutput(0.0),
      headingSetpoint(0.0),
      headingPid(&headingInput, &headingOutput, &headingSetpoint, 2.2, 0.05, 0.8, P_ON_E, DIRECT)
{
}

bool DriveSystem::begin(IMU *imuPtr)
{
  imu = imuPtr;
  pinMode(pinLeftPwm, OUTPUT);
  pinMode(pinLeftDir, OUTPUT);
  pinMode(pinRightPwm, OUTPUT);
  pinMode(pinRightDir, OUTPUT);
  pinMode(pinStandby, OUTPUT);

  digitalWrite(pinStandby, HIGH);
  headingPid.SetMode(AUTOMATIC);
  headingPid.SetOutputLimits(-150, 150);
  return imu && imu->isReady();
}

void DriveSystem::stop()
{
  analogWrite(pinLeftPwm, 0);
  analogWrite(pinRightPwm, 0);
}

void DriveSystem::setMotorPower(int left, int right)
{
  left = constrainPower(left);
  right = constrainPower(right);

  digitalWrite(pinLeftDir, left >= 0 ? LOW : HIGH);
  analogWrite(pinLeftPwm, abs(left));
  digitalWrite(pinRightDir, right >= 0 ? LOW : HIGH);
  analogWrite(pinRightPwm, abs(right));
}

void DriveSystem::driveHeading(float targetHeading, int speed)
{
  if (!imu)
    return;

  float currentHeading = imu->getHeading();
  float error = wrapAngle(targetHeading - currentHeading);
  headingSetpoint = 0.0;
  headingInput = error;
  headingPid.Compute();
  int correction = static_cast<int>(headingOutput);
  setMotorPower(speed + correction, speed - correction);
}

void DriveSystem::moveForward(float distance_mm, int speed)
{
  if (!imu)
    return;

  resetEncoders();
  headingSetpoint = imu->getHeading();
  const int32_t targetTicks = static_cast<int32_t>(distance_mm / DISTANCE_PER_TICK_MM);
  const int basePower = constrainPower(speed);

  while (true)
  {
    imu->update();
    int32_t leftTicks = abs(leftEncoder.read());
    int32_t rightTicks = abs(rightEncoder.read());
    if (leftTicks >= targetTicks || rightTicks >= targetTicks)
      break;

    float currentHeading = imu->getHeading();
    float error = wrapAngle(headingSetpoint - currentHeading);
    headingInput = error;
    headingPid.Compute();
    int correction = static_cast<int>(headingOutput);
    int leftPower = constrainPower(basePower + correction);
    int rightPower = constrainPower(basePower - correction);
    setMotorPower(leftPower, rightPower);
    delay(10);
  }

  stop();
}

void DriveSystem::turnLeft90()
{
  if (!imu)
    return;
  float current = imu->getHeading();
  float target = wrapAngle(current - 90.0f);
  rotateToHeading(target);
}

void DriveSystem::turnRight90()
{
  if (!imu)
    return;
  float current = imu->getHeading();
  float target = wrapAngle(current + 90.0f);
  rotateToHeading(target);
}

void DriveSystem::rotateToHeading(float targetHeading)
{
  if (!imu)
    return;

  while (true)
  {
    imu->update();
    float current = imu->getHeading();
    float error = wrapAngle(targetHeading - current);
    if (abs(error) < 2.5f)
      break;
    headingSetpoint = 0.0;
    headingInput = error;
    headingPid.Compute();
    int output = static_cast<int>(headingOutput);
    output = constrainPower(output);
    setMotorPower(output, -output);
    delay(10);
  }
  stop();
}

void DriveSystem::update()
{
  if (imu)
  {
    imu->update();
  }
}

void DriveSystem::resetEncoders()
{
  leftEncoder.write(0);
  rightEncoder.write(0);
}

int32_t DriveSystem::leftEncoderCount()
{
  return leftEncoder.read();
}

int32_t DriveSystem::rightEncoderCount()
{
  return rightEncoder.read();
}

int DriveSystem::constrainPower(int value)
{
  return constrain(value, -255, 255);
}

float DriveSystem::wrapAngle(float degrees)
{
  while (degrees > 180.0f)
    degrees -= 360.0f;
  while (degrees < -180.0f)
    degrees += 360.0f;
  return degrees;
}
