#include "PIDWallFollower.h"

namespace Motion
{

PIDWallFollower::PIDWallFollower(Hardware::ToF &tof)
    : tof(tof),
      pid(1.8, 0.02, 0.5) // Steer centering PID constants
{
  pid.setSetpoint(0.0);
}

void PIDWallFollower::reset()
{
  pid.reset();
}

double PIDWallFollower::computeCorrection(bool &hasWall)
{
  uint16_t leftDist = tof.getDistance(Hardware::L);
  uint16_t rightDist = tof.getDistance(Hardware::R);

  bool leftWall = tof.isClear(Hardware::L, WALL_THRESHOLD_MM) == false;
  bool rightWall = tof.isClear(Hardware::R, WALL_THRESHOLD_MM) == false;

  float error = 0.0f;
  hasWall = false;

  if (leftWall && rightWall)
  {
    // Centering error (positive = closer to left wall, negative = closer to right wall)
    error = static_cast<float>(rightDist) - static_cast<float>(leftDist);
    hasWall = true;
  }
  else if (leftWall)
  {
    // Alignment to left wall only
    // If leftDist is 70mm and target is 55mm, error is 15mm. We need to steer right.
    error = static_cast<float>(leftDist) - targetSideDistance;
    hasWall = true;
  }
  else if (rightWall)
  {
    // Alignment to right wall only
    // If rightDist is 70mm and target is 55mm, error is -15mm. We need to steer left.
    error = targetSideDistance - static_cast<float>(rightDist);
    hasWall = true;
  }

  if (hasWall)
  {
    double correction = pid.compute(error);
    // Limit maximum wall following correction to avoid unstable oscillation
    return constrain(correction, -60.0, 60.0);
  }

  return 0.0;
}

} // namespace Motion
