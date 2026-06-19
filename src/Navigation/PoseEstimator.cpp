#include "PoseEstimator.h"

namespace Navigation
{

PoseEstimator::PoseEstimator()
{
  reset();
}

void PoseEstimator::reset(Coordinate startCell, CompassDir startHeading)
{
  currentPose.cell = startCell;
  currentPose.heading = startHeading;
  
  // Place the robot in the center of the starting grid cell
  currentPose.xMm = startCell.x * CELL_SIZE_MM + (CELL_SIZE_MM / 2.0f);
  currentPose.yMm = startCell.y * CELL_SIZE_MM + (CELL_SIZE_MM / 2.0f);
  currentPose.yawDeg = headingToYaw(startHeading);
}

void PoseEstimator::update(int32_t currTicksLeft, int32_t currTicksRight, float currentYawDeg)
{
  // 1. Calculate difference in encoder ticks
  int32_t dLeft = currTicksLeft - lastTicksLeft;
  int32_t dRight = currTicksRight - lastTicksRight;

  lastTicksLeft = currTicksLeft;
  lastTicksRight = currTicksRight;

  // Always update continuous yaw and compass heading
  currentPose.yawDeg = currentYawDeg;
  currentPose.heading = yawToHeading(currentYawDeg);

  // If no change in encoder ticks, skip expensive trigonometric odometry calculation
  if (dLeft == 0 && dRight == 0)
  {
    return;
  }

  // 2. Average distance moved on this step
  float averageTicks = (dLeft + dRight) / 2.0f;
  float dDistanceMm = averageTicks * DISTANCE_PER_TICK_MM;

  // 3. Project movement onto global 2D coordinate system
  float yawRad = currentYawDeg * (PI / 180.0f);
  
  float dx = dDistanceMm * sin(yawRad);
  float dy = -dDistanceMm * cos(yawRad);

  currentPose.xMm += dx;
  currentPose.yMm += dy;

  // 4. Update discrete cell coordinate based on raw position
  currentPose.cell.x = static_cast<int8_t>(currentPose.xMm / CELL_SIZE_MM);
  currentPose.cell.y = static_cast<int8_t>(currentPose.yMm / CELL_SIZE_MM);
}

void PoseEstimator::setCell(Coordinate cell)
{
  currentPose.cell = cell;
  currentPose.xMm = cell.x * CELL_SIZE_MM + (CELL_SIZE_MM / 2.0f);
  currentPose.yMm = cell.y * CELL_SIZE_MM + (CELL_SIZE_MM / 2.0f);
}

void PoseEstimator::setHeading(CompassDir heading)
{
  currentPose.heading = heading;
  currentPose.yawDeg = headingToYaw(heading);
}

void PoseEstimator::forceYaw(float yawDeg)
{
  currentPose.yawDeg = yawDeg;
}

CompassDir PoseEstimator::yawToHeading(float yawDeg)
{
  // Normalize yaw to [0, 360)
  float angle = fmod(yawDeg, 360.0f);
  if (angle < 0.0f) angle += 360.0f;

  if (angle >= 315.0f || angle < 45.0f)   return DIR_NORTH;
  if (angle >= 45.0f  && angle < 135.0f)  return DIR_EAST;
  if (angle >= 135.0f && angle < 225.0f)  return DIR_SOUTH;
  if (angle >= 225.0f && angle < 315.0f)  return DIR_WEST;
  
  return DIR_NORTH;
}

float PoseEstimator::headingToYaw(CompassDir heading)
{
  switch (heading)
  {
    case DIR_NORTH: return 0.0f;
    case DIR_EAST:  return 90.0f;
    case DIR_SOUTH: return 180.0f;
    case DIR_WEST:  return 270.0f;
    default:        return 0.0f;
  }
}

} // namespace Navigation
