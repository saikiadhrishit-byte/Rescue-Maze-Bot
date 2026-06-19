#pragma once

#include <Arduino.h>
#include "RobotConfig.h"
#include "GridMaze.h"

namespace Navigation
{

struct Pose
{
  Coordinate cell;     // Discrete (x, y) cell coordinate
  CompassDir heading;  // Heading (North, East, South, West)
  
  float xMm;           // High precision position in millimeters
  float yMm;
  float yawDeg;        // Yaw angle in degrees [0, 360)
};

class PoseEstimator
{
public:
  PoseEstimator();
  void reset(Coordinate startCell = {GRID_WIDTH / 2, GRID_HEIGHT / 2}, CompassDir startHeading = DIR_NORTH);
  
  void update(int32_t dTicksLeft, int32_t dTicksRight, float currentYawDeg);
  
  // Setters/Getters
  Pose getPose() const { return currentPose; }
  void setCell(Coordinate cell);
  void setHeading(CompassDir heading);
  void forceYaw(float yawDeg);

  // Helper translations
  static CompassDir yawToHeading(float yawDeg);
  static float headingToYaw(CompassDir heading);

private:
  Pose currentPose;
  int32_t lastTicksLeft;
  int32_t lastTicksRight;
};

} // namespace Navigation
