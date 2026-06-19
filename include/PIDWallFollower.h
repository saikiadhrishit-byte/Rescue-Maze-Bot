#pragma once

#include <Arduino.h>
#include "ToF.h"
#include "PID.h"

namespace Motion
{

class PIDWallFollower
{
public:
  PIDWallFollower(Hardware::ToF &tof);
  void reset();
  
  // Computes steering correction based on wall distances. 
  // Set hasWall to true if wall centering is active.
  double computeCorrection(bool &hasWall);

private:
  Hardware::ToF &tof;
  PIDController pid;
  
  // Target distance from a single wall (corridor is ~280mm wide, robot width is 170mm, clearance is ~55mm per side)
  // Let's target ~55mm distance from the wall for perfect centering.
  const float targetSideDistance = 55.0f; 
};

} // namespace Motion
