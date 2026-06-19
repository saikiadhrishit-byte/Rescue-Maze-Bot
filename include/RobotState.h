#pragma once

enum class RobotState
{
  Idle,
  Init,
  Explore,
  MoveForward,
  Turn,
  ScanTile,
  ReturnHome,
  Finished
};
