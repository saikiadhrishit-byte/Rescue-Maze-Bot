#pragma once

#include <Arduino.h>

class ObstacleManager
{
public:
  ObstacleManager();
  void begin();
  bool isObstacleAhead();
  bool isObstacleLeft();
  bool isObstacleRight();
};
