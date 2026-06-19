#pragma once

#include "RobotConfig.h"
#include <VL53L1X.h>

enum ToFPosition {
  FRONT_LEFT = 0,
  FRONT_CENTER = 1,
  FRONT_RIGHT = 2,
  SIDE_LEFT = 3,
  SIDE_RIGHT = 4,
  ToFCount = 5
};

struct ToFMeasurements {
  uint16_t distances[ToFCount];
  bool valid[ToFCount];
};

class RangeFinder
{
public:
  RangeFinder();
  bool begin();
  void update();
  uint16_t distance(ToFPosition position) const;
  bool isClear(ToFPosition position, uint16_t threshold = JUNCTION_THRESHOLD_MM) const;
  const ToFMeasurements &getMeasurements() const;

private:
  VL53L1X sensors[ToFCount];
  ToFMeasurements readings;
  void activateSensor(uint8_t index);
};
