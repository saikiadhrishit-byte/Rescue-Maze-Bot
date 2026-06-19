#pragma once

#include "RobotConfig.h"
#include <VL53L1X.h>

namespace Hardware
{

enum ToFPosition {
  FC = 0, // Front Center
  FL = 1, // Front Left
  FR = 2, // Front Right
  L  = 3, // Left
  R  = 4, // Right
  ToFCount = 5
};

struct RangeData
{
  uint16_t distances[ToFCount];
  bool valid[ToFCount];
  uint32_t lastUpdateMs[ToFCount];
};

class ToF
{
public:
  ToF();
  bool begin();
  void update();
  
  uint16_t getDistance(ToFPosition pos) const;
  bool isClear(ToFPosition pos, uint16_t threshold = JUNCTION_THRESHOLD_MM) const;
  bool checkSensorsTimeout(uint32_t timeoutMs = 500) const;

  static void selectTcaChannel(uint8_t channel);

private:
  VL53L1X sensors[ToFCount];
  RangeData readings;
  uint8_t tcaChannels[ToFCount];
};

} // namespace Hardware
