#pragma once

#include <Arduino.h>
#include <Encoder.h>
#include "RobotConfig.h"

namespace Hardware
{

class Encoders
{
public:
  Encoders();
  void reset();
  
  int32_t getFL() const;
  int32_t getFR() const;
  int32_t getRL() const;
  int32_t getRR() const;

  int32_t getLeftAverage() const;
  int32_t getRightAverage() const;

  // Safety functions
  bool checkStall(int leftPower, int rightPower, uint32_t dtMs);
  bool checkFailure() const;

private:
  mutable Encoder encFL;
  mutable Encoder encFR;
  mutable Encoder encRL;
  mutable Encoder encRR;

  // Storing history for stall detection
  int32_t lastFL;
  int32_t lastFR;
  int32_t lastRL;
  int32_t lastRR;
  uint32_t stallTimerMs;
};

} // namespace Hardware
