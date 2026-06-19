#pragma once

#include <Arduino.h>

class PowerManager
{
public:
  PowerManager(uint8_t powerSensePin);
  void begin();
  float readBatteryVoltage() const;
  bool isBatteryLow() const;

private:
  uint8_t powerSensePin;
};
