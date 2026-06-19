#pragma once

#include <Arduino.h>
#include <Adafruit_TCS34725.h>
#include "RobotConfig.h"
#include "TileTypes.h"
#include "Timer.h"

namespace Hardware
{

class FloorSensor
{
public:
  FloorSensor();
  bool begin();
  void update();
  void calibrate();

  bool isLineOrBlackDetected() const;
  bool isBlackTile() const;
  bool isSilverTile() const;
  bool isBlueTile() const;
  TileColor getTileColor() const;

  // Raw readings (useful for debugging)
  uint16_t getLeftRaw() const { return rawLeft; }
  uint16_t getRightRaw() const { return rawRight; }

private:
  Adafruit_TCS34725 colorSensor;
  bool colorSensorInitialized;

  uint16_t rawLeft;
  uint16_t rawRight;

  uint16_t calWhiteLeft;
  uint16_t calWhiteRight;
  uint16_t blackThresholdLeft;
  uint16_t blackThresholdRight;

  TileColor currentColor;
  Timer colorReadTimer;
  bool readingInProgress;

  TileColor classifyColor(uint16_t r, uint16_t g, uint16_t b, uint16_t c) const;
};

} // namespace Hardware
