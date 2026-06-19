#pragma once

#include "RobotConfig.h"
#include "TileTypes.h"
#include <Adafruit_TCS34725.h>
#include <Arduino.h>

class ColorDetector
{
public:
  ColorDetector();
  bool begin();
  void update();
  bool tileTriggered() const;
  TileColor getTileColor() const;

private:
  Adafruit_TCS34725 colorSensor;
  bool triggered;
  bool triggeredEdge;
  TileColor currentColor;
  TileColor classifyColor(uint16_t r, uint16_t g, uint16_t b, uint16_t c) const;
};
