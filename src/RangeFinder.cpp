#include "RangeFinder.h"
#include <Wire.h>

RangeFinder::RangeFinder()
{
  for (uint8_t i = 0; i < ToFCount; ++i)
  {
    readings.distances[i] = MAX_SENSOR_DISTANCE_MM;
    readings.valid[i] = false;
  }
}

bool RangeFinder::begin()
{
  Wire.begin();

  for (uint8_t i = 0; i < ToFCount; ++i)
  {
    pinMode(PIN_VL53_RESET_PINS[i], OUTPUT);
    digitalWrite(PIN_VL53_RESET_PINS[i], LOW);
  }

  delay(50);

  for (uint8_t i = 0; i < ToFCount; ++i)
  {
    activateSensor(i);
    sensors[i].setAddress(VL53_ADDRESS[i]);
    if (!sensors[i].init())
      return false;
    sensors[i].setDistanceMode(VL53L1X::Short);
    sensors[i].setMeasurementTimingBudget(20000);
    sensors[i].startContinuous(20);
  }

  return true;
}

void RangeFinder::activateSensor(uint8_t index)
{
  for (uint8_t i = 0; i < ToFCount; ++i)
  {
    digitalWrite(PIN_VL53_RESET_PINS[i], i == index ? HIGH : LOW);
  }
  delay(20);
}

void RangeFinder::update()
{
  for (uint8_t i = 0; i < ToFCount; ++i)
  {
    activateSensor(i);
    if (sensors[i].dataReady())
    {
      uint16_t range = sensors[i].read(true);
      readings.distances[i] = range;
      readings.valid[i] = (range > 0 && range < MAX_SENSOR_DISTANCE_MM);
    }
    else
    {
      readings.valid[i] = false;
    }
  }
}

uint16_t RangeFinder::distance(ToFPosition position) const
{
  return readings.distances[position];
}

bool RangeFinder::isClear(ToFPosition position, uint16_t threshold) const
{
  return readings.valid[position] && readings.distances[position] > threshold;
}

const ToFMeasurements &RangeFinder::getMeasurements() const
{
  return readings;
}
