#pragma once

#include <Arduino.h>

class Timer
{
public:
  Timer() : lastTime(0), intervalMs(0) {}
  
  void start(uint32_t interval)
  {
    intervalMs = interval;
    lastTime = millis();
  }

  bool isExpired() const
  {
    if (intervalMs == 0) return false;
    return (millis() - lastTime) >= intervalMs;
  }

  void reset()
  {
    lastTime = millis();
  }

  void stop()
  {
    intervalMs = 0;
  }

  uint32_t elapsed() const
  {
    return millis() - lastTime;
  }

private:
  uint32_t lastTime;
  uint32_t intervalMs;
};
