#pragma once

#include <Arduino.h>
#include "RobotConfig.h"
#include "Timer.h"

namespace Hardware
{

class BuzzerLED
{
public:
  BuzzerLED();
  void begin();
  void update();

  // Non-blocking Buzzer actions
  void beep(uint32_t durationMs = 150, uint16_t frequencyHz = 2000);
  void playAlertPattern();
  void stopBuzzer();

  // LED actions
  void setGreen(bool on);
  void setRed(bool on);
  void setBlue(bool on);
  
  void startBlinkGreen(uint32_t intervalMs);
  void startBlinkRed(uint32_t intervalMs);
  void stopBlink();

private:
  Timer buzzerTimer;
  Timer blinkTimer;

  bool isBuzzerActive;
  bool isBlinking;
  uint32_t blinkInterval;
  bool blinkState;
  
  uint8_t activeBlinkLed; // PIN_LED_GREEN or PIN_LED_RED
};

} // namespace Hardware
