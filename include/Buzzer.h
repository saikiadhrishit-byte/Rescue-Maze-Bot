#pragma once
#include <Arduino.h>
#include "RobotConfig.h"

class Buzzer {
public:
  static void begin() { pinMode(PIN_BUZZER, OUTPUT); digitalWrite(PIN_BUZZER, LOW); }
  static void beep(unsigned ms = 150) { tone(PIN_BUZZER, 2000); delay(ms); noTone(PIN_BUZZER); }
  static void on() { tone(PIN_BUZZER, 2000); }
  static void off() { noTone(PIN_BUZZER); }
};
