#pragma once

#include <Arduino.h>

class Logger
{
public:
  static void begin(unsigned long baud = 115200);
  static void info(const char *message);
  static void warn(const char *message);
  static void error(const char *message);
};
