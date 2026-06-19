#include <Arduino.h>
#include "Robot.h"

// Global Robot Instance
Robot robot;

void setup()
{
  Serial.begin(115200);
  while (!Serial && millis() < 1000)
  {
    // Wait briefly for native USB serial port (e.g. Teensy)
  }
  
  robot.begin();
}

void loop()
{
  robot.update();
}
