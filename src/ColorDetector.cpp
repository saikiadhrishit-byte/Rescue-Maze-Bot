#include "ColorDetector.h"
#include "Buzzer.h"
#include "RobotConfig.h"

ColorDetector::ColorDetector()
    : colorSensor(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X),
      triggered(false),
      currentColor(TILE_UNKNOWN)
{
}

bool ColorDetector::begin()
{
  if (!colorSensor.begin())
    return false;

  colorSensor.setInterrupt(false);
  return true;
}

void ColorDetector::update()
{
  // TCRT sensors are active low. If either detects line, start color read.
  bool left = digitalRead(PIN_TCRT_LEFT) == LOW;
  bool right = digitalRead(PIN_TCRT_RIGHT) == LOW;

  if (!left && !right)
  {
    // no tile under robot
    triggered = false;
    return;
  }

  if (!triggered)
  {
    triggered = true;
    uint16_t r, g, b, c;
    colorSensor.getRawData(&r, &g, &b, &c);
    currentColor = classifyColor(r, g, b, c);
    // handle black/blue immediately
    if (currentColor == TILE_BLACK)
    {
      // treat black as wall: beep and set state so main avoids entering
      Buzzer::beep(300);
    }
    else if (currentColor == TILE_BLUE)
    {
      // flash blue LED and pause
      digitalWrite(PIN_BLUE_LED, HIGH);
      delay(5000);
      digitalWrite(PIN_BLUE_LED, LOW);
    }
  }
}

bool ColorDetector::tileTriggered() const
{
  return triggered;
}

TileColor ColorDetector::getTileColor() const
{
  return currentColor;
}

TileColor ColorDetector::classifyColor(uint16_t r, uint16_t g, uint16_t b, uint16_t c) const
{
  if (c < 500)
    return TILE_BLACK;

  if (b > r + 60 && b > g + 40)
    return TILE_BLUE;

  if (r > 200 && g > 200 && b > 200)
    return TILE_SILVER;

  return TILE_UNKNOWN;
}
