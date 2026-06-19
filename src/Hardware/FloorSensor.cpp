#include "FloorSensor.h"

namespace Hardware
{

FloorSensor::FloorSensor()
    : colorSensor(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X),
      colorSensorInitialized(false),
      rawLeft(0), rawRight(0),
      calWhiteLeft(600), calWhiteRight(600),
      blackThresholdLeft(300), blackThresholdRight(300),
      currentColor(TILE_UNKNOWN),
      readingInProgress(false)
{
}

bool FloorSensor::begin()
{
  pinMode(PIN_TCRT_LEFT, INPUT);
  pinMode(PIN_TCRT_RIGHT, INPUT);

  // Default calibration settings
  calibrate();

  // Initialize color sensor
  if (colorSensor.begin())
  {
    colorSensorInitialized = true;
    colorSensor.setInterrupt(false);
    colorReadTimer.start(100); // Poll color every 100ms
  }
  else
  {
    Serial.println("Warning: TCS34725 color sensor initialization failed!");
    colorSensorInitialized = false;
  }

  return true;
}

void FloorSensor::calibrate()
{
  // Read analog pins multiple times on startup to establish white level
  uint32_t sumLeft = 0;
  uint32_t sumRight = 0;
  constexpr int numSamples = 20;

  for (int i = 0; i < numSamples; ++i)
  {
    sumLeft += analogRead(PIN_TCRT_LEFT);
    sumRight += analogRead(PIN_TCRT_RIGHT);
    delay(10);
  }

  calWhiteLeft = sumLeft / numSamples;
  calWhiteRight = sumRight / numSamples;

  // Black tile threshold is set to 50% of the white tile reading
  blackThresholdLeft = calWhiteLeft / 2;
  blackThresholdRight = calWhiteRight / 2;

  Serial.print("Floor calibration: WhiteL=");
  Serial.print(calWhiteLeft);
  Serial.print(" WhiteR=");
  Serial.print(calWhiteRight);
  Serial.print(" BlackLThresh=");
  Serial.print(blackThresholdLeft);
  Serial.print(" BlackRThresh=");
  Serial.println(blackThresholdRight);
}

void FloorSensor::update()
{
  // Read analog values from TCRT5000s
  rawLeft = analogRead(PIN_TCRT_LEFT);
  rawRight = analogRead(PIN_TCRT_RIGHT);

  // 1. Black Tile Detection (Analog threshold logic)
  if (rawLeft < blackThresholdLeft || rawRight < blackThresholdRight)
  {
    currentColor = TILE_BLACK;
    return;
  }

  // 2. Silver Checkpoint Detection (Extremely reflective)
  if (rawLeft > TCRT_SILVER_THRESHOLD || rawRight > TCRT_SILVER_THRESHOLD)
  {
    currentColor = TILE_SILVER;
    return;
  }

  // 3. Blue Checkpoint / Color Sensor Logic
  if (colorSensorInitialized && colorReadTimer.isExpired())
  {
    colorReadTimer.reset();
    uint16_t r, g, b, c;
    colorSensor.getRawData(&r, &g, &b, &c);
    
    TileColor detectedColor = classifyColor(r, g, b, c);
    
    if (detectedColor == TILE_BLUE)
    {
      currentColor = TILE_BLUE;
    }
    else if (detectedColor == TILE_BLACK)
    {
      currentColor = TILE_BLACK;
    }
    else if (detectedColor == TILE_SILVER)
    {
      currentColor = TILE_SILVER;
    }
    else
    {
      currentColor = TILE_UNKNOWN; // White / normal floor
    }
  }
}

bool FloorSensor::isLineOrBlackDetected() const
{
  return (currentColor == TILE_BLACK);
}

bool FloorSensor::isBlackTile() const
{
  return (currentColor == TILE_BLACK);
}

bool FloorSensor::isSilverTile() const
{
  return (currentColor == TILE_SILVER);
}

bool FloorSensor::isBlueTile() const
{
  return (currentColor == TILE_BLUE);
}

TileColor FloorSensor::getTileColor() const
{
  return currentColor;
}

TileColor FloorSensor::classifyColor(uint16_t r, uint16_t g, uint16_t b, uint16_t c) const
{
  if (c < 200) // Low clear channel suggests black
    return TILE_BLACK;

  // Blue detection: Blue channel is higher than red and green by a margin
  if (b > r + 30 && b > g + 20)
    return TILE_BLUE;

  // Silver detection: Raw values are very high and balanced
  if (r > 600 && g > 600 && b > 600)
    return TILE_SILVER;

  return TILE_UNKNOWN;
}

} // namespace Hardware
