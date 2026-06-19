#include "ToF.h"
#include <Wire.h>

namespace Hardware
{

ToF::ToF()
{
  tcaChannels[FC] = TCA_CH_FRONT_CENTER;
  tcaChannels[FL] = TCA_CH_FRONT_LEFT;
  tcaChannels[FR] = TCA_CH_FRONT_RIGHT;
  tcaChannels[L]  = TCA_CH_LEFT;
  tcaChannels[R]  = TCA_CH_RIGHT;

  for (int i = 0; i < ToFCount; ++i)
  {
    readings.distances[i] = MAX_SENSOR_DISTANCE_MM;
    readings.valid[i] = false;
    readings.lastUpdateMs[i] = 0;
  }
}

void ToF::selectTcaChannel(uint8_t channel)
{
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548A_ADDRESS);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

bool ToF::begin()
{
  Wire.begin();
  bool allSuccess = true;

  for (int i = 0; i < ToFCount; ++i)
  {
    selectTcaChannel(tcaChannels[i]);
    sensors[i].setTimeout(100); // Non-blocking 100ms timeout limit
    
    if (!sensors[i].init())
    {
      Serial.print("ToF sensor init failed on channel ");
      Serial.println(tcaChannels[i]);
      allSuccess = false;
      continue;
    }

    sensors[i].setDistanceMode(VL53L1X::Short);
    sensors[i].setMeasurementTimingBudget(20000); // 20ms timing budget
    sensors[i].startContinuous(20);
    
    readings.lastUpdateMs[i] = millis();
    readings.valid[i] = true;
  }

  return allSuccess;
}

void ToF::update()
{
  uint32_t now = millis();
  static uint32_t lastPollMs = 0;
  
  // VL53L1X measurement budget is 20ms, polling faster creates redundant I2C traffic
  if (now - lastPollMs < 20) return;
  lastPollMs = now;

  for (int i = 0; i < ToFCount; ++i)
  {
    selectTcaChannel(tcaChannels[i]);
    
    // VL53L1X dataReady check
    if (sensors[i].dataReady())
    {
      uint16_t dist = sensors[i].read(false); // read without blocking
      
      // VL53L1X returns 0 or large values on out-of-range or error
      if (dist > 0 && dist < MAX_SENSOR_DISTANCE_MM && !sensors[i].timeoutOccurred())
      {
        readings.distances[i] = dist;
        readings.valid[i] = true;
        readings.lastUpdateMs[i] = now;
      }
      else
      {
        readings.valid[i] = false;
      }
    }
    else
    {
      // If no reading for more than 150ms, mark as invalid (sensor might be blocked or glitched)
      if (now - readings.lastUpdateMs[i] > 150)
      {
        readings.valid[i] = false;
      }
    }
  }
}

uint16_t ToF::getDistance(ToFPosition pos) const
{
  return readings.distances[pos];
}

bool ToF::isClear(ToFPosition pos, uint16_t threshold) const
{
  // A channel is clear if we don't detect a wall within threshold
  return readings.valid[pos] && (readings.distances[pos] > threshold);
}

bool ToF::checkSensorsTimeout(uint32_t timeoutMs) const
{
  uint32_t now = millis();
  for (int i = 0; i < ToFCount; ++i)
  {
    if (now - readings.lastUpdateMs[i] > timeoutMs)
    {
      return true; // Sensor timeout detected
    }
  }
  return false;
}

} // namespace Hardware
