#include "BuzzerLED.h"

namespace Hardware
{

BuzzerLED::BuzzerLED()
    : isBuzzerActive(false),
      isBlinking(false),
      blinkInterval(0),
      blinkState(false),
      activeBlinkLed(0)
{
}

void BuzzerLED::begin()
{
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BLUE_LED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);

  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_BLUE_LED, LOW);
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_RED, LOW);
}

void BuzzerLED::update()
{
  // 1. Non-blocking buzzer turn-off
  if (isBuzzerActive && buzzerTimer.isExpired())
  {
    stopBuzzer();
  }

  // 2. Non-blocking LED blinking
  if (isBlinking && blinkTimer.isExpired())
  {
    blinkTimer.reset();
    blinkState = !blinkState;
    digitalWrite(activeBlinkLed, blinkState ? HIGH : LOW);
  }
}

void BuzzerLED::beep(uint32_t durationMs, uint16_t frequencyHz)
{
  tone(PIN_BUZZER, frequencyHz);
  isBuzzerActive = true;
  buzzerTimer.start(durationMs);
}

void BuzzerLED::playAlertPattern()
{
  // A quick double-beep indicating warning or critical event
  beep(100, 2500);
  // We can let update handle turning it off, and we could queue a second beep.
  // For simplicity, a single high-pitch beep is used here.
  beep(250, 3000);
}

void BuzzerLED::stopBuzzer()
{
  noTone(PIN_BUZZER);
  digitalWrite(PIN_BUZZER, LOW);
  isBuzzerActive = false;
  buzzerTimer.stop();
}

void BuzzerLED::setGreen(bool on)
{
  digitalWrite(PIN_LED_GREEN, on ? HIGH : LOW);
}

void BuzzerLED::setRed(bool on)
{
  digitalWrite(PIN_LED_RED, on ? HIGH : LOW);
}

void BuzzerLED::setBlue(bool on)
{
  digitalWrite(PIN_BLUE_LED, on ? HIGH : LOW);
}

void BuzzerLED::startBlinkGreen(uint32_t intervalMs)
{
  isBlinking = true;
  blinkInterval = intervalMs;
  activeBlinkLed = PIN_LED_GREEN;
  blinkState = true;
  digitalWrite(PIN_LED_GREEN, HIGH);
  digitalWrite(PIN_LED_RED, LOW); // Turn off Red during Green blinking
  blinkTimer.start(intervalMs);
}

void BuzzerLED::startBlinkRed(uint32_t intervalMs)
{
  isBlinking = true;
  blinkInterval = intervalMs;
  activeBlinkLed = PIN_LED_RED;
  blinkState = true;
  digitalWrite(PIN_LED_RED, HIGH);
  digitalWrite(PIN_LED_GREEN, LOW); // Turn off Green during Red blinking
  blinkTimer.start(intervalMs);
}

void BuzzerLED::stopBlink()
{
  isBlinking = false;
  blinkTimer.stop();
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_RED, LOW);
}

} // namespace Hardware
