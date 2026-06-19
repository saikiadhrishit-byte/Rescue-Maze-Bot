#include "Encoders.h"

namespace Hardware
{

Encoders::Encoders()
    : encFL(PIN_ENC_FL_A, PIN_ENC_FL_B),
      encFR(PIN_ENC_FR_A, PIN_ENC_FR_B),
      encRL(PIN_ENC_RL_A, PIN_ENC_RL_B),
      encRR(PIN_ENC_RR_A, PIN_ENC_RR_B),
      lastFL(0), lastFR(0), lastRL(0), lastRR(0),
      stallTimerMs(0)
{
}

void Encoders::reset()
{
  encFL.write(0);
  encFR.write(0);
  encRL.write(0);
  encRR.write(0);
  
  lastFL = 0;
  lastFR = 0;
  lastRL = 0;
  lastRR = 0;
  stallTimerMs = millis();
}

int32_t Encoders::getFL() const
{
  return encFL.read();
}

int32_t Encoders::getFR() const
{
  return encFR.read();
}

int32_t Encoders::getRL() const
{
  return encRL.read();
}

int32_t Encoders::getRR() const
{
  return encRR.read();
}

int32_t Encoders::getLeftAverage() const
{
  // Average of Front Left and Rear Left
  return (getFL() + getRL()) / 2;
}

int32_t Encoders::getRightAverage() const
{
  // Average of Front Right and Rear Right
  return (getFR() + getRR()) / 2;
}

bool Encoders::checkStall(int leftPower, int rightPower, uint32_t dtMs)
{
  int32_t currFL = getFL();
  int32_t currFR = getFR();
  int32_t currRL = getRL();
  int32_t currRR = getRR();

  bool flMoving = abs(currFL - lastFL) > 5;
  bool frMoving = abs(currFR - lastFR) > 5;
  bool rlMoving = abs(currRL - lastRL) > 5;
  bool rrMoving = abs(currRR - lastRR) > 5;

  bool flStall = (abs(leftPower) > 80) && !flMoving;
  bool frStall = (abs(rightPower) > 80) && !frMoving;
  bool rlStall = (abs(leftPower) > 80) && !rlMoving;
  bool rrStall = (abs(rightPower) > 80) && !rrMoving;

  lastFL = currFL;
  lastFR = currFR;
  lastRL = currRL;
  lastRR = currRR;

  if (flStall || frStall || rlStall || rrStall)
  {
    if (millis() - stallTimerMs > 400) // Stalled for 400ms
    {
      return true;
    }
  }
  else
  {
    stallTimerMs = millis();
  }

  return false;
}

bool Encoders::checkFailure() const
{
  int32_t fl = abs(getFL());
  int32_t fr = abs(getFR());
  int32_t rl = abs(getRL());
  int32_t rr = abs(getRR());

  // If one side has moved significantly but the other is completely zero, there is likely a hardware issue
  if ((fl > 200 && rl > 200 && fr == 0 && rr == 0) ||
      (fr > 200 && rr > 200 && fl == 0 && rl == 0))
  {
    return true;
  }
  return false;
}

} // namespace Hardware
