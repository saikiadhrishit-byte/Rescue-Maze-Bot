#include "Motors.h"

namespace Hardware
{

Motors::Motors()
{
}

void Motors::begin()
{
  pinMode(PIN_MOTOR_FL_PWM, OUTPUT);
  pinMode(PIN_MOTOR_FL_DIR, OUTPUT);
  pinMode(PIN_MOTOR_FR_PWM, OUTPUT);
  pinMode(PIN_MOTOR_FR_DIR, OUTPUT);
  pinMode(PIN_MOTOR_RL_PWM, OUTPUT);
  pinMode(PIN_MOTOR_RL_DIR, OUTPUT);
  pinMode(PIN_MOTOR_RR_PWM, OUTPUT);
  pinMode(PIN_MOTOR_RR_DIR, OUTPUT);
  pinMode(PIN_MOTOR_STANDBY, OUTPUT);

  digitalWrite(PIN_MOTOR_STANDBY, HIGH);
  stop();
}

void Motors::setPower(int fl, int fr, int rl, int rr)
{
  driveMotor(PIN_MOTOR_FL_PWM, PIN_MOTOR_FL_DIR, fl);
  driveMotor(PIN_MOTOR_FR_PWM, PIN_MOTOR_FR_DIR, fr);
  driveMotor(PIN_MOTOR_RL_PWM, PIN_MOTOR_RL_DIR, rl);
  driveMotor(PIN_MOTOR_RR_PWM, PIN_MOTOR_RR_DIR, rr);
}

void Motors::setDrivePower(int left, int right)
{
  setPower(left, right, left, right);
}

void Motors::stop()
{
  analogWrite(PIN_MOTOR_FL_PWM, 0);
  analogWrite(PIN_MOTOR_FR_PWM, 0);
  analogWrite(PIN_MOTOR_RL_PWM, 0);
  analogWrite(PIN_MOTOR_RR_PWM, 0);
}

void Motors::brake()
{
  // DRV8871 brakes by pulling both inputs high, but since we use PWM/DIR:
  // Usually, standard H-bridge brakes when PWM is high and DIR is in a state that locks it,
  // or we write high to both. For simple non-blocking brake, we can write PWM 0 first, or stop.
  stop();
}

int Motors::constrainPower(int value)
{
  return constrain(value, -255, 255);
}

void Motors::driveMotor(uint8_t pwmPin, uint8_t dirPin, int power)
{
  power = constrainPower(power);
  if (power >= 0)
  {
    digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, power);
  }
  else
  {
    digitalWrite(dirPin, HIGH);
    analogWrite(pwmPin, abs(power));
  }
}

} // namespace Hardware
