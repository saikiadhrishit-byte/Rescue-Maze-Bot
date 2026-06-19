#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846
#endif

// I2C (Teensy 4.1)
constexpr uint8_t PIN_I2C_SDA = 18; // SDA (Wire)
constexpr uint8_t PIN_I2C_SCL = 19; // SCL (Wire)
constexpr uint8_t TCA9548A_ADDRESS = 0x70;

// TCA9548A channel assignments (0-7)
constexpr uint8_t TCA_CH_FRONT_CENTER = 0; // VL53L1X front center
constexpr uint8_t TCA_CH_FRONT_LEFT   = 1; // VL53L1X front left
constexpr uint8_t TCA_CH_FRONT_RIGHT  = 2; // VL53L1X front right
constexpr uint8_t TCA_CH_LEFT         = 3; // VL53L1X left
constexpr uint8_t TCA_CH_RIGHT        = 4; // VL53L1X right
constexpr uint8_t TCA_CH_BNO085       = 5; // Reserved (BNO085 can be on main bus)

// Motor driver pins (DRV8871)
constexpr uint8_t PIN_MOTOR_FL_PWM = 2; // Front Left PWM
constexpr uint8_t PIN_MOTOR_FL_DIR = 3; // Front Left DIR
constexpr uint8_t PIN_MOTOR_FR_PWM = 4; // Front Right PWM
constexpr uint8_t PIN_MOTOR_FR_DIR = 5; // Front Right DIR
constexpr uint8_t PIN_MOTOR_RL_PWM = 6; // Rear Left PWM
constexpr uint8_t PIN_MOTOR_RL_DIR = 7; // Rear Left DIR
constexpr uint8_t PIN_MOTOR_RR_PWM = 8; // Rear Right PWM
constexpr uint8_t PIN_MOTOR_RR_DIR = 9; // Rear Right DIR
constexpr uint8_t PIN_MOTOR_STANDBY = 12; // Standby / enable (if used)

// Legacy two-motor mapping (keeps existing DriveSystem compatible)
constexpr uint8_t PIN_MOTOR_LEFT_PWM  = PIN_MOTOR_FL_PWM;
constexpr uint8_t PIN_MOTOR_LEFT_DIR  = PIN_MOTOR_FL_DIR;
constexpr uint8_t PIN_MOTOR_RIGHT_PWM = PIN_MOTOR_FR_PWM;
constexpr uint8_t PIN_MOTOR_RIGHT_DIR = PIN_MOTOR_FR_DIR;

// Encoder pins (A/B for each wheel) - use interrupt-capable pins
constexpr uint8_t PIN_ENC_FL_A = 22; // Front Left A
constexpr uint8_t PIN_ENC_FL_B = 23; // Front Left B
constexpr uint8_t PIN_ENC_FR_A = 20; // Front Right A
constexpr uint8_t PIN_ENC_FR_B = 21; // Front Right B
constexpr uint8_t PIN_ENC_RL_A = 14; // Rear Left A
constexpr uint8_t PIN_ENC_RL_B = 15; // Rear Left B
constexpr uint8_t PIN_ENC_RR_A = 16; // Rear Right A
constexpr uint8_t PIN_ENC_RR_B = 17; // Rear Right B

// Legacy left/right encoder pins (used by DriveSystem)
constexpr uint8_t PIN_ENC_LEFT_A  = PIN_ENC_FL_A;
constexpr uint8_t PIN_ENC_LEFT_B  = PIN_ENC_FL_B;
constexpr uint8_t PIN_ENC_RIGHT_A = PIN_ENC_FR_A;
constexpr uint8_t PIN_ENC_RIGHT_B = PIN_ENC_FR_B;

// IMU and sensor pins
constexpr uint8_t PIN_BNO085_INT = 30; // BNO085 interrupt (can be any free interrupt-capable pin)

// Color sensor and line detectors (analog inputs)
constexpr uint8_t PIN_TCRT_LEFT = A0;
constexpr uint8_t PIN_TCRT_RIGHT = A1;
constexpr uint8_t PIN_TCS34725_INT = A2;
constexpr uint8_t PIN_BATTERY_SENSE = A9; // Battery voltage sensing pin

// LEDs and buzzer
constexpr uint8_t PIN_BLUE_LED = 11;
constexpr uint8_t PIN_LED_GREEN = 33; // External Green LED
constexpr uint8_t PIN_LED_RED = 34;   // External Red LED
constexpr uint8_t PIN_BUZZER = 10;

// VL53L1X reset pins for multiple sensors (non-conflicting pins, kept for hardware flexibility)
constexpr uint8_t PIN_VL53_RESET_PINS[5] = {24, 25, 26, 27, 28};
constexpr uint8_t VL53_ADDRESS[5] = {0x30, 0x31, 0x32, 0x33, 0x34};

// Physical robot constants
constexpr float WHEEL_DIAMETER_MM = 44.0f;
constexpr float WHEEL_BASE_MM = 75.0f;
constexpr float ENCODER_TICKS_PER_REV = 1440.0f;
constexpr float WHEEL_CIRCUMFERENCE_MM = WHEEL_DIAMETER_MM * PI;
constexpr float DISTANCE_PER_TICK_MM = WHEEL_CIRCUMFERENCE_MM / ENCODER_TICKS_PER_REV;
constexpr float CELL_SIZE_MM = 300.0f;

// Maze boundaries (Rescue Maze Arena is max ~6x4 cells, we configure a safe 10x10 grid)
constexpr uint8_t GRID_WIDTH = 10;
constexpr uint8_t GRID_HEIGHT = 10;
constexpr uint8_t MAX_MAP_NODES = GRID_WIDTH * GRID_HEIGHT;

constexpr uint16_t MAX_SENSOR_DISTANCE_MM = 1200;
constexpr uint16_t JUNCTION_THRESHOLD_MM = 260;
constexpr uint16_t WALL_THRESHOLD_MM = 220;

// Safety & voltage check constants
constexpr float BATTERY_MIN_VOLTAGE = 6.8f;      // Safe lower limit for 2S LiPo (3.4V per cell)
constexpr float BATTERY_CRITICAL_VOLTAGE = 6.4f; // Critical threshold to immediately stop motors
constexpr float BATTERY_DIVIDER_RATIO = 2.0f;    // Resistor divider factor (e.g. 10k/10k ratio = 2.0)
constexpr float ADC_REF_VOLTAGE = 3.3f;          // ADC reference on Teensy 4.1
constexpr float ADC_RESOLUTION_MAX = 1023.0f;    // Teensy 10-bit analog read default

// Calibration constants
constexpr uint16_t TCRT_BLACK_THRESHOLD = 500;  // Analog reading below this indicates black tile
constexpr uint16_t TCRT_SILVER_THRESHOLD = 900; // Analog reading above this indicates silver checkpoint
