#pragma once

#include <Arduino.h>
#include "RobotConfig.h"
#include "Motors.h"
#include "Encoders.h"
#include "IMU.h"
#include "ToF.h"
#include "FloorSensor.h"
#include "BuzzerLED.h"
#include "GridMaze.h"
#include "PoseEstimator.h"
#include "DriveController.h"
#include "PIDWallFollower.h"
#include "Timer.h"

enum class RobotState
{
  Boot,
  Init,
  SensorCheck,
  Calibration,
  Ready,
  Exploration,
  MoveForward,
  Turn,
  VictimLogic,
  GoalFound,
  ShortestPathReturn,
  Finish
};

class Robot
{
public:
  Robot();
  void begin();
  void update();

  RobotState getState() const { return state; }
  const char* stateToString(RobotState s) const;

private:
  // Hardware components
  Hardware::Motors motors;
  Hardware::Encoders encoders;
  IMU imu;
  Hardware::ToF tof;
  Hardware::FloorSensor floor;
  Hardware::BuzzerLED ui;

  // Navigation & Motion components
  Navigation::GridMaze maze;
  Navigation::PoseEstimator poseEstimator;
  Motion::DriveController drive;
  Motion::PIDWallFollower wallFollower;

  // Robot State tracking
  RobotState state;
  Navigation::Coordinate currentCell;
  Navigation::Coordinate nextCell;
  Navigation::CompassDir desiredHeading;

  // Path planning variables
  Navigation::Coordinate plannedPath[MAX_MAP_NODES];
  int pathLength;
  int pathIndex;

  // Timers
  Timer stateTimer;
  Timer safetyTimer;
  Timer logTimer;

  // Safety checks
  void checkBattery();
  void checkSafety();
  
  // State Machine handlers
  void handleBoot();
  void handleInit();
  void handleSensorCheck();
  void handleCalibration();
  void handleReady();
  void handleExploration();
  void handleMoveForward();
  void handleTurn();
  void handleVictimLogic();
  void handleGoalFound();
  void handleShortestPathReturn();
  void handleFinish();

  // Log robot telemetry
  void logTelemetry();
};
