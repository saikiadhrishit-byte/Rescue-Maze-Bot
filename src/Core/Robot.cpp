#include "Robot.h"

Robot::Robot()
    : drive(motors, encoders, imu),
      wallFollower(tof),
      state(RobotState::Boot),
      currentCell({GRID_WIDTH / 2, GRID_HEIGHT / 2}),
      nextCell({GRID_WIDTH / 2, GRID_HEIGHT / 2}),
      desiredHeading(Navigation::DIR_NORTH),
      pathLength(0),
      pathIndex(0)
{
}

void Robot::begin()
{
  state = RobotState::Boot;
  safetyTimer.start(100);  // Safety check every 100ms
  logTimer.start(500);     // Telemetry log every 500ms
}

void Robot::update()
{
  // 1. Update hardware sensors
  tof.update();
  floor.update();
  ui.update();

  // 2. Perform dead reckoning state estimation
  if (imu.isReady())
  {
    poseEstimator.update(
        encoders.getLeftAverage(),
        encoders.getRightAverage(),
        imu.getHeading()
    );
    currentCell = poseEstimator.getPose().cell;
  }

  // 3. Non-blocking safety check
  if (safetyTimer.isExpired())
  {
    safetyTimer.reset();
    checkSafety();
    checkBattery();
  }

  // 4. Non-blocking telemetry logs
  if (logTimer.isExpired())
  {
    logTimer.reset();
    logTelemetry();
  }

  // 5. Main State Machine Execution
  switch (state)
  {
    case RobotState::Boot:
      handleBoot();
      break;
    case RobotState::Init:
      handleInit();
      break;
    case RobotState::SensorCheck:
      handleSensorCheck();
      break;
    case RobotState::Calibration:
      handleCalibration();
      break;
    case RobotState::Ready:
      handleReady();
      break;
    case RobotState::Exploration:
      handleExploration();
      break;
    case RobotState::MoveForward:
      handleMoveForward();
      break;
    case RobotState::Turn:
      handleTurn();
      break;
    case RobotState::VictimLogic:
      handleVictimLogic();
      break;
    case RobotState::GoalFound:
      handleGoalFound();
      break;
    case RobotState::ShortestPathReturn:
      handleShortestPathReturn();
      break;
    case RobotState::Finish:
      handleFinish();
      break;
  }
}

void Robot::checkBattery()
{
  // Read raw voltage from ADC divider pin
  int rawVal = analogRead(PIN_BATTERY_SENSE);
  float measuredV = (rawVal * ADC_REF_VOLTAGE / ADC_RESOLUTION_MAX) * BATTERY_DIVIDER_RATIO;

  if (measuredV < BATTERY_CRITICAL_VOLTAGE)
  {
    Serial.print("ERROR: CRITICAL BATTERY VOLTAGE DETECTED: ");
    Serial.print(measuredV);
    Serial.println("V. Shutting down.");
    
    ui.playAlertPattern();
    motors.stop();
    state = RobotState::Finish;
  }
  else if (measuredV < BATTERY_MIN_VOLTAGE)
  {
    Serial.print("WARNING: LOW BATTERY: ");
    Serial.print(measuredV);
    Serial.println("V.");
    
    ui.setRed(true); // Light up warning LED
  }
  else
  {
    ui.setRed(false);
  }
}

void Robot::checkSafety()
{
  // 1. Check for rangefinder data loss (sensor disconnect or crash)
  if (tof.checkSensorsTimeout(500))
  {
    Serial.println("WARNING: Rangefinder communication timeout detected.");
    ui.setRed(true);
  }

  // 2. Check for encoder hardware failures
  if (encoders.checkFailure())
  {
    Serial.println("ERROR: Encoder sensor failure detected! Emergency stop triggered.");
    ui.playAlertPattern();
    drive.stopImmediate();
    state = RobotState::Finish;
  }

  // 3. Check for motor stall under load
  if (drive.getMotionState() != Motion::MotionType::Idle)
  {
    int power = (drive.getMotionState() == Motion::MotionType::MovingForward) ? 130 : 100;
    if (encoders.checkStall(power, power, 100))
    {
      Serial.println("WARNING: Motor stall detected. Backing away.");
      ui.beep(300, 1500);
      drive.startMoveForward(-60.0f, 90); // Back away slightly
      state = RobotState::MoveForward;
    }
  }
}

void Robot::handleBoot()
{
  Serial.println("--- Booting Rescue Maze Robot ---");
  ui.begin();
  state = RobotState::Init;
}

void Robot::handleInit()
{
  Serial.println("Initializing hardware layers...");
  
  motors.begin();
  encoders.reset();
  
  bool imuOk = imu.begin();
  bool tofOk = tof.begin();
  bool floorOk = floor.begin();
  
  drive.begin();
  wallFollower.reset();

  if (imuOk && tofOk && floorOk)
  {
    Serial.println("All hardware initialized successfully.");
  }
  else
  {
    Serial.println("Warning: Subsystems missing. Check I2C bus wiring!");
  }

  state = RobotState::SensorCheck;
}

void Robot::handleSensorCheck()
{
  if (!imu.isReady())
  {
    Serial.println("IMU Offline. Waiting...");
    ui.beep(100, 1000);
    delay(500);
    return;
  }

  state = RobotState::Calibration;
}

void Robot::handleCalibration()
{
  Serial.println("Calibrating floor reflective sensors...");
  floor.calibrate();
  
  ui.beep(100, 2000);
  delay(150);
  ui.beep(100, 2000);
  
  state = RobotState::Ready;
  stateTimer.start(3000); // 3-second countdown to start
}

void Robot::handleReady()
{
  ui.startBlinkGreen(250);

  if (stateTimer.isExpired() || (Serial.available() && Serial.read() == 's'))
  {
    ui.stopBlink();
    ui.setGreen(true);
    
    // Reset starting pose at center of starting grid cell facing North
    poseEstimator.reset({GRID_WIDTH / 2, GRID_HEIGHT / 2}, Navigation::DIR_NORTH);
    currentCell = poseEstimator.getPose().cell;
    
    maze.reset();
    maze.setVisited(currentCell.x, currentCell.y, true);
    
    Serial.println("Exploration started.");
    state = RobotState::Exploration;
  }
}

void Robot::handleExploration()
{
  Navigation::Pose pose = poseEstimator.getPose();

  // 1. Mark current cell visited and map its walls
  if (!maze.isVisited(currentCell.x, currentCell.y))
  {
    maze.setVisited(currentCell.x, currentCell.y, true);

    // Wall status in relative directions
    bool wallFC = !tof.isClear(Hardware::FC, WALL_THRESHOLD_MM);
    bool wallFL = !tof.isClear(Hardware::FL, WALL_THRESHOLD_MM);
    bool wallFR = !tof.isClear(Hardware::FR, WALL_THRESHOLD_MM);

    // Map relative directions to absolute compass directions
    bool absoluteWalls[4] = {false};
    absoluteWalls[pose.heading] = wallFC;
    absoluteWalls[Navigation::GridMaze::rotateRight(pose.heading)] = wallFR;
    absoluteWalls[Navigation::GridMaze::rotateLeft(pose.heading)] = wallFL;

    // Set walls in the maze structure
    for (int d = 0; d < 4; ++d)
    {
      maze.setWall(currentCell.x, currentCell.y, static_cast<Navigation::CompassDir>(d), absoluteWalls[d]);
    }

    // Process floor tiles
    TileColor color = floor.getTileColor();
    maze.setTileProperties(currentCell.x, currentCell.y, color);

    if (color == TILE_BLACK)
    {
      Serial.println("Encountered black tile! Backing away.");
      ui.beep(400, 1000);
      drive.startMoveForward(-120.0f, 90);
      state = RobotState::MoveForward;
      return;
    }
    else if (color == TILE_BLUE || color == TILE_SILVER)
    {
      state = RobotState::VictimLogic;
      stateTimer.start(color == TILE_BLUE ? 5000 : 3000);
      return;
    }
  }

  // 2. Run flood fill to update cell weights to closest unvisited tile
  maze.runExplorationFloodFill();

  // 3. Search open neighbors to find one with minimum distance value
  Navigation::CompassDir bestDir = Navigation::DIR_NONE;
  uint8_t minDistance = 255;

  // Prefer straight, right, left, and then reverse
  Navigation::CompassDir priorities[4] = {
    pose.heading,
    Navigation::GridMaze::rotateRight(pose.heading),
    Navigation::GridMaze::rotateLeft(pose.heading),
    Navigation::GridMaze::getOpposite(pose.heading)
  };

  for (int i = 0; i < 4; ++i)
  {
    Navigation::CompassDir dir = priorities[i];
    if (!maze.hasWall(currentCell.x, currentCell.y, dir))
    {
      Navigation::Coordinate nbr = Navigation::GridMaze::getNeighborCoordinate(currentCell, dir);
      if (!maze.isBlackTile(nbr.x, nbr.y))
      {
        uint8_t dist = maze.getDistance(nbr.x, nbr.y);
        if (dist < minDistance)
        {
          minDistance = dist;
          bestDir = dir;
        }
      }
    }
  }

  // 4. Determine state transition
  if (bestDir == Navigation::DIR_NONE || minDistance == 255)
  {
    // No reachable unvisited cell. Exploration completed. Return Home!
    Serial.println("Exploration finished. Return home sequence triggered.");
    state = RobotState::GoalFound;
  }
  else
  {
    nextCell = Navigation::GridMaze::getNeighborCoordinate(currentCell, bestDir);
    desiredHeading = bestDir;

    if (desiredHeading == pose.heading)
    {
      // Move forward one cell
      drive.startMoveForward(CELL_SIZE_MM, 130);
      state = RobotState::MoveForward;
    }
    else
    {
      // Rotate to desired heading
      float turnAngle = Navigation::PoseEstimator::headingToYaw(desiredHeading) - Navigation::PoseEstimator::headingToYaw(pose.heading);
      turnAngle = Motion::DriveController::wrapAngle(turnAngle);
      drive.startTurn(turnAngle, 100);
      state = RobotState::Turn;
    }
  }
}

void Robot::handleMoveForward()
{
  drive.update();

  // Apply non-blocking PID wall-following alignment (only if moving forward, not reversing)
  if (drive.getMotionState() == Motion::MotionType::MovingForward && drive.getDirectionSign() > 0.0f)
  {
    bool hasWall = false;
    double corr = wallFollower.computeCorrection(hasWall);
    drive.setHeadingCorrection(hasWall ? corr : 0.0);
  }
  else
  {
    drive.setHeadingCorrection(0.0);
  }

  // Check for black tile edge safety during forward drive (only when moving forward, not reversing)
  if (drive.getDirectionSign() > 0.0f && floor.isBlackTile())
  {
    Serial.println("Line safety: black line triggered. Stopping and reversing.");
    ui.beep(300, 1000);
    
    // Stop forward motion immediately
    drive.stopImmediate();

    // Mark the cell ahead as black tile
    Navigation::Pose pose = poseEstimator.getPose();
    Navigation::Coordinate ahead = Navigation::GridMaze::getNeighborCoordinate(currentCell, pose.heading);
    maze.setTileProperties(ahead.x, ahead.y, TILE_BLACK);
    maze.setWall(currentCell.x, currentCell.y, pose.heading, true);

    // Back away slightly and return to exploration
    drive.startMoveForward(-100.0f, 90);
    state = RobotState::MoveForward; // Stay in move state until reverse finishes
    return;
  }

  if (drive.isMotionComplete())
  {
    drive.stopImmediate();
    state = RobotState::Exploration;
  }
}

void Robot::handleTurn()
{
  drive.update();

  if (drive.isMotionComplete())
  {
    drive.stopImmediate();
    
    // Once turn is done, move forward immediately into nextCell
    drive.startMoveForward(CELL_SIZE_MM, 130);
    state = RobotState::MoveForward;
  }
}

void Robot::handleVictimLogic()
{
  drive.stopImmediate();

  // Signal checkpoint detection
  ui.setBlue(true);
  ui.beep(150, 1800);

  if (stateTimer.isExpired())
  {
    ui.setBlue(false);
    
    // Re-plan next step
    state = RobotState::Exploration;
  }
}

void Robot::handleGoalFound()
{
  ui.beep(200, 2200);
  delay(100);
  ui.beep(200, 2600);
  delay(100);
  ui.beep(400, 3000);

  // Shortest path back to starting cell center coordinate (5,5)
  Navigation::Coordinate home = {GRID_WIDTH / 2, GRID_HEIGHT / 2};
  pathLength = maze.planPath(currentCell, home, plannedPath, MAX_MAP_NODES);

  if (pathLength > 1)
  {
    pathIndex = 1; // Index 0 is the currentCell
    state = RobotState::ShortestPathReturn;
    
    // Trigger first step turn or move
    nextCell = plannedPath[pathIndex];
    desiredHeading = Navigation::GridMaze::getDirectionTo(currentCell, nextCell);
    Navigation::CompassDir heading = poseEstimator.getPose().heading;

    if (desiredHeading == heading)
    {
      drive.startMoveForward(CELL_SIZE_MM, 140);
    }
    else
    {
      float turnAngle = Navigation::PoseEstimator::headingToYaw(desiredHeading) - Navigation::PoseEstimator::headingToYaw(heading);
      turnAngle = Motion::DriveController::wrapAngle(turnAngle);
      drive.startTurn(turnAngle, 110);
    }
  }
  else
  {
    state = RobotState::Finish;
  }
}

void Robot::handleShortestPathReturn()
{
  drive.update();

  // Active PID wall following alignment on straight segments (only if moving forward, not reversing)
  if (drive.getMotionState() == Motion::MotionType::MovingForward && drive.getDirectionSign() > 0.0f)
  {
    bool hasWall = false;
    double corr = wallFollower.computeCorrection(hasWall);
    drive.setHeadingCorrection(hasWall ? corr : 0.0);
  }
  else
  {
    drive.setHeadingCorrection(0.0);
  }

  if (drive.isMotionComplete())
  {
    drive.stopImmediate();
    
    if (currentCell == plannedPath[pathLength - 1])
    {
      // Arrived at home cell!
      state = RobotState::Finish;
      return;
    }

    pathIndex++;
    if (pathIndex < pathLength)
    {
      nextCell = plannedPath[pathIndex];
      desiredHeading = Navigation::GridMaze::getDirectionTo(currentCell, nextCell);
      Navigation::CompassDir heading = poseEstimator.getPose().heading;

      if (desiredHeading == heading)
      {
        drive.startMoveForward(CELL_SIZE_MM, 140);
      }
      else
      {
        float turnAngle = Navigation::PoseEstimator::headingToYaw(desiredHeading) - Navigation::PoseEstimator::headingToYaw(heading);
        turnAngle = Motion::DriveController::wrapAngle(turnAngle);
        drive.startTurn(turnAngle, 110);
      }
    }
  }
}

void Robot::handleFinish()
{
  motors.stop();
  ui.setGreen(true);
  ui.setRed(false);
  
  // Periodic finish chime every 5 seconds
  static uint32_t lastChime = 0;
  if (millis() - lastChime > 5000)
  {
    lastChime = millis();
    ui.beep(100, 2500);
  }
}

void Robot::logTelemetry()
{
  Navigation::Pose pose = poseEstimator.getPose();
  Serial.print("[Telemetry] State: ");
  Serial.print(stateToString(state));
  Serial.print(" | Cell: (");
  Serial.print(pose.cell.x);
  Serial.print(",");
  Serial.print(pose.cell.y);
  Serial.print(") | Yaw: ");
  Serial.print(pose.yawDeg);
  Serial.print(" | Heading: ");
  Serial.print(Navigation::GridMaze::dirToString(pose.heading));
  Serial.print(" | FL: ");
  Serial.print(tof.getDistance(Hardware::FL));
  Serial.print(" FC: ");
  Serial.print(tof.getDistance(Hardware::FC));
  Serial.print(" FR: ");
  Serial.print(tof.getDistance(Hardware::FR));
  Serial.print(" FloorColor: ");
  Serial.println(floor.getTileColor());
}

const char* Robot::stateToString(RobotState s) const
{
  switch (s)
  {
    case RobotState::Boot:               return "BOOT";
    case RobotState::Init:               return "INIT";
    case RobotState::SensorCheck:        return "SENSOR_CHECK";
    case RobotState::Calibration:        return "CALIBRATION";
    case RobotState::Ready:              return "READY";
    case RobotState::Exploration:        return "EXPLORATION";
    case RobotState::MoveForward:        return "MOVE_FORWARD";
    case RobotState::Turn:               return "TURN";
    case RobotState::VictimLogic:        return "VICTIM_LOGIC";
    case RobotState::GoalFound:          return "GOAL_FOUND";
    case RobotState::ShortestPathReturn: return "SHORTEST_PATH_RETURN";
    case RobotState::Finish:             return "FINISH";
  }
  return "UNKNOWN";
}
