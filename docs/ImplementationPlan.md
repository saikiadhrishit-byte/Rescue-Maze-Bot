# Rescue Maze Robot - Implementation Plan

This document outlines the design and implementation details for upgrading the Rescue Maze robot codebase into a modular, robust, competition-grade modern C++ system.

## 1. Directory Structure Organization
We will reorganize the flat `src/` directory into the requested modular structure:
- `src/Core/`: Core robot coordinator, state machine, main loop, logging.
- `src/Hardware/`: Motor controls (4WD), encoder readings, IMU (BNO085), Range Finder (VL53L1X via TCA9548A), Floor sensors (TCRT5000 + TCS34725 color sensor), LEDs, Buzzer.
- `src/Motion/`: PID wall follower, speed profiler (linear/S-curve acceleration), precise turns, collision avoidance.
- `src/Navigation/`: 2D Grid Maze representation, Flood Fill, BFS, Shortest Path planning, Position/Pose Estimator (Odometry + IMU integration).
- `src/Utilities/`: Non-blocking Timer, Moving average / Median filters, Math utils.
- `docs/`: Technical documentation (Mechanical, Electronics, Pinout, Assembly, Software, Algorithms, Calibration, Strategy, Testing).

## 2. Architecture & Design Upgrades

### Hardware Layer
1. **4WD Skid Steer & Encoders**: Update `DriveSystem` to control four independent N20 motors and read four separate encoders. Implement stall detection and encoder slip analysis.
2. **I2C Multiplexer (TCA9548A)**: Replace the resetting address scheme in `RangeFinder` with proper TCA9548A channel selection, ensuring high reliability and avoiding sensor reset communication drops.
3. **Floor Sensors**: Handle 2x TCRT5000 analog readings for line/black tile/silver checkpoint detection with automatic calibration. Use the TCS34725 color sensor only when triggered to detect blue tiles (checkpoint/rescue zone) and victims.
4. **Safety Watchdogs**:
   - **Battery Check**: Monitor voltage divider on an analog pin, beep and stop on low voltage.
   - **Motor Stall Detection**: Check if motor PWM is high but encoder count is not changing.
   - **Sensor Timeout**: Detect if a VL53L1X is not responding and flag it as inactive.

### Motion Layer
1. **Speed Profiling**: Implement smooth speed ramp-up (acceleration) and ramp-down (deceleration) to prevent slip.
2. **Precision Turn PID**: Pivot turns (90° and 180°) using IMU heading feedback combined with encoder targets.
3. **PID Wall Follower**: Maintain corridor center using Left and Right ToF sensors, fallback to single-wall following if one side opens.

### Navigation Layer
1. **Grid-Based Maze**: Implement a 2D grid class (`Maze`) representing cells with coordinate offsets. Each cell stores: `visited`, `walls` (4 directions), `hasBlackTile`, `hasCheckpoint` (blue/silver), `distance`, `parent`, and status flags.
2. **Flood Fill & BFS**:
   - Flood Fill for generating optimal exploration steps toward unvisited cells.
   - BFS for finding the absolute shortest path back to the start cell (0,0) once the run finishes or a return is triggered.
3. **Pose Estimator**: Use dead reckoning combining encoder counts (distance) and BNO085 yaw angle (heading) to update the cell position.

### Core Layer
1. **State Machine**: Completely non-blocking, timer-driven state machine:
   `Boot` -> `Initialization` -> `Sensor Check` -> `Calibration` -> `Ready` -> `Exploration` -> `Mapping` -> `Victim Logic` -> `Goal Found` -> `Shortest Path Return` -> `Finish`.

---

## 3. Implementation Steps

### Step 1: Create Utilities & Constants
- `include/RobotConfig.h`: Update pin map to support 4WD, TCRT thresholds, and safety limits.
- `include/Timer.h` / `src/Utilities/Timer.cpp`: Non-blocking milliseconds/microseconds timers.
- `include/Filters.h` / `src/Utilities/Filters.cpp`: Median and Moving Average classes.

### Step 2: Implement Hardware Layer
- `include/Motors.h` & `src/Hardware/Motors.cpp`: 4WD skid-steer driver.
- `include/Encoders.h` & `src/Hardware/Encoders.cpp`: Wrapper for 4 x Encoder objects.
- `include/IMU.h` & `src/Hardware/IMU.cpp`: Refine BNO085 service loop.
- `include/ToF.h` & `src/Hardware/ToF.cpp`: TCA9548A-based VL53L1X range finder.
- `include/FloorSensor.h` & `src/Hardware/FloorSensor.cpp`: TCRT5000 & TCS34725.
- `include/BuzzerLED.h` & `src/Hardware/BuzzerLED.cpp`: Non-blocking blink and beep patterns.

### Step 3: Implement Navigation Layer
- `include/GridMaze.h` & `src/Navigation/GridMaze.cpp`: Grid-based map with Flood Fill and BFS.
- `include/PoseEstimator.h` & `src/Navigation/PoseEstimator.cpp`: Odometry + IMU integration.

### Step 4: Implement Motion Layer
- `include/SpeedProfiler.h` & `src/Motion/SpeedProfiler.cpp`: S-curve and linear acceleration.
- `include/DriveController.h` & `src/Motion/DriveController.cpp`: Coordinates motors, encoders, and IMU.
- `include/PIDWallFollower.h` & `src/Motion/PIDWallFollower.cpp`: Corridor centering.

### Step 5: Implement Core Layer & State Machine
- `include/Robot.h` & `src/Core/Robot.cpp`: Combines all subsystems.
- `src/Core/main.cpp`: Main loop running state transitions.

### Step 6: Create Competition Documentation
- Write structured technical Markdown files under `docs/`.
