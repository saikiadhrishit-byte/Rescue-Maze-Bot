# Rescue-Maze-Bot

A Teensy 4.1-based rescue maze robot firmware built with PlatformIO.

## Project Overview

This project implements a layered maze exploration and return-home system for a small robot. The robot uses:

- SparkFun BNO08x IMU for heading and turn stabilization
- VL53L1X ToF distance sensors for wall and junction detection
- TCRT reflective sensors for black/blue tile detection
- A buzzer for black tile alerts
- A blue status LED for blue tile detection
- PID control for wall following and motion correction
- A graph-based mapper for junction navigation and home return

## Key Behaviors

- Avoids entering black tiles
- Pauses on blue tiles and lights a blue LED
- Uses a DFS/Tremaux-style exploration strategy
- Records junctions and path connectivity in a map
- Returns home using BFS on the discovered graph

## Repository Structure

- `platformio.ini` - PlatformIO project configuration
- `include/` - Hardware and software module headers
- `src/` - Implementation files and main robot state machine
- `test/` - Placeholder for unit or integration tests

## Build Instructions

1. Open the project in VS Code with PlatformIO.
2. Ensure Teensy 4.1 and required libraries are connected.
3. Build with:

```bash
platformio run
```

4. Upload with:

```bash
platformio run --target upload
```

## Notes

- The current firmware includes serial status output for node position and heading.
- Real-world tuning may be required for sensor thresholds and PID gains.
- This repository is set up to push changes to GitHub and track development.
