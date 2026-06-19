#include <Arduino.h>
#include "RobotConfig.h"
#include "IMU.h"
#include "DriveSystem.h"
#include "RangeFinder.h"
#include "MazeMapper.h"
#include "ColorDetector.h"
#include "Buzzer.h"
#include "WallFollower.h"

enum RobotState {
  STATE_IDLE,
  STATE_INIT,
  STATE_EXPLORE,
  STATE_MOVE_FORWARD,
  STATE_TURN,
  STATE_SCAN_TILE,
  STATE_RETURN_HOME,
  STATE_FINISHED
};

IMU imu;
DriveSystem drive(PIN_ENC_LEFT_A, PIN_ENC_LEFT_B,
                  PIN_ENC_RIGHT_A, PIN_ENC_RIGHT_B,
                  PIN_MOTOR_LEFT_PWM, PIN_MOTOR_LEFT_DIR,
                  PIN_MOTOR_RIGHT_PWM, PIN_MOTOR_RIGHT_DIR,
                  PIN_MOTOR_STANDBY);
RangeFinder rangeFinder;
MazeMapper maze;
WallFollower wallFollower(rangeFinder, drive);
ColorDetector colorDetector;

const char* headingToString(Direction heading)
{
  switch (heading)
  {
    case NORTH: return "NORTH";
    case EAST: return "EAST";
    case SOUTH: return "SOUTH";
    case WEST: return "WEST";
  }
  return "UNKNOWN";
}

RobotState state = STATE_IDLE;
RobotPose pose = {0, 0, NORTH};
int currentNodeId = -1;
bool returnHomeRequested = false;

void setup()
{
  Serial.begin(115200);
  delay(2000);

  pinMode(PIN_TCRT_LEFT, INPUT_PULLUP);
  pinMode(PIN_TCRT_RIGHT, INPUT_PULLUP);

  pinMode(PIN_BLUE_LED, OUTPUT);
  digitalWrite(PIN_BLUE_LED, LOW);
  Buzzer::begin();

  state = STATE_INIT;
}

void loop()
{
  if (Serial.available())
  {
    char command = static_cast<char>(Serial.read());
    if (command == 'h' || command == 'H')
    {
      returnHomeRequested = true;
    }
  }

  switch (state)
  {
    case STATE_IDLE:
      state = STATE_INIT;
      break;

    case STATE_INIT:
    {
      Serial.println("Initializing robot...");
      if (!imu.begin())
      {
        Serial.println("IMU init failed");
        delay(1000);
        return;
      }

      if (!drive.begin(&imu))
      {
        Serial.println("Drive init failed");
        delay(1000);
        return;
      }

      if (!rangeFinder.begin())
      {
        Serial.println("Range finder init failed");
        delay(1000);
        return;
      }

      if (!colorDetector.begin())
      {
        Serial.println("Color detector init failed");
        delay(1000);
        return;
      }

      maze.reset();
      pose = {0, 0, NORTH};

      bool startOpen[4] = {true, true, true, true};
      currentNodeId = maze.addOrUpdateNode(pose, startOpen);
      state = STATE_EXPLORE;
      break;
    }

    case STATE_EXPLORE:
    {
      rangeFinder.update();
      drive.update();
      colorDetector.update();

      if (colorDetector.tileTriggered())
      {
        state = STATE_SCAN_TILE;
        break;
      }

      auto toAbsolute = [&](Direction relative) {
        return static_cast<Direction>((pose.heading + relative) % 4);
      };

      bool open[4] = {false, false, false, false};
      open[toAbsolute(NORTH)] = rangeFinder.isClear(FRONT_CENTER, WALL_THRESHOLD_MM);
      open[toAbsolute(EAST)] = rangeFinder.isClear(FRONT_RIGHT, WALL_THRESHOLD_MM);
      open[toAbsolute(WEST)] = rangeFinder.isClear(FRONT_LEFT, WALL_THRESHOLD_MM);

      currentNodeId = maze.addOrUpdateNode(pose, open);
      Serial.print("Status: node=");
      Serial.print(currentNodeId);
      Serial.print(" pose=(");
      Serial.print(pose.x);
      Serial.print(",");
      Serial.print(pose.y);
      Serial.print(") heading=");
      Serial.println(headingToString(pose.heading));

      if (returnHomeRequested)
      {
        state = STATE_RETURN_HOME;
      }
      else
      {
        state = open[toAbsolute(NORTH)] ? STATE_MOVE_FORWARD : STATE_TURN;
      }
      break;
    }

    case STATE_MOVE_FORWARD:
    {
      drive.resetEncoders();
      int32_t targetTicks = static_cast<int32_t>(CELL_SIZE_MM / DISTANCE_PER_TICK_MM);

      while (true)
      {
        drive.update();
        if (colorDetector.tileTriggered())
        {
          drive.stop();
          state = STATE_SCAN_TILE;
          break;
        }

        int32_t leftTicks = abs(drive.leftEncoderCount());
        int32_t rightTicks = abs(drive.rightEncoderCount());
        if (leftTicks >= targetTicks || rightTicks >= targetTicks)
          break;

        wallFollower.maintainCenter(120);
      }

      if (state == STATE_MOVE_FORWARD)
      {
        drive.stop();
        pose = MazeMapper::moveForward(pose);
        rangeFinder.update();

        auto toAbsolute = [&](Direction relative) {
          return static_cast<Direction>((pose.heading + relative) % 4);
        };

        bool open[4] = {false, false, false, false};
        open[toAbsolute(NORTH)] = rangeFinder.isClear(FRONT_CENTER, WALL_THRESHOLD_MM);
        open[toAbsolute(EAST)] = rangeFinder.isClear(FRONT_RIGHT, WALL_THRESHOLD_MM);
        open[toAbsolute(WEST)] = rangeFinder.isClear(FRONT_LEFT, WALL_THRESHOLD_MM);
        currentNodeId = maze.addOrUpdateNode(pose, open);
        state = STATE_EXPLORE;
      }
      break;
    }

    case STATE_TURN:
    {
      rangeFinder.update();
      bool frontOpen = rangeFinder.isClear(FRONT_CENTER, WALL_THRESHOLD_MM);
      bool rightOpen = rangeFinder.isClear(FRONT_RIGHT, WALL_THRESHOLD_MM);
      bool leftOpen = rangeFinder.isClear(FRONT_LEFT, WALL_THRESHOLD_MM);

      if (rightOpen)
      {
        drive.turnRight90();
        pose.heading = MazeMapper::rotateRight(pose.heading);
      }
      else if (frontOpen)
      {
        // keep heading
      }
      else if (leftOpen)
      {
        drive.turnLeft90();
        pose.heading = MazeMapper::rotateLeft(pose.heading);
      }
      else
      {
        drive.turnRight90();
        drive.turnRight90();
        pose.heading = MazeMapper::rotateRight(MazeMapper::rotateRight(pose.heading));
      }

      state = STATE_EXPLORE;
      break;
    }

    case STATE_SCAN_TILE:
    {
      TileColor color = colorDetector.getTileColor();
      Serial.print("Detected tile: ");
      switch (color)
      {
        case TILE_BLACK: Serial.println("BLACK"); break;
        case TILE_BLUE: Serial.println("BLUE"); break;
        case TILE_SILVER: Serial.println("SILVER"); break;
        default: Serial.println("UNKNOWN"); break;
      }
      // Enforce tile behavior: black = treat as wall (do not enter), blue = pause 5s (handled in detector)
      if (color == TILE_BLACK)
      {
        // stop and back away a bit
        drive.stop();
        drive.setMotorPower(-120, -120);
        delay(350);
        drive.stop();

        // mark current node with the black tile metadata and close the forward edge
        maze.setTileColor(currentNodeId, color);
        maze.setEdgeOpen(currentNodeId, pose.heading, false);
      }
      else
      {
        maze.setTileColor(currentNodeId, color);
      }

      state = STATE_EXPLORE;
      break;
    }

    case STATE_RETURN_HOME:
    {
      if (currentNodeId == 0)
      {
        Serial.println("Already at home node");
        state = STATE_FINISHED;
        break;
      }

      int path[MAX_MAP_NODES];
      int pathLength = maze.bfs(currentNodeId, 0, path, MAX_MAP_NODES);
      if (pathLength <= 1)
      {
        Serial.println("No path home");
        state = STATE_FINISHED;
        break;
      }

      Serial.print("Returning home via ");
      Serial.print(pathLength);
      Serial.println(" nodes");

      for (int idx = 1; idx < pathLength; ++idx)
      {
        int nextNode = path[idx];
        Direction desired = maze.directionToNeighbor(currentNodeId, nextNode);
        if (desired != pose.heading)
        {
          if ((desired + 4 - pose.heading) % 4 == 1)
          {
            drive.turnRight90();
          }
          else if ((pose.heading + 4 - desired) % 4 == 1)
          {
            drive.turnLeft90();
          }
          else
          {
            drive.turnRight90();
            drive.turnRight90();
          }
          pose.heading = desired;
        }

        drive.moveForward(CELL_SIZE_MM, 120);
        currentNodeId = nextNode;
        pose = MazeMapper::moveForward(pose);
      }

      Serial.println("Arrived at home.");
      state = STATE_FINISHED;
      break;
    }

    case STATE_FINISHED:
      drive.stop();
      break;
  }

  delay(50);
}
