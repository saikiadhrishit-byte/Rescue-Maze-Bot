#pragma once

#include "RobotConfig.h"
#include "TileTypes.h"
#include <Arduino.h>

enum Direction {
  NORTH = 0,
  EAST = 1,
  SOUTH = 2,
  WEST = 3
};

struct RobotPose {
  int8_t x;
  int8_t y;
  Direction heading;
};

struct MazeNode {
  uint8_t id;
  int8_t x;
  int8_t y;
  bool open[4];
  int8_t neighbor[4];
  bool visited;
  bool hasBlackTile;
  bool hasBlueTile;
  bool hasSilverTile;
};

class MazeMapper
{
public:
  MazeMapper();
  void reset();
  int findNode(int8_t x, int8_t y) const;
  int addOrUpdateNode(const RobotPose &pose, const bool open[4]);
  void recordJunction(const RobotPose &pose, const bool open[4]);
  void setTileColor(int nodeId, TileColor color);
  void setEdgeOpen(int nodeId, Direction dir, bool isOpen);
  int bfs(int startId, int targetId, int path[], uint8_t pathSize) const;
  Direction directionToNeighbor(int fromId, int toId) const;
  int getNodeCount() const;

  static Direction rotateRight(Direction d);
  static Direction rotateLeft(Direction d);
  static RobotPose moveForward(const RobotPose &pose);

private:
  MazeNode nodes[MAX_MAP_NODES];
  uint8_t nodeCount;
  void linkNeighbors(int a, int b, Direction dir);
};
