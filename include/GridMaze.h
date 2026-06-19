#pragma once

#include <Arduino.h>
#include "RobotConfig.h"
#include "TileTypes.h"

namespace Navigation
{

enum CompassDir {
  DIR_NORTH = 0,
  DIR_EAST = 1,
  DIR_SOUTH = 2,
  DIR_WEST = 3,
  DIR_NONE = 4
};

struct Cell
{
  bool visited = false;
  bool walls[4] = {false, false, false, false}; // North, East, South, West
  bool hasBlackTile = false;
  bool hasCheckpoint = false;
  bool hasSilver = false;
  uint8_t distance = 255;
  int8_t parentX = -1;
  int8_t parentY = -1;
};

struct Coordinate
{
  int8_t x;
  int8_t y;

  bool operator==(const Coordinate &other) const
  {
    return x == other.x && y == other.y;
  }
};

class GridMaze
{
public:
  GridMaze();
  void reset();

  // Cell state getters/setters
  bool isVisited(int8_t x, int8_t y) const;
  void setVisited(int8_t x, int8_t y, bool visited);
  uint8_t getDistance(int8_t x, int8_t y) const;
  
  bool hasWall(int8_t x, int8_t y, CompassDir dir) const;
  void setWall(int8_t x, int8_t y, CompassDir dir, bool wallPresent);

  void setTileProperties(int8_t x, int8_t y, TileColor color);
  bool isBlackTile(int8_t x, int8_t y) const;
  bool isCheckpoint(int8_t x, int8_t y) const;

  // Pathfinding algorithms
  void runFloodFill(Coordinate target);
  void runExplorationFloodFill();
  int planPath(Coordinate start, Coordinate target, Coordinate path[], int maxLen);

  // Helper coordinate math
  static Coordinate getNeighborCoordinate(Coordinate current, CompassDir dir);
  static CompassDir getDirectionTo(Coordinate from, Coordinate to);
  static CompassDir rotateRight(CompassDir dir);
  static CompassDir rotateLeft(CompassDir dir);
  static CompassDir getOpposite(CompassDir dir);

  static const char* dirToString(CompassDir dir);

private:
  Cell grid[GRID_WIDTH][GRID_HEIGHT];
  bool isValid(int8_t x, int8_t y) const;
};

} // namespace Navigation
