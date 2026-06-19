#include "GridMaze.h"

namespace Navigation
{

GridMaze::GridMaze()
{
  reset();
}

void GridMaze::reset()
{
  for (int8_t x = 0; x < GRID_WIDTH; ++x)
  {
    for (int8_t y = 0; y < GRID_HEIGHT; ++y)
    {
      grid[x][y].visited = false;
      grid[x][y].hasBlackTile = false;
      grid[x][y].hasCheckpoint = false;
      grid[x][y].hasSilver = false;
      grid[x][y].distance = 255;
      grid[x][y].parentX = -1;
      grid[x][y].parentY = -1;
      for (int d = 0; d < 4; ++d)
      {
        grid[x][y].walls[d] = false;
      }

      // Pre-add boundary walls for the maze perimeter
      if (y == 0) grid[x][y].walls[DIR_NORTH] = true;
      if (x == GRID_WIDTH - 1) grid[x][y].walls[DIR_EAST] = true;
      if (y == GRID_HEIGHT - 1) grid[x][y].walls[DIR_SOUTH] = true;
      if (x == 0) grid[x][y].walls[DIR_WEST] = true;
    }
  }
}

bool GridMaze::isValid(int8_t x, int8_t y) const
{
  return x >= 0 && x < GRID_WIDTH && y >= 0 && y < GRID_HEIGHT;
}

bool GridMaze::isVisited(int8_t x, int8_t y) const
{
  return isValid(x, y) && grid[x][y].visited;
}

void GridMaze::setVisited(int8_t x, int8_t y, bool visited)
{
  if (isValid(x, y))
  {
    grid[x][y].visited = visited;
  }
}

uint8_t GridMaze::getDistance(int8_t x, int8_t y) const
{
  if (isValid(x, y))
  {
    return grid[x][y].distance;
  }
  return 255;
}

bool GridMaze::hasWall(int8_t x, int8_t y, CompassDir dir) const
{
  if (!isValid(x, y) || dir == DIR_NONE) return true;
  return grid[x][y].walls[dir];
}

void GridMaze::setWall(int8_t x, int8_t y, CompassDir dir, bool wallPresent)
{
  if (!isValid(x, y) || dir == DIR_NONE) return;
  grid[x][y].walls[dir] = wallPresent;

  // Set corresponding wall on the neighbor cell to maintain consistency
  Coordinate nbr = getNeighborCoordinate({x, y}, dir);
  if (isValid(nbr.x, nbr.y))
  {
    grid[nbr.x][nbr.y].walls[getOpposite(dir)] = wallPresent;
  }
}

void GridMaze::setTileProperties(int8_t x, int8_t y, TileColor color)
{
  if (!isValid(x, y)) return;
  grid[x][y].hasBlackTile = (color == TILE_BLACK);
  grid[x][y].hasCheckpoint = (color == TILE_BLUE || color == TILE_SILVER);
  grid[x][y].hasSilver = (color == TILE_SILVER);

  // If a tile is black, seal all walls surrounding it as a safety precaution
  if (color == TILE_BLACK)
  {
    for (int d = 0; d < 4; ++d)
    {
      setWall(x, y, static_cast<CompassDir>(d), true);
    }
  }
}

bool GridMaze::isBlackTile(int8_t x, int8_t y) const
{
  return isValid(x, y) && grid[x][y].hasBlackTile;
}

bool GridMaze::isCheckpoint(int8_t x, int8_t y) const
{
  return isValid(x, y) && grid[x][y].hasCheckpoint;
}

void GridMaze::runFloodFill(Coordinate target)
{
  if (!isValid(target.x, target.y)) return;

  // 1. Initialize distances
  for (int8_t x = 0; x < GRID_WIDTH; ++x)
  {
    for (int8_t y = 0; y < GRID_HEIGHT; ++y)
    {
      grid[x][y].distance = 255;
    }
  }

  // 2. Queue setup
  Coordinate queue[MAX_MAP_NODES];
  int head = 0;
  int tail = 0;

  grid[target.x][target.y].distance = 0;
  queue[tail++] = target;

  // 3. Flood Fill BFS
  while (head < tail)
  {
    Coordinate curr = queue[head++];
    uint8_t currDist = grid[curr.x][curr.y].distance;

    for (int d = 0; d < 4; ++d)
    {
      CompassDir dir = static_cast<CompassDir>(d);
      if (!grid[curr.x][curr.y].walls[dir])
      {
        Coordinate nbr = getNeighborCoordinate(curr, dir);
        if (isValid(nbr.x, nbr.y) && !grid[nbr.x][nbr.y].hasBlackTile)
        {
          if (grid[nbr.x][nbr.y].distance > currDist + 1)
          {
            grid[nbr.x][nbr.y].distance = currDist + 1;
            queue[tail++] = nbr;
          }
        }
      }
    }
  }
}

void GridMaze::runExplorationFloodFill()
{
  // 1. Initialize distances to infinity (255)
  for (int8_t x = 0; x < GRID_WIDTH; ++x)
  {
    for (int8_t y = 0; y < GRID_HEIGHT; ++y)
    {
      grid[x][y].distance = 255;
    }
  }

  // 2. Queue setup
  Coordinate queue[MAX_MAP_NODES];
  int head = 0;
  int tail = 0;

  // Add all unvisited, non-black cells as target nodes (distance 0)
  for (int8_t x = 0; x < GRID_WIDTH; ++x)
  {
    for (int8_t y = 0; y < GRID_HEIGHT; ++y)
    {
      if (!grid[x][y].visited && !grid[x][y].hasBlackTile)
      {
        grid[x][y].distance = 0;
        if (tail < MAX_MAP_NODES)
        {
          queue[tail++] = {x, y};
        }
      }
    }
  }

  // 3. Flood Fill BFS propagation
  while (head < tail)
  {
    Coordinate curr = queue[head++];
    uint8_t currDist = grid[curr.x][curr.y].distance;

    for (int d = 0; d < 4; ++d)
    {
      CompassDir dir = static_cast<CompassDir>(d);
      if (!grid[curr.x][curr.y].walls[dir])
      {
        Coordinate nbr = getNeighborCoordinate(curr, dir);
        if (isValid(nbr.x, nbr.y) && !grid[nbr.x][nbr.y].hasBlackTile)
        {
          if (grid[nbr.x][nbr.y].distance > currDist + 1)
          {
            grid[nbr.x][nbr.y].distance = currDist + 1;
            if (tail < MAX_MAP_NODES)
            {
              queue[tail++] = nbr;
            }
          }
        }
      }
    }
  }
}

int GridMaze::planPath(Coordinate start, Coordinate target, Coordinate path[], int maxLen)
{
  if (!isValid(start.x, start.y) || !isValid(target.x, target.y)) return 0;
  if (start == target)
  {
    path[0] = start;
    return 1;
  }

  // BFS Queue Setup
  Coordinate queue[MAX_MAP_NODES];
  bool visitedMatrix[GRID_WIDTH][GRID_HEIGHT] = {false};
  int8_t parentX[GRID_WIDTH][GRID_HEIGHT];
  int8_t parentY[GRID_WIDTH][GRID_HEIGHT];

  for (int8_t x = 0; x < GRID_WIDTH; ++x)
  {
    for (int8_t y = 0; y < GRID_HEIGHT; ++y)
    {
      parentX[x][y] = -1;
      parentY[x][y] = -1;
    }
  }

  int head = 0;
  int tail = 0;

  queue[tail++] = start;
  visitedMatrix[start.x][start.y] = true;

  bool found = false;

  while (head < tail)
  {
    Coordinate curr = queue[head++];
    if (curr == target)
    {
      found = true;
      break;
    }

    for (int d = 0; d < 4; ++d)
    {
      CompassDir dir = static_cast<CompassDir>(d);
      if (!grid[curr.x][curr.y].walls[dir])
      {
        Coordinate nbr = getNeighborCoordinate(curr, dir);
        if (isValid(nbr.x, nbr.y) && !visitedMatrix[nbr.x][nbr.y] && !grid[nbr.x][nbr.y].hasBlackTile)
        {
          visitedMatrix[nbr.x][nbr.y] = true;
          parentX[nbr.x][nbr.y] = curr.x;
          parentY[nbr.x][nbr.y] = curr.y;
          queue[tail++] = nbr;
        }
      }
    }
  }

  if (!found) return 0;

  // Reconstruct path backward
  Coordinate tempPath[MAX_MAP_NODES];
  int tempLen = 0;
  Coordinate curr = target;

  while (!(curr == start) && tempLen < MAX_MAP_NODES)
  {
    tempPath[tempLen++] = curr;
    int8_t px = parentX[curr.x][curr.y];
    int8_t py = parentY[curr.x][curr.y];
    curr = {px, py};
  }
  tempPath[tempLen++] = start;

  // Reverse tempPath into output path
  int count = 0;
  for (int i = tempLen - 1; i >= 0 && count < maxLen; --i)
  {
    path[count++] = tempPath[i];
  }

  return count;
}

Coordinate GridMaze::getNeighborCoordinate(Coordinate current, CompassDir dir)
{
  Coordinate nbr = current;
  switch (dir)
  {
    case DIR_NORTH: nbr.y -= 1; break;
    case DIR_EAST:  nbr.x += 1; break;
    case DIR_SOUTH: nbr.y += 1; break;
    case DIR_WEST:  nbr.x -= 1; break;
    default: break;
  }
  return nbr;
}

CompassDir GridMaze::getDirectionTo(Coordinate from, Coordinate to)
{
  if (to.x == from.x && to.y == from.y - 1) return DIR_NORTH;
  if (to.x == from.x + 1 && to.y == from.y)   return DIR_EAST;
  if (to.x == from.x && to.y == from.y + 1) return DIR_SOUTH;
  if (to.x == from.x - 1 && to.y == from.y)   return DIR_WEST;
  return DIR_NONE;
}

CompassDir GridMaze::rotateRight(CompassDir dir)
{
  if (dir == DIR_NONE) return DIR_NONE;
  return static_cast<CompassDir>((dir + 1) % 4);
}

CompassDir GridMaze::rotateLeft(CompassDir dir)
{
  if (dir == DIR_NONE) return DIR_NONE;
  return static_cast<CompassDir>((dir + 3) % 4);
}

CompassDir GridMaze::getOpposite(CompassDir dir)
{
  if (dir == DIR_NONE) return DIR_NONE;
  return static_cast<CompassDir>((dir + 2) % 4);
}

const char* GridMaze::dirToString(CompassDir dir)
{
  switch (dir)
  {
    case DIR_NORTH: return "NORTH";
    case DIR_EAST:  return "EAST";
    case DIR_SOUTH: return "SOUTH";
    case DIR_WEST:  return "WEST";
    default:        return "NONE";
  }
}

} // namespace Navigation
