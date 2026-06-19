#include "MazeMapper.h"
#include <limits.h>

MazeMapper::MazeMapper()
{
  reset();
}

void MazeMapper::reset()
{
  nodeCount = 0;
  for (uint8_t i = 0; i < MAX_MAP_NODES; ++i)
  {
    nodes[i].id = i;
    nodes[i].x = 0;
    nodes[i].y = 0;
    nodes[i].visited = false;
    for (uint8_t d = 0; d < 4; ++d)
    {
      nodes[i].open[d] = false;
      nodes[i].neighbor[d] = -1;
    }
  }
}

int MazeMapper::findNode(int8_t x, int8_t y) const
{
  for (uint8_t i = 0; i < nodeCount; ++i)
  {
    if (nodes[i].x == x && nodes[i].y == y)
      return i;
  }
  return -1;
}

int MazeMapper::addOrUpdateNode(const RobotPose &pose, const bool open[4])
{
  int existing = findNode(pose.x, pose.y);
  if (existing >= 0)
  {
    for (uint8_t d = 0; d < 4; ++d)
    {
      nodes[existing].open[d] = open[d] || nodes[existing].open[d];
    }
    return existing;
  }

  if (nodeCount >= MAX_MAP_NODES)
    return -1;

  int index = nodeCount++;
  nodes[index].x = pose.x;
  nodes[index].y = pose.y;
  nodes[index].visited = true;
  nodes[index].hasBlackTile = false;
  nodes[index].hasBlueTile = false;
  nodes[index].hasSilverTile = false;

  for (uint8_t d = 0; d < 4; ++d)
  {
    nodes[index].open[d] = open[d];
    nodes[index].neighbor[d] = -1;
  }

  RobotPose forwardPose = pose;
  switch (pose.heading)
  {
    case NORTH:
      forwardPose.y -= 1;
      break;
    case EAST:
      forwardPose.x += 1;
      break;
    case SOUTH:
      forwardPose.y += 1;
      break;
    case WEST:
      forwardPose.x -= 1;
      break;
  }

  int forwardNeighbor = findNode(forwardPose.x, forwardPose.y);
  if (forwardNeighbor >= 0 && open[pose.heading])
  {
    linkNeighbors(index, forwardNeighbor, pose.heading);
  }

  return index;
}

void MazeMapper::setTileColor(int nodeId, TileColor color)
{
  if (nodeId < 0 || nodeId >= nodeCount)
    return;

  nodes[nodeId].hasBlackTile = (color == TILE_BLACK);
  nodes[nodeId].hasBlueTile = (color == TILE_BLUE);
  nodes[nodeId].hasSilverTile = (color == TILE_SILVER);
}

void MazeMapper::setEdgeOpen(int nodeId, Direction dir, bool isOpen)
{
  if (nodeId < 0 || nodeId >= nodeCount)
    return;

  nodes[nodeId].open[dir] = isOpen;
  int neighbor = nodes[nodeId].neighbor[dir];
  if (!isOpen && neighbor >= 0)
  {
    nodes[nodeId].neighbor[dir] = -1;
    nodes[neighbor].neighbor[(dir + 2) % 4] = -1;
  }
}

void MazeMapper::recordJunction(const RobotPose &pose, const bool open[4])
{
  addOrUpdateNode(pose, open);
}

int MazeMapper::bfs(int startId, int targetId, int path[], uint8_t pathSize) const
{
  if (startId < 0 || targetId < 0 || startId >= nodeCount || targetId >= nodeCount)
    return 0;

  int queue[MAX_MAP_NODES];
  int parent[MAX_MAP_NODES];
  bool seen[MAX_MAP_NODES] = {false};

  int qHead = 0, qTail = 0;
  queue[qTail++] = startId;
  parent[startId] = -1;
  seen[startId] = true;

  while (qHead < qTail)
  {
    int current = queue[qHead++];
    if (current == targetId)
      break;

    for (uint8_t d = 0; d < 4; ++d)
    {
      int neighbor = nodes[current].neighbor[d];
      if (neighbor >= 0 && !seen[neighbor])
      {
        seen[neighbor] = true;
        parent[neighbor] = current;
        queue[qTail++] = neighbor;
      }
    }
  }

  if (!seen[targetId])
    return 0;

  int index = 0;
  for (int node = targetId; node >= 0 && index < pathSize; node = parent[node])
  {
    path[index++] = node;
  }

  for (int i = 0; i < index / 2; ++i)
  {
    int temp = path[i];
    path[i] = path[index - 1 - i];
    path[index - 1 - i] = temp;
  }

  return index;
}

Direction MazeMapper::directionToNeighbor(int fromId, int toId) const
{
  if (fromId < 0 || fromId >= nodeCount || toId < 0 || toId >= nodeCount)
    return NORTH;

  const MazeNode &from = nodes[fromId];
  const MazeNode &to = nodes[toId];
  if (to.x == from.x && to.y == from.y - 1)
    return NORTH;
  if (to.x == from.x + 1 && to.y == from.y)
    return EAST;
  if (to.x == from.x && to.y == from.y + 1)
    return SOUTH;
  if (to.x == from.x - 1 && to.y == from.y)
    return WEST;

  return NORTH;
}

int MazeMapper::getNodeCount() const
{
  return nodeCount;
}

Direction MazeMapper::rotateRight(Direction d)
{
  return static_cast<Direction>((d + 1) % 4);
}

Direction MazeMapper::rotateLeft(Direction d)
{
  return static_cast<Direction>((d + 3) % 4);
}

RobotPose MazeMapper::moveForward(const RobotPose &pose)
{
  RobotPose next = pose;
  switch (pose.heading)
  {
  case NORTH:
    next.y -= 1;
    break;
  case EAST:
    next.x += 1;
    break;
  case SOUTH:
    next.y += 1;
    break;
  case WEST:
    next.x -= 1;
    break;
  }
  return next;
}

void MazeMapper::linkNeighbors(int a, int b, Direction dir)
{
  nodes[a].neighbor[dir] = b;
  nodes[b].neighbor[(dir + 2) % 4] = a;
}
