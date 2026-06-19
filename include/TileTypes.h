#pragma once

#include <stdint.h>

enum TileColor {
  TILE_UNKNOWN = 0,
  TILE_BLACK,
  TILE_BLUE,
  TILE_SILVER,
};

inline bool isBlockingTile(TileColor color)
{
  return color == TILE_BLACK;
}
