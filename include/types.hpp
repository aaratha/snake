#pragma once

#include <cstdint>

struct Vec2 {
  float x, y;
};

enum class RopeId : uint32_t {};

struct EnemyID {
  int id;
};

struct Camera {
  Vec2 pos;   // world-space point at the center of the screen
  float zoom; // screen pixels per world unit
};
