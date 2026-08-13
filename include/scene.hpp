#pragma once

#include "rope.hpp"
#include "food.hpp"
#include "rigidbody.hpp"

struct Scene {
  RopeStore      ropes;
  FoodStore      foodStore;
  RigidBodyStore rigidBodies;
};
