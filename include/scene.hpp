#pragma once

#include "entity.hpp"
#include "food.hpp"
#include "rigidbody.hpp"
#include "rope.hpp"

struct Scene {
  RopeStore      ropes;
  FoodStore      foodStore;
  RigidBodyStore rigidBodies;
  EntityStore    entities;
};
