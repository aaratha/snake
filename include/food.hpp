#pragma once

#include <vector>

#include "types.hpp"


struct Food {
  Vec2 position;
  float quantity;
  float radius;
  bool isEaten;
};

struct FoodStore {
  std::vector<Food> foods;
};

void InitFoodStore(FoodStore &foodStore, int count);
void SpawnFood(FoodStore &foodStore, int count);
void DestroyFood(FoodStore &foodStore);

void removeFood(FoodStore &foodStore, size_t index);
