#include "food.hpp"

#include <cstdlib>


void InitFoodStore(FoodStore &foodStore, int count) {
    foodStore.foods.reserve(count);
    SpawnFood(foodStore, count);
}

void SpawnFood(FoodStore &foodStore, int count) {
    for (int i = 0; i < count; ++i) {
        Food food;
        food.position = {static_cast<float>(std::rand() % 1000), static_cast<float>(std::rand() % 1000)};
        food.quantity = 1.0f;
        food.radius = 10.0f;
        food.isEaten = false;
        foodStore.foods.push_back(food);
    }
}

void DestroyFood(FoodStore &foodStore) {
    foodStore.foods.clear();
}

void removeFood(FoodStore &foodStore, size_t index) {
    if (index < foodStore.foods.size()) {
        foodStore.foods.erase(foodStore.foods.begin() + index);
    }
}
