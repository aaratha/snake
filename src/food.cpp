#include "food.hpp"
#include "scene.hpp"
#include "rope.hpp"

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

void CheckFoodCollisions(Scene &scene, bool &growByLength) {
    if (scene.ropes.ropeStart.empty()) return;
    Vec2 head = scene.ropes.c_pos[0];
    for (auto &food : scene.foodStore.foods) {
        if (food.isEaten) continue;
        float dx = head.x - food.position.x, dy = head.y - food.position.y;
        if (dx*dx + dy*dy >= food.radius * food.radius) continue;
        food.isEaten = true;
        int  segs  = scene.ropes.segCount[0];
        bool atMax = segs >= (int)MAX_SEGMENTS_PER_ROPE;
        if (!growByLength && !atMax) {
            addRopeSegment(scene.ropes, static_cast<RopeId>(0));
        } else {
            float restLen   = scene.ropes.constraints[0].restLength;
            float addPerSeg = restLen / (segs - 1);
            for (int i = 0; i < segs - 1; ++i)
                scene.ropes.constraints[i].restLength += addPerSeg;
        }
        growByLength = !growByLength;
    }
}
