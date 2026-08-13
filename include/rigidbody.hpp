#pragma once
#include <vector>
#include "types.hpp"

enum class ShapeType : uint8_t { Circle, Rectangle };

struct RigidBody {
    Vec2      pos         = {};
    Vec2      vel         = {};
    float     angle       = 0.0f;
    float     angVel      = 0.0f;
    float     invMass     = 1.0f;
    float     invInertia  = 1.0f;
    float     restitution = 0.6f;
    ShapeType shape       = ShapeType::Circle;
    float     radius      = 0.0f;  // Circle
    Vec2      halfExtents = {};    // Rectangle
};

struct RigidBodyStore {
    std::vector<RigidBody> bodies;
};

RigidBody MakeCircle   (Vec2 pos, Vec2 vel, float radius,
                        float density = 0.0001f);
RigidBody MakeRectangle(Vec2 pos, Vec2 vel, float w, float h,
                        float angle = 0.0f, float density = 0.0001f);

void StepRigidBodies(RigidBodyStore &store, float dt);

struct RopeStore;
void ResolveRopeRigidBodyCollisions(RopeStore &ropes, RigidBodyStore &store);
