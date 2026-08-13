#pragma once

#include "rope.hpp"
#include "types.hpp"
#include <vector>

enum class EntityType {
    RopeEnemy,
};

struct Entity {
    EntityType type;
    RopeId     ropeId;
};

struct EntityStore {
    std::vector<Entity> entities;
};

// Spawns a fully free rope entity — no pinned nodes, no AI.
// Motion emerges from self-collision physics.
void SpawnRopeEnemy(EntityStore &entities, RopeStore &ropes,
                    Vec2 from, Vec2 to, int segments);
