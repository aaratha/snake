#include "entity.hpp"

void SpawnRopeEnemy(EntityStore &entities, RopeStore &ropes,
                    Vec2 from, Vec2 to, int segments) {
    RopeId id = SpawnRope(ropes, from, to, segments);
    ropes.freePhysics[static_cast<size_t>(id)] = true;
    entities.entities.push_back({EntityType::RopeEnemy, id});
}
