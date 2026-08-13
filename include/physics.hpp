#pragma once

#include "rope.hpp"

static constexpr float MAX_VELOCITY = 0.3f; // world units per substep

struct Scene;

void SolveRopeConstraints(RopeStore &ropes, float dt);
void IntegrateRopePositions(RopeStore &ropes, float dt);
void StepPhysics(Scene &scene, float dt);
