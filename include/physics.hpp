#pragma once

#include "rope.hpp"

struct Scene;

void SolveRopeConstraints(RopeStore &ropes, float dt);
void IntegrateRopePositions(RopeStore &ropes, float dt);
void StepPhysics(Scene &scene, float dt);
