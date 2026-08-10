#pragma once

#include <vector>
#include <cstdint>

#include "types.hpp"

constexpr uint32_t MAX_ROPES = 10;
constexpr uint32_t MAX_SEGMENTS_PER_ROPE = 128;

struct RopeConstraint {
    uint32_t p1, p2; // local indices within a rope
    float restLength;
    float lambda = 0.0f; // accumulated XPBD multiplier, reset each substep
};

struct RopeStore {
    std::vector<Vec2> c_pos;
    std::vector<Vec2> p_pos;
    std::vector<float> invMass;
    std::vector<RopeConstraint> constraints;
    std::vector<uint32_t> ropeStart; // ropeStart[r] == r (rope index)
    std::vector<int> segCount;       // active segment count per rope
};

void InitRopeStore(RopeStore &store);
RopeId SpawnRope(RopeStore &store, Vec2 origin, Vec2 target, int segments);
void DestroyRope(RopeStore &store, RopeId rope);
