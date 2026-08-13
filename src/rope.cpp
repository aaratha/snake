#include "rope.hpp"
#include <cmath>

void InitRopeStore(RopeStore &store) {
  store.c_pos.resize(MAX_ROPES * MAX_SEGMENTS_PER_ROPE, Vec2{});
  store.p_pos.resize(MAX_ROPES * MAX_SEGMENTS_PER_ROPE, Vec2{});
  store.invMass.resize(MAX_ROPES * MAX_SEGMENTS_PER_ROPE, 0.0f);
  store.constraints.resize(MAX_ROPES * (MAX_SEGMENTS_PER_ROPE - 1));
  store.ropeStart.reserve(MAX_ROPES);
  store.segCount.reserve(MAX_ROPES);
}

RopeId SpawnRope(RopeStore &store, Vec2 origin, Vec2 target, int segments) {
  RopeId ropeId = static_cast<RopeId>(store.ropeStart.size());
  store.ropeStart.push_back(static_cast<uint32_t>(ropeId));
  store.segCount.push_back(segments);

  size_t base = static_cast<size_t>(ropeId) * MAX_SEGMENTS_PER_ROPE;
  for (int i = 0; i < segments; ++i) {
    float t = static_cast<float>(i) / (segments - 1);
    Vec2 pos = {origin.x * (1 - t) + target.x * t,
                origin.y * (1 - t) + target.y * t};
    store.c_pos[base + i] = pos;
    store.p_pos[base + i] = pos;
    store.invMass[base + i] = 1.0f;

    if (i < segments - 1) {
      size_t cbase = static_cast<size_t>(ropeId) * (MAX_SEGMENTS_PER_ROPE - 1);
      float dx = target.x - origin.x;
      float dy = target.y - origin.y;
      float segLen = std::sqrt(dx * dx + dy * dy) / (segments - 1);
      store.constraints[cbase + i] = RopeConstraint{
          static_cast<uint32_t>(i), static_cast<uint32_t>(i + 1), segLen};
    }
  }
  return ropeId;
}

void DestroyRope(RopeStore &store, RopeId rope) {
  size_t base = static_cast<size_t>(rope) * MAX_SEGMENTS_PER_ROPE;
  for (size_t i = 0; i < MAX_SEGMENTS_PER_ROPE; ++i) {
    store.c_pos[base + i] = Vec2{};
    store.p_pos[base + i] = Vec2{};
    store.invMass[base + i] = 0.0f;
  }
  size_t cbase = static_cast<size_t>(rope) * (MAX_SEGMENTS_PER_ROPE - 1);
  for (size_t i = 0; i < MAX_SEGMENTS_PER_ROPE - 1; ++i) {
    store.constraints[cbase + i] = RopeConstraint{0, 0, 0.0f};
  }
}

void addRopeSegment(RopeStore &store, RopeId rope) {
  size_t base = static_cast<size_t>(rope) * MAX_SEGMENTS_PER_ROPE;
  int segs = store.segCount[static_cast<size_t>(rope)];
  if (segs >= static_cast<int>(MAX_SEGMENTS_PER_ROPE)) return;

  Vec2 lastPos = store.c_pos[base + segs - 1];
  Vec2 secondLastPos = store.c_pos[base + segs - 2];
  float dx = lastPos.x - secondLastPos.x;
  float dy = lastPos.y - secondLastPos.y;
  float segLen = std::sqrt(dx * dx + dy * dy);

  store.c_pos[base + segs] = lastPos;
  store.p_pos[base + segs] = lastPos;
  store.invMass[base + segs] = 1.0f;

  size_t cbase = static_cast<size_t>(rope) * (MAX_SEGMENTS_PER_ROPE - 1);
  store.constraints[cbase + segs - 1] = RopeConstraint{
      static_cast<uint32_t>(segs - 1), static_cast<uint32_t>(segs), segLen};

  store.segCount[static_cast<size_t>(rope)]++;
}
