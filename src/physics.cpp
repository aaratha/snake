#include "physics.hpp"
#include "scene.hpp"
#include <cmath>

static constexpr float FRICTION        = 0.99f;
static constexpr float COMPLIANCE      = 1e-7f;
static constexpr int   SUBSTEPS        = 8;
static constexpr int   PIN_DAMP_RADIUS = 3; // segments near pin that get velocity zeroed

void IntegrateRopePositions(RopeStore &ropes, float /*dt*/) {
  for (size_t r = 0; r < ropes.ropeStart.size(); ++r) {
    int    segs = ropes.segCount[r];
    size_t base = r * MAX_SEGMENTS_PER_ROPE;
    for (int i = 0; i < segs; ++i) {
      size_t idx = base + i;
      if (ropes.invMass[idx] == 0.0f) continue;
      Vec2 vel = {ropes.c_pos[idx].x - ropes.p_pos[idx].x,
                  ropes.c_pos[idx].y - ropes.p_pos[idx].y};
      ropes.p_pos[idx] = ropes.c_pos[idx];
      ropes.c_pos[idx].x += vel.x * FRICTION;
      ropes.c_pos[idx].y += vel.y * FRICTION;
    }
  }
}

static void SolveConstraint(RopeStore &ropes, size_t ia, size_t ib,
                             RopeConstraint &c, float alpha) {
  Vec2 &a = ropes.c_pos[ia];
  Vec2 &b = ropes.c_pos[ib];

  float dx   = b.x - a.x;
  float dy   = b.y - a.y;
  float dist = std::sqrt(dx * dx + dy * dy);
  if (dist < 1e-6f) return;

  float nx           = dx / dist;
  float ny           = dy / dist;
  float C            = dist - c.restLength;
  float w1           = ropes.invMass[ia];
  float w2           = ropes.invMass[ib];
  float delta_lambda = (-C - alpha * c.lambda) / (w1 + w2 + alpha);
  c.lambda += delta_lambda;

  a.x -= w1 * delta_lambda * nx;
  a.y -= w1 * delta_lambda * ny;
  b.x += w2 * delta_lambda * nx;
  b.y += w2 * delta_lambda * ny;
}

void SolveRopeConstraints(RopeStore &ropes, float dt) {
  const float alpha = COMPLIANCE / (dt * dt);
  for (size_t r = 0; r < ropes.ropeStart.size(); ++r) {
    int    segs  = ropes.segCount[r];
    size_t base  = r * MAX_SEGMENTS_PER_ROPE;
    size_t cbase = r * (MAX_SEGMENTS_PER_ROPE - 1);

    // solve from the pinned end toward the free end
    bool tailPinned = ropes.invMass[base + segs - 1] == 0.0f;
    if (tailPinned) {
      for (int i = segs - 2; i >= 0; --i)
        SolveConstraint(ropes, base + ropes.constraints[cbase + i].p1,
                               base + ropes.constraints[cbase + i].p2,
                               ropes.constraints[cbase + i], alpha);
    } else {
      for (int i = 0; i < segs - 1; ++i)
        SolveConstraint(ropes, base + ropes.constraints[cbase + i].p1,
                               base + ropes.constraints[cbase + i].p2,
                               ropes.constraints[cbase + i], alpha);
    }
  }
}

void StepPhysics(Scene &scene, float dt) {
  RopeStore &ropes   = scene.ropes;
  const float sub_dt = dt / SUBSTEPS;

  for (int s = 0; s < SUBSTEPS; ++s) {
    // reset lambdas at the start of each substep
    for (size_t r = 0; r < ropes.ropeStart.size(); ++r) {
      int    segs  = ropes.segCount[r];
      size_t cbase = r * (MAX_SEGMENTS_PER_ROPE - 1);
      for (int i = 0; i < segs - 1; ++i)
        ropes.constraints[cbase + i].lambda = 0.0f;
    }
    IntegrateRopePositions(ropes, sub_dt);
    SolveRopeConstraints(ropes, sub_dt);

    // zero velocity near pins so oscillation can't build up there
    for (size_t r = 0; r < ropes.ropeStart.size(); ++r) {
      int    segs = ropes.segCount[r];
      size_t base = r * MAX_SEGMENTS_PER_ROPE;
      for (int i = 0; i < segs; ++i) {
        if (ropes.invMass[base + i] != 0.0f) continue;
        int lo = std::max(0, i - PIN_DAMP_RADIUS);
        int hi = std::min(segs - 1, i + PIN_DAMP_RADIUS);
        for (int j = lo; j <= hi; ++j)
          ropes.p_pos[base + j] = ropes.c_pos[base + j];
      }
    }
  }
}
