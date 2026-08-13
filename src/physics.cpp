#include "physics.hpp"
#include "scene.hpp"
#include "spatial_hash.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>

static constexpr float FRICTION        = 0.99f;
static constexpr float COMPLIANCE      = 1e-7f;
static constexpr int   SUBSTEPS        = 8;
static constexpr int   PIN_DAMP_RADIUS  = 3;
static constexpr float PIN_DAMP         = 0.50f; // max fraction of velocity removed per substep at pin boundary
static constexpr int   FREE_DAMP_RADIUS = 6;   // segments from free end to damp
static constexpr float FREE_DAMP        = 0.1f; // fraction of velocity removed per substep
static constexpr float ROPE_RADIUS      = 6.0f;
static constexpr int   MIN_COLL_GAP    = 5;
static constexpr float MIN_BEND_ANGLE   = 90.0f; // degrees — max allowed bend between segments

consteval float cos_deg(float deg) {
  constexpr float PI = std::numbers::pi_v<float>;
  float x = deg * (PI / 180.0f);
  while (x >  PI) x -= 2.0f * PI;
  while (x < -PI) x += 2.0f * PI;
  float s = 1.0f, t = 1.0f;
  for (int n = 1; n <= 15; ++n)
    t *= -(x * x) / float((2*n - 1) * (2*n)), s += t;
  return s;
}

static constexpr float MIN_BEND_COS     = cos_deg(MIN_BEND_ANGLE);
static constexpr float MAX_BODY_VELOCITY = 30.0f; // world units per substep

void IntegrateRopePositions(RopeStore &ropes, float /*dt*/) {
  for (size_t r = 0; r < ropes.ropeStart.size(); ++r) {
    int    segs = ropes.segCount[r];
    size_t base = r * MAX_SEGMENTS_PER_ROPE;
    for (int i = 0; i < segs; ++i) {
      size_t idx = base + i;
      if (ropes.invMass[idx] == 0.0f) continue;
      Vec2 vel = {ropes.c_pos[idx].x - ropes.p_pos[idx].x,
                  ropes.c_pos[idx].y - ropes.p_pos[idx].y};
      float speed2 = vel.x*vel.x + vel.y*vel.y;
      if (speed2 > MAX_BODY_VELOCITY * MAX_BODY_VELOCITY) {
        float scale = MAX_BODY_VELOCITY / std::sqrt(speed2);
        vel.x *= scale;
        vel.y *= scale;
      }
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

static float clamp01(float v) { return v < 0.0f ? 0.0f : v > 1.0f ? 1.0f : v; }

// Ericson §5.1.9 — closest parametric points s, t on segments P0P1 and Q0Q1
static void ClosestSegmentPoints(Vec2 p0, Vec2 p1, Vec2 q0, Vec2 q1,
                                  float &s, float &t) {
  float d1x = p1.x - p0.x, d1y = p1.y - p0.y;
  float d2x = q1.x - q0.x, d2y = q1.y - q0.y;
  float rx  = p0.x - q0.x, ry  = p0.y - q0.y;
  float a   = d1x*d1x + d1y*d1y;
  float e   = d2x*d2x + d2y*d2y;
  float f   = d2x*rx  + d2y*ry;

  if (a < 1e-10f && e < 1e-10f) { s = t = 0.0f; return; }
  if (a < 1e-10f) { s = 0.0f; t = clamp01(f / e); return; }

  float c = d1x*rx + d1y*ry;
  if (e < 1e-10f) { s = clamp01(-c / a); t = 0.0f; return; }

  float b     = d1x*d2x + d1y*d2y;
  float denom = a*e - b*b;
  s = denom > 1e-10f ? clamp01((b*f - c*e) / denom) : 0.0f;
  t = (b*s + f) / e;
  if      (t < 0.0f) { t = 0.0f; s = clamp01(-c / a); }
  else if (t > 1.0f) { t = 1.0f; s = clamp01((b - c) / a); }
}

static void SolveAngleConstraints(RopeStore &ropes) {
  for (size_t r = 0; r < ropes.ropeStart.size(); ++r) {
    int    segs = ropes.segCount[r];
    size_t base = r * MAX_SEGMENTS_PER_ROPE;

    for (int i = 0; i < segs - 2; ++i) {
      size_t ai = base + i, bi = base + i + 1, ci = base + i + 2;

      Vec2 e0 = { ropes.c_pos[bi].x - ropes.c_pos[ai].x,
                  ropes.c_pos[bi].y - ropes.c_pos[ai].y };
      Vec2 e1 = { ropes.c_pos[ci].x - ropes.c_pos[bi].x,
                  ropes.c_pos[ci].y - ropes.c_pos[bi].y };

      float len0 = std::sqrt(e0.x*e0.x + e0.y*e0.y);
      float len1 = std::sqrt(e1.x*e1.x + e1.y*e1.y);
      if (len0 < 1e-6f || len1 < 1e-6f) continue;

      Vec2  e0h     = { e0.x / len0, e0.y / len0 };
      float proj    = e1.x*e0h.x + e1.y*e0h.y; // forward component of e1 along e0
      float minProj = len1 * MIN_BEND_COS;
      if (proj >= minProj) continue;

      // directly correct the forward deficit — works even at full fold-back (d = -1)
      float deficit = minProj - proj;
      float wb = ropes.invMass[bi], wc = ropes.invMass[ci];
      float wsum = wb + wc;
      if (wsum < 1e-10f) continue;

      float lam = deficit / wsum;
      ropes.c_pos[bi].x -= wb * lam * e0h.x;
      ropes.c_pos[bi].y -= wb * lam * e0h.y;
      ropes.c_pos[ci].x += wc * lam * e0h.x;
      ropes.c_pos[ci].y += wc * lam * e0h.y;
    }
  }
}


static void SolveRopeSelfCollisions(RopeStore &ropes) {
  const float minDist  = 2.0f * ROPE_RADIUS;
  const float minDist2 = minDist * minDist;
  const float CELL     = minDist;

  static SpatialHash sh;
  static std::vector<int> cands;
  static uint32_t visited[MAX_SEGMENTS_PER_ROPE];
  static uint32_t gen = 0;

  for (size_t r = 0; r < ropes.ropeStart.size(); ++r) {
    int    segs = ropes.segCount[r];
    size_t base = r * MAX_SEGMENTS_PER_ROPE;

    auto predPos = [&](size_t idx) -> Vec2 {
      return {2*ropes.c_pos[idx].x - ropes.p_pos[idx].x,
              2*ropes.c_pos[idx].y - ropes.p_pos[idx].y};
    };

    // Build: use union of current and predicted AABBs so fast-moving segments aren't missed
    sh.clear();
    for (int i = 0; i < segs - 1; ++i) {
      size_t ia = base+i, ib = base+i+1;
      Vec2 pa = predPos(ia), pb = predPos(ib);
      float x0 = std::min({ropes.c_pos[ia].x, ropes.c_pos[ib].x, pa.x, pb.x});
      float x1 = std::max({ropes.c_pos[ia].x, ropes.c_pos[ib].x, pa.x, pb.x});
      float y0 = std::min({ropes.c_pos[ia].y, ropes.c_pos[ib].y, pa.y, pb.y});
      float y1 = std::max({ropes.c_pos[ia].y, ropes.c_pos[ib].y, pa.y, pb.y});
      for (int x = (int)std::floor(x0/CELL); x <= (int)std::floor(x1/CELL); x++)
        for (int y = (int)std::floor(y0/CELL); y <= (int)std::floor(y1/CELL); y++)
          sh.insert(x, y, i);
    }

    for (int i = 0; i < segs - 1; ++i) {
      size_t ia = base+i, ib = base+i+1;
      Vec2 pa = predPos(ia), pb = predPos(ib);
      float x0 = std::min({ropes.c_pos[ia].x, ropes.c_pos[ib].x, pa.x, pb.x});
      float x1 = std::max({ropes.c_pos[ia].x, ropes.c_pos[ib].x, pa.x, pb.x});
      float y0 = std::min({ropes.c_pos[ia].y, ropes.c_pos[ib].y, pa.y, pb.y});
      float y1 = std::max({ropes.c_pos[ia].y, ropes.c_pos[ib].y, pa.y, pb.y});

      cands.clear();
      ++gen;
      for (int x = (int)std::floor(x0/CELL)-1; x <= (int)std::floor(x1/CELL)+1; x++)
        for (int y = (int)std::floor(y0/CELL)-1; y <= (int)std::floor(y1/CELL)+1; y++)
          sh.query(x, y, [&](int j) {
            if (j < i + MIN_COLL_GAP) return;
            if (visited[j] == gen)     return;
            visited[j] = gen;
            cands.push_back(j);
          });

      for (int j : cands) {
        if (j >= segs - 1) continue;
        size_t ja = base+j, jb = base+j+1;

        // Narrowphase on predicted positions
        Vec2 p_ia = predPos(ia), p_ib = predPos(ib);
        Vec2 p_ja = predPos(ja), p_jb = predPos(jb);

        float s, t;
        ClosestSegmentPoints(p_ia, p_ib, p_ja, p_jb, s, t);

        float c1x = p_ia.x + s*(p_ib.x - p_ia.x), c1y = p_ia.y + s*(p_ib.y - p_ia.y);
        float c2x = p_ja.x + t*(p_jb.x - p_ja.x), c2y = p_ja.y + t*(p_jb.y - p_ja.y);
        float dx = c2x - c1x, dy = c2y - c1y;
        float pred_dist2 = dx*dx + dy*dy;
        if (pred_dist2 >= minDist2) continue;

        float pred_dist = std::sqrt(pred_dist2);
        if (pred_dist < 1e-6f) continue;

        float nx = dx/pred_dist, ny = dy/pred_dist;

        float w_ia = ropes.invMass[ia], w_ib = ropes.invMass[ib];
        float w_ja = ropes.invMass[ja], w_jb = ropes.invMass[jb];
        float wEff = (1-s)*(1-s)*w_ia + s*s*w_ib
                   + (1-t)*(1-t)*w_ja + t*t*w_jb;
        if (wEff < 1e-10f) continue;

        // Closing velocity at predicted contact points (positive = approaching)
        auto vn = [&](size_t idx) {
          return (ropes.c_pos[idx].x - ropes.p_pos[idx].x)*nx
               + (ropes.c_pos[idx].y - ropes.p_pos[idx].y)*ny;
        };
        float v_rel = (1-s)*vn(ia) + s*vn(ib) - (1-t)*vn(ja) - t*vn(jb);

        // Velocity correction: zero when separating (false positive filter)
        float vel_lambda = std::max(0.0f, v_rel) / wEff;

        // Position correction: push out current overlap immediately
        float cur_c1x = ropes.c_pos[ia].x + s*(ropes.c_pos[ib].x - ropes.c_pos[ia].x);
        float cur_c1y = ropes.c_pos[ia].y + s*(ropes.c_pos[ib].y - ropes.c_pos[ia].y);
        float cur_c2x = ropes.c_pos[ja].x + t*(ropes.c_pos[jb].x - ropes.c_pos[ja].x);
        float cur_c2y = ropes.c_pos[ja].y + t*(ropes.c_pos[jb].y - ropes.c_pos[ja].y);
        float cur_dx = cur_c2x - cur_c1x, cur_dy = cur_c2y - cur_c1y;
        float cur_dist2 = cur_dx*cur_dx + cur_dy*cur_dy;
        float pos_lambda = cur_dist2 < minDist2
            ? (minDist - std::sqrt(cur_dist2)) / wEff
            : 0.0f;

        if (pos_lambda > 0.0f) {
          ropes.c_pos[ia].x -= w_ia*(1-s)*pos_lambda*nx;
          ropes.c_pos[ia].y -= w_ia*(1-s)*pos_lambda*ny;
          ropes.c_pos[ib].x -= w_ib*s    *pos_lambda*nx;
          ropes.c_pos[ib].y -= w_ib*s    *pos_lambda*ny;
          ropes.c_pos[ja].x += w_ja*(1-t)*pos_lambda*nx;
          ropes.c_pos[ja].y += w_ja*(1-t)*pos_lambda*ny;
          ropes.c_pos[jb].x += w_jb*t    *pos_lambda*nx;
          ropes.c_pos[jb].y += w_jb*t    *pos_lambda*ny;
        }

        if (vel_lambda > 0.0f) {
          ropes.p_pos[ia].x += w_ia*(1-s)*vel_lambda*nx;
          ropes.p_pos[ia].y += w_ia*(1-s)*vel_lambda*ny;
          ropes.p_pos[ib].x += w_ib*s    *vel_lambda*nx;
          ropes.p_pos[ib].y += w_ib*s    *vel_lambda*ny;
          ropes.p_pos[ja].x -= w_ja*(1-t)*vel_lambda*nx;
          ropes.p_pos[ja].y -= w_ja*(1-t)*vel_lambda*ny;
          ropes.p_pos[jb].x -= w_jb*t    *vel_lambda*nx;
          ropes.p_pos[jb].y -= w_jb*t    *vel_lambda*ny;
        }
      }
    }
  }
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
    SolveAngleConstraints(ropes);
    SolveRopeSelfCollisions(ropes);

    for (size_t r = 0; r < ropes.ropeStart.size(); ++r) {
      int    segs = ropes.segCount[r];
      size_t base = r * MAX_SEGMENTS_PER_ROPE;

      // damp velocity near pinned points; pin itself is fully zeroed, neighbours get a
      // small per-substep fraction that falls off linearly — keeps the gradient real
      // across all 8 substeps rather than collapsing to zero at every boundary
      for (int i = 0; i < segs; ++i) {
        if (ropes.invMass[base + i] != 0.0f) continue;
        ropes.p_pos[base + i] = ropes.c_pos[base + i]; // pin: full zero
        int lo = std::max(0, i - PIN_DAMP_RADIUS);
        int hi = std::min(segs - 1, i + PIN_DAMP_RADIUS);
        for (int j = lo; j <= hi; ++j) {
          if (j == i) continue;
          float t = PIN_DAMP * (1.0f - (float)std::abs(j - i) / (PIN_DAMP_RADIUS + 1.0f));
          ropes.p_pos[base+j].x += (ropes.c_pos[base+j].x - ropes.p_pos[base+j].x) * t;
          ropes.p_pos[base+j].y += (ropes.c_pos[base+j].y - ropes.p_pos[base+j].y) * t;
        }
      }

      // bleed velocity near each endpoint — apply even when pinned so fast turns
      // don't leave segments 4-6 completely undamped on the dragged-head side
      auto dampEnd = [&](int endpoint) {
        int lo = (endpoint == 0) ? 0 : std::max(0, segs - 1 - FREE_DAMP_RADIUS);
        int hi = (endpoint == 0) ? std::min(segs - 1, FREE_DAMP_RADIUS) : segs - 1;
        for (int j = lo; j <= hi; ++j) {
          ropes.p_pos[base+j].x += (ropes.c_pos[base+j].x - ropes.p_pos[base+j].x) * FREE_DAMP;
          ropes.p_pos[base+j].y += (ropes.c_pos[base+j].y - ropes.p_pos[base+j].y) * FREE_DAMP;
        }
      };
      dampEnd(0);
      dampEnd(segs - 1);
    }
  }
}
