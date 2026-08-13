#include "rigidbody.hpp"
#include "rope.hpp"
#include "spatial_hash.hpp"
#include <cmath>
#include <algorithm>
#include <numbers>

static constexpr float LINEAR_DAMP  = 0.998f;
static constexpr float ANGULAR_DAMP = 0.993f;
static constexpr float BAUMGARTE    = 0.4f;
static constexpr float SLOP         = 0.5f;

// ---- math helpers ----

static float cross2(Vec2 a, Vec2 b) { return a.x*b.y - a.y*b.x; }
static float dot2  (Vec2 a, Vec2 b) { return a.x*b.x + a.y*b.y; }

static void applyImpulse(RigidBody &b, Vec2 j, Vec2 r) {
    b.vel.x  += b.invMass    * j.x;
    b.vel.y  += b.invMass    * j.y;
    b.angVel += b.invInertia * cross2(r, j);
}

static Vec2 velAtPoint(const RigidBody &b, Vec2 r) {
    return {b.vel.x - b.angVel * r.y, b.vel.y + b.angVel * r.x};
}

static void getRectCorners(const RigidBody &rect, Vec2 corners[4]) {
    float c = std::cos(rect.angle), s = std::sin(rect.angle);
    Vec2 ax = { c * rect.halfExtents.x,  s * rect.halfExtents.x};
    Vec2 ay = {-s * rect.halfExtents.y,  c * rect.halfExtents.y};
    corners[0] = {rect.pos.x + ax.x + ay.x, rect.pos.y + ax.y + ay.y};
    corners[1] = {rect.pos.x - ax.x + ay.x, rect.pos.y - ax.y + ay.y};
    corners[2] = {rect.pos.x - ax.x - ay.x, rect.pos.y - ax.y - ay.y};
    corners[3] = {rect.pos.x + ax.x - ay.x, rect.pos.y + ax.y - ay.y};
}

static bool pointInRect(Vec2 p, const RigidBody &rect) {
    float dx = p.x - rect.pos.x, dy = p.y - rect.pos.y;
    float c = std::cos(-rect.angle), s = std::sin(-rect.angle);
    float lx = c*dx - s*dy, ly = s*dx + c*dy;
    return std::abs(lx) < rect.halfExtents.x && std::abs(ly) < rect.halfExtents.y;
}

// ---- contact detection ----

struct Contact {
    bool  valid  = false;
    Vec2  normal = {};  // points from a to b (push a along -normal)
    float depth  = 0.0f;
    Vec2  point  = {};
};

static void resolveContact(RigidBody &a, RigidBody &b, const Contact &c) {
    if (!c.valid) return;

    Vec2  ra   = {c.point.x - a.pos.x, c.point.y - a.pos.y};
    Vec2  rb   = {c.point.x - b.pos.x, c.point.y - b.pos.y};
    Vec2  vA   = velAtPoint(a, ra);
    Vec2  vB   = velAtPoint(b, rb);
    float vn   = dot2({vB.x - vA.x, vB.y - vA.y}, c.normal);
    if (vn > 0.0f) return;

    float raN  = cross2(ra, c.normal);
    float rbN  = cross2(rb, c.normal);
    float denom = a.invMass + b.invMass
                + raN*raN*a.invInertia + rbN*rbN*b.invInertia;
    if (denom < 1e-10f) return;

    float e = std::min(a.restitution, b.restitution);
    float j = -(1.0f + e) * vn / denom;

    applyImpulse(a, {-j * c.normal.x, -j * c.normal.y}, ra);
    applyImpulse(b, { j * c.normal.x,  j * c.normal.y}, rb);

    float totalInvMass = a.invMass + b.invMass;
    if (totalInvMass < 1e-10f) return;
    float corr = std::max(c.depth - SLOP, 0.0f) * BAUMGARTE / totalInvMass;
    a.pos.x -= a.invMass * corr * c.normal.x;
    a.pos.y -= a.invMass * corr * c.normal.y;
    b.pos.x += b.invMass * corr * c.normal.x;
    b.pos.y += b.invMass * corr * c.normal.y;
}

static Contact contactCircleCircle(const RigidBody &a, const RigidBody &b) {
    float dx    = b.pos.x - a.pos.x, dy = b.pos.y - a.pos.y;
    float dist2 = dx*dx + dy*dy;
    float minD  = a.radius + b.radius;
    if (dist2 >= minD * minD) return {};
    float dist = std::sqrt(dist2);
    Vec2  n    = dist > 1e-6f ? Vec2{dx/dist, dy/dist} : Vec2{1.0f, 0.0f};
    return {true, n, minD - dist, {a.pos.x + n.x*a.radius, a.pos.y + n.y*a.radius}};
}

// a = circle, b = rect; normal points from a to b
static Contact contactCircleRect(const RigidBody &circle, const RigidBody &rect) {
    float ca = std::cos(-rect.angle), sa = std::sin(-rect.angle);
    float dx = circle.pos.x - rect.pos.x, dy = circle.pos.y - rect.pos.y;
    float lx = ca*dx - sa*dy, ly = sa*dx + ca*dy;

    float cx = std::clamp(lx, -rect.halfExtents.x, rect.halfExtents.x);
    float cy = std::clamp(ly, -rect.halfExtents.y, rect.halfExtents.y);
    float ex = lx - cx, ey = ly - cy;
    float dist2 = ex*ex + ey*ey;
    if (dist2 >= circle.radius * circle.radius) return {};

    float cb = std::cos(rect.angle), sb = std::sin(rect.angle);
    Vec2  ln, contactLocal;
    float depth, dist = std::sqrt(dist2);

    if (dist < 1e-6f) {
        // Circle centre inside rectangle — push out through nearest face
        float ox = rect.halfExtents.x - std::abs(lx);
        float oy = rect.halfExtents.y - std::abs(ly);
        if (ox < oy) {
            float sgn  = lx > 0.0f ? 1.0f : -1.0f;
            ln           = {sgn, 0.0f};
            contactLocal = {sgn * rect.halfExtents.x, ly};
            depth        = ox + circle.radius;
        } else {
            float sgn  = ly > 0.0f ? 1.0f : -1.0f;
            ln           = {0.0f, sgn};
            contactLocal = {lx, sgn * rect.halfExtents.y};
            depth        = oy + circle.radius;
        }
    } else {
        ln           = {ex / dist, ey / dist};  // local-space outward from rect surface
        contactLocal = {cx, cy};
        depth        = circle.radius - dist;
    }

    // ln points rect-surface → circle; negate for a(circle)→b(rect) convention
    Vec2 n  = {-(cb*ln.x - sb*ln.y), -(sb*ln.x + cb*ln.y)};
    Vec2 pt = {rect.pos.x + cb*contactLocal.x - sb*contactLocal.y,
               rect.pos.y + sb*contactLocal.x + cb*contactLocal.y};
    return {true, n, depth, pt};
}

static Contact contactRectRect(const RigidBody &a, const RigidBody &b) {
    Vec2 cornersA[4], cornersB[4];
    getRectCorners(a, cornersA);
    getRectCorners(b, cornersB);

    float ca = std::cos(a.angle), sa = std::sin(a.angle);
    float cb = std::cos(b.angle), sb = std::sin(b.angle);
    Vec2 axes[4] = {{ca, sa}, {-sa, ca}, {cb, sb}, {-sb, cb}};

    float minOverlap = 1e30f;
    Vec2  minAxis    = {};

    for (auto &axis : axes) {
        float minA = 1e30f, maxA = -1e30f, minB = 1e30f, maxB = -1e30f;
        for (int i = 0; i < 4; i++) {
            float dA = dot2(cornersA[i], axis), dB = dot2(cornersB[i], axis);
            if (dA < minA) minA = dA; if (dA > maxA) maxA = dA;
            if (dB < minB) minB = dB; if (dB > maxB) maxB = dB;
        }
        float overlap = std::min(maxA, maxB) - std::max(minA, minB);
        if (overlap <= 0.0f) return {};
        if (overlap < minOverlap) {
            minOverlap = overlap;
            minAxis    = axis;
            Vec2 d = {b.pos.x - a.pos.x, b.pos.y - a.pos.y};
            if (dot2(d, axis) < 0.0f) minAxis = {-axis.x, -axis.y};
        }
    }

    // Contact point: average of vertices from each body that penetrate the other
    Vec2 sum = {}; int count = 0;
    for (int i = 0; i < 4; i++) {
        if (pointInRect(cornersB[i], a)) { sum.x += cornersB[i].x; sum.y += cornersB[i].y; count++; }
        if (pointInRect(cornersA[i], b)) { sum.x += cornersA[i].x; sum.y += cornersA[i].y; count++; }
    }
    Vec2 pt = count > 0
        ? Vec2{sum.x / count, sum.y / count}
        : Vec2{(a.pos.x + b.pos.x) * 0.5f, (a.pos.y + b.pos.y) * 0.5f};

    return {true, minAxis, minOverlap, pt};
}

// ---- rope-rigidbody collision ----

void ResolveRopeRigidBodyCollisions(RopeStore &ropes, RigidBodyStore &store) {
    static constexpr float RB_ROPE_RADIUS = 6.0f;  // must match ROPE_RADIUS in physics.cpp
    static constexpr float CELL = RB_ROPE_RADIUS * 2.0f;

    // build hash of all rope nodes (global flat index) once per call
    static SpatialHash sh;
    sh.clear();
    for (size_t r = 0; r < ropes.ropeStart.size(); ++r) {
        int    segs = ropes.segCount[r];
        size_t base = r * MAX_SEGMENTS_PER_ROPE;
        for (int i = 0; i < segs; ++i) {
            int cx = (int)std::floor(ropes.c_pos[base + i].x / CELL);
            int cy = (int)std::floor(ropes.c_pos[base + i].y / CELL);
            sh.insert(cx, cy, (int)(base + i));
        }
    }

    for (auto &rb : store.bodies) {
        // AABB of the rigid body expanded by rope radius → query cell range
        float x0, x1, y0, y1;
        if (rb.shape == ShapeType::Circle) {
            float r = rb.radius + RB_ROPE_RADIUS;
            x0 = rb.pos.x - r; x1 = rb.pos.x + r;
            y0 = rb.pos.y - r; y1 = rb.pos.y + r;
        } else {
            Vec2 corners[4];
            getRectCorners(rb, corners);
            x0 = x1 = corners[0].x;
            y0 = y1 = corners[0].y;
            for (int k = 1; k < 4; ++k) {
                if (corners[k].x < x0) x0 = corners[k].x;
                if (corners[k].x > x1) x1 = corners[k].x;
                if (corners[k].y < y0) y0 = corners[k].y;
                if (corners[k].y > y1) y1 = corners[k].y;
            }
            x0 -= RB_ROPE_RADIUS; x1 += RB_ROPE_RADIUS;
            y0 -= RB_ROPE_RADIUS; y1 += RB_ROPE_RADIUS;
        }
        int cx0 = (int)std::floor(x0 / CELL);
        int cx1 = (int)std::floor(x1 / CELL);
        int cy0 = (int)std::floor(y0 / CELL);
        int cy1 = (int)std::floor(y1 / CELL);

        for (int qx = cx0; qx <= cx1; ++qx) {
            for (int qy = cy0; qy <= cy1; ++qy) {
                sh.query(qx, qy, [&](int idx) {
                    float wRope = ropes.invMass[idx];
                    Vec2  rp    = ropes.c_pos[idx];
                    Vec2  n     = {};
                    float depth = 0.0f;
                    Vec2  contactPt = {};
                    bool  valid = false;

                    if (rb.shape == ShapeType::Circle) {
                        float dx = rp.x - rb.pos.x, dy = rp.y - rb.pos.y;
                        float dist2 = dx*dx + dy*dy;
                        float minD  = RB_ROPE_RADIUS + rb.radius;
                        if (dist2 >= minD * minD) return;
                        float dist = std::sqrt(dist2);
                        n     = dist > 1e-6f ? Vec2{dx/dist, dy/dist} : Vec2{1.0f, 0.0f};
                        depth = minD - dist;
                        contactPt = {rb.pos.x + n.x*rb.radius, rb.pos.y + n.y*rb.radius};
                        valid = true;
                    } else {
                        float ca = std::cos(-rb.angle), sa = std::sin(-rb.angle);
                        float dx = rp.x - rb.pos.x, dy = rp.y - rb.pos.y;
                        float lx = ca*dx - sa*dy, ly = sa*dx + ca*dy;
                        float lcx = std::clamp(lx, -rb.halfExtents.x, rb.halfExtents.x);
                        float lcy = std::clamp(ly, -rb.halfExtents.y, rb.halfExtents.y);
                        float ex = lx - lcx, ey = ly - lcy;
                        float dist2 = ex*ex + ey*ey;
                        if (dist2 >= RB_ROPE_RADIUS * RB_ROPE_RADIUS) return;
                        float dist = std::sqrt(dist2);
                        float cb = std::cos(rb.angle), sb = std::sin(rb.angle);
                        Vec2 ln, localPt;
                        if (dist < 1e-6f) {
                            float ox = rb.halfExtents.x - std::abs(lx);
                            float oy = rb.halfExtents.y - std::abs(ly);
                            if (ox < oy) {
                                float sgn = lx > 0.0f ? 1.0f : -1.0f;
                                ln = {sgn, 0.0f};
                                localPt = {sgn * rb.halfExtents.x, ly};
                                depth = ox + RB_ROPE_RADIUS;
                            } else {
                                float sgn = ly > 0.0f ? 1.0f : -1.0f;
                                ln = {0.0f, sgn};
                                localPt = {lx, sgn * rb.halfExtents.y};
                                depth = oy + RB_ROPE_RADIUS;
                            }
                        } else {
                            ln      = {ex/dist, ey/dist};
                            localPt = {lcx, lcy};
                            depth   = RB_ROPE_RADIUS - dist;
                        }
                        n         = {cb*ln.x - sb*ln.y, sb*ln.x + cb*ln.y};
                        contactPt = {rb.pos.x + cb*localPt.x - sb*localPt.y,
                                     rb.pos.y + sb*localPt.x + cb*localPt.y};
                        valid = true;
                    }

                    if (!valid || depth <= 0.0f) return;

                    Vec2  r_vec = {contactPt.x - rb.pos.x, contactPt.y - rb.pos.y};
                    Vec2  rpVel = {ropes.c_pos[idx].x - ropes.p_pos[idx].x,
                                   ropes.c_pos[idx].y - ropes.p_pos[idx].y};
                    Vec2  rbVel = velAtPoint(rb, r_vec);
                    float vn    = dot2({rpVel.x - rbVel.x, rpVel.y - rbVel.y}, n);

                    if (vn < 0.0f) {
                        float raN   = cross2(r_vec, n);
                        float denom = wRope + rb.invMass + raN*raN*rb.invInertia;
                        if (denom > 1e-10f) {
                            float j = -(1.0f + rb.restitution) * vn / denom;
                            applyImpulse(rb, {-j * n.x, -j * n.y}, r_vec);
                            ropes.p_pos[idx].x -= wRope * j * n.x;
                            ropes.p_pos[idx].y -= wRope * j * n.y;
                        }
                    }

                    float totalInvMass = wRope + rb.invMass;
                    if (totalInvMass < 1e-10f) return;
                    float corr = std::max(depth - SLOP, 0.0f) * BAUMGARTE / totalInvMass;
                    ropes.c_pos[idx].x += wRope      * corr * n.x;
                    ropes.c_pos[idx].y += wRope      * corr * n.y;
                    rb.pos.x           -= rb.invMass * corr * n.x;
                    rb.pos.y           -= rb.invMass * corr * n.y;
                });
            }
        }
    }
}

// ---- factories ----

RigidBody MakeCircle(Vec2 pos, Vec2 vel, float radius, float density) {
    float mass    = density * std::numbers::pi_v<float> * radius * radius;
    float inertia = 0.5f * mass * radius * radius;
    RigidBody b;
    b.pos = pos; b.vel = vel;
    b.shape = ShapeType::Circle; b.radius = radius;
    b.invMass = 1.0f / mass; b.invInertia = 1.0f / inertia;
    return b;
}

RigidBody MakeRectangle(Vec2 pos, Vec2 vel, float w, float h, float angle, float density) {
    float mass    = density * w * h;
    float inertia = mass * (w*w + h*h) / 12.0f;
    RigidBody b;
    b.pos = pos; b.vel = vel; b.angle = angle;
    b.shape = ShapeType::Rectangle; b.halfExtents = {w * 0.5f, h * 0.5f};
    b.invMass = 1.0f / mass; b.invInertia = 1.0f / inertia;
    return b;
}

// ---- step ----

void StepRigidBodies(RigidBodyStore &store, float dt) {
    for (auto &b : store.bodies) {
        if (b.invMass == 0.0f) continue;
        b.pos.x  += b.vel.x  * dt;
        b.pos.y  += b.vel.y  * dt;
        b.angle  += b.angVel * dt;
        b.vel.x  *= LINEAR_DAMP;
        b.vel.y  *= LINEAR_DAMP;
        b.angVel *= ANGULAR_DAMP;
    }

    int n = (int)store.bodies.size();
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            RigidBody &a = store.bodies[i];
            RigidBody &b = store.bodies[j];
            Contact c;
            bool swapped = false;

            if      (a.shape == ShapeType::Circle    && b.shape == ShapeType::Circle)
                c = contactCircleCircle(a, b);
            else if (a.shape == ShapeType::Circle    && b.shape == ShapeType::Rectangle)
                c = contactCircleRect(a, b);
            else if (a.shape == ShapeType::Rectangle && b.shape == ShapeType::Circle) {
                c = contactCircleRect(b, a);
                c.normal = {-c.normal.x, -c.normal.y};
            } else
                c = contactRectRect(a, b);

            resolveContact(a, b, c);
        }
    }
}
