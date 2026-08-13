#include "renderer.hpp"
#include "scene.hpp"
#include <SDL3/SDL.h>
#include <SDL3_ttf/SDL_ttf.h>
#include <cmath>
#include <cstdio>
#include <vector>
#include <bit>

static constexpr float FLATNESS_PX    = 0.5f;
static constexpr int   MAX_SUBDIV     = 8;
static constexpr float ROPE_THICKNESS = 6.0f; // world-space units; scales with zoom
static constexpr float MAX_STRETCH    = 0.15f; // normalized stretch at which color is fully red

static Vec2 WorldToScreen(Vec2 w, const Camera &cam, float sw, float sh) {
  return {
      (w.x - cam.pos.x) * cam.zoom + sw * 0.5f,
      (w.y - cam.pos.y) * cam.zoom + sh * 0.5f,
  };
}

static Vec2 CReval(Vec2 p0, Vec2 p1, Vec2 p2, Vec2 p3, float t) {
  float t2 = t * t, t3 = t2 * t;
  float b0 = -t3 + 2.0f*t2 - t;
  float b1 =  3.0f*t3 - 5.0f*t2 + 2.0f;
  float b2 = -3.0f*t3 + 4.0f*t2 + t;
  float b3 =  t3 - t2;
  return { 0.5f*(b0*p0.x + b1*p1.x + b2*p2.x + b3*p3.x),
           0.5f*(b0*p0.y + b1*p1.y + b2*p2.y + b3*p3.y) };
}

// Adaptively collect screen-space points; a is already in pts, b is appended.
static void CRcollect(std::vector<Vec2> &pts,
                      Vec2 p0, Vec2 p1, Vec2 p2, Vec2 p3,
                      Vec2 a, Vec2 b, float ta, float tb, int depth) {
  float tm = (ta + tb) * 0.5f;
  Vec2  m  = CReval(p0, p1, p2, p3, tm);
  float dx = m.x - (a.x + b.x) * 0.5f;
  float dy = m.y - (a.y + b.y) * 0.5f;
  if (depth >= MAX_SUBDIV || dx*dx + dy*dy < FLATNESS_PX * FLATNESS_PX) {
    pts.push_back(b);
    return;
  }
  CRcollect(pts, p0, p1, p2, p3, a, m, ta, tm, depth + 1);
  CRcollect(pts, p0, p1, p2, p3, m, b, tm, tb, depth + 1);
}

static void DrawFilledCircle(SDL_Renderer *ren, Vec2 center, float radius, SDL_FColor col) {
  constexpr int SEGS = 24;
  std::vector<SDL_Vertex> verts;
  verts.reserve(SEGS + 1);
  verts.push_back({{center.x, center.y}, col, {0, 0}});
  for (int i = 0; i <= SEGS; ++i) {
    float a = (float)i / SEGS * 2.0f * 3.14159265f;
    verts.push_back({{center.x + std::cos(a) * radius, center.y + std::sin(a) * radius}, col, {0, 0}});
  }
  std::vector<int> indices;
  indices.reserve(SEGS * 3);
  for (int i = 0; i < SEGS; ++i) {
    indices.push_back(0);
    indices.push_back(i + 1);
    indices.push_back(i + 2);
  }
  SDL_RenderGeometry(ren, nullptr, verts.data(), (int)verts.size(), indices.data(), (int)indices.size());
}

static void DrawThickPolyline(SDL_Renderer *ren,
                              const std::vector<Vec2> &pts, const std::vector<Color> &colors, float thickness) {
  size_t n = pts.size();
  if (n < 2) return;

  auto segDir = [&](size_t i) -> Vec2 {
    float dx = pts[i+1].x - pts[i].x, dy = pts[i+1].y - pts[i].y;
    float len = std::sqrt(dx*dx + dy*dy);
    return len > 1e-6f ? Vec2{dx/len, dy/len} : Vec2{1.0f, 0.0f};
  };
  auto perp = [](Vec2 d) -> Vec2 { return {-d.y, d.x}; };

  // per-vertex normal: average of adjacent segment normals
  std::vector<Vec2> normals(n);
  normals[0]   = perp(segDir(0));
  normals[n-1] = perp(segDir(n-2));
  for (size_t i = 1; i < n-1; ++i) {
    Vec2 d0 = segDir(i-1), d1 = segDir(i);
    Vec2 avg = { d0.x + d1.x, d0.y + d1.y };
    float len = std::sqrt(avg.x*avg.x + avg.y*avg.y);
    normals[i] = len > 1e-6f ? perp({avg.x/len, avg.y/len}) : perp(d0);
  }

  float half = thickness * 0.5f;

  std::vector<SDL_Vertex> verts;
  verts.reserve(n * 2);
  for (size_t i = 0; i < n; ++i) {
    Vec2 nx = normals[i];
    Color col = colors[i];
    verts.push_back({{pts[i].x - nx.x*half, pts[i].y - nx.y*half}, std::bit_cast<SDL_FColor>(col), {0,0}});
    verts.push_back({{pts[i].x + nx.x*half, pts[i].y + nx.y*half}, std::bit_cast<SDL_FColor>(col), {0,0}});
  }

  std::vector<int> indices;
  indices.reserve((n-1) * 6);
  for (size_t i = 0; i < n-1; ++i) {
    int b = (int)(i * 2);
    indices.push_back(b);   indices.push_back(b+1); indices.push_back(b+2);
    indices.push_back(b+1); indices.push_back(b+3); indices.push_back(b+2);
  }

  SDL_RenderGeometry(ren, nullptr,
                     verts.data(), (int)verts.size(),
                     indices.data(), (int)indices.size());
}

void DrawRope(SDL_Renderer *ren, const RopeStore &ropes, const Camera &cam) {
  int iw, ih;
  SDL_GetRenderOutputSize(ren, &iw, &ih);
  float sw = (float)iw, sh = (float)ih;

  for (size_t r = 0; r < ropes.ropeStart.size(); ++r) {
    int    segs  = ropes.segCount[r];
    size_t base  = r * MAX_SEGMENTS_PER_ROPE;
    size_t cbase = r * (MAX_SEGMENTS_PER_ROPE - 1);
    if (segs < 2) continue;

    auto tensionColor = [&](int seg) -> Color {
      float dx = ropes.c_pos[base + seg + 1].x - ropes.c_pos[base + seg].x;
      float dy = ropes.c_pos[base + seg + 1].y - ropes.c_pos[base + seg].y;
      float dist    = std::sqrt(dx*dx + dy*dy);
      float rest    = ropes.constraints[cbase + seg].restLength;
      float stretch = (dist - rest) / rest;
      float t       = stretch < 0.0f ? 0.0f : stretch > MAX_STRETCH ? 1.0f : stretch / MAX_STRETCH;
      return {t, 0, 0, 1.0f};
    };

    // per-node color: average adjacent constraint tensions so colors blend smoothly across boundaries
    std::vector<Color> nodeColor(segs);
    for (int i = 0; i < segs; ++i) {
      if (i == 0) {
        nodeColor[i] = tensionColor(0);
      } else if (i == segs - 1) {
        nodeColor[i] = tensionColor(segs - 2);
      } else {
        Color a = tensionColor(i - 1), b = tensionColor(i);
        nodeColor[i] = {(a.r+b.r)*0.5f, (a.g+b.g)*0.5f, (a.b+b.b)*0.5f, 1.0f};
      }
    }

    std::vector<Vec2>  pts;
    std::vector<Color> ptColors;
    pts.reserve(segs * 4);
    ptColors.reserve(segs * 4);

    pts.push_back(WorldToScreen(ropes.c_pos[base], cam, sw, sh));
    ptColors.push_back(nodeColor[0]);

    for (int i = 0; i < segs - 1; ++i) {
      Vec2 sA = WorldToScreen(ropes.c_pos[base + i],     cam, sw, sh);
      Vec2 sB = WorldToScreen(ropes.c_pos[base + i + 1], cam, sw, sh);
      Vec2 sP = (i > 0)
        ? WorldToScreen(ropes.c_pos[base + i - 1], cam, sw, sh)
        : Vec2{2.0f*sA.x - sB.x, 2.0f*sA.y - sB.y};
      Vec2 sN = (i < segs - 2)
        ? WorldToScreen(ropes.c_pos[base + i + 2], cam, sw, sh)
        : Vec2{2.0f*sB.x - sA.x, 2.0f*sB.y - sA.y};

      size_t before = pts.size();
      CRcollect(pts, sP, sA, sB, sN, sA, sB, 0.0f, 1.0f, 0);
      size_t added = pts.size() - before;
      Color ca = nodeColor[i], cb = nodeColor[i + 1];
      for (size_t j = 0; j < added; ++j) {
        float t = (float)(j + 1) / (float)added;
        ptColors.push_back({ca.r + (cb.r-ca.r)*t, ca.g + (cb.g-ca.g)*t, ca.b + (cb.b-ca.b)*t, 1.0f});
      }
    }

    DrawThickPolyline(ren, pts, ptColors, ROPE_THICKNESS * cam.zoom);

    Vec2 head = WorldToScreen(ropes.c_pos[base], cam, sw, sh);
    SDL_FColor headCol = {0.2f, 0.2f, 0.2f, 1.0f};
    DrawFilledCircle(ren, head, ROPE_THICKNESS * cam.zoom * 0.5f, headCol);
  }
}

void DrawFood(SDL_Renderer *ren, const FoodStore &foodStore, const Camera &cam) {
  constexpr int SEGS = 24;
  int iw, ih;
  SDL_GetRenderOutputSize(ren, &iw, &ih);
  float sw = (float)iw, sh = (float)ih;

  std::vector<SDL_Vertex> verts;
  std::vector<int> indices;
  SDL_FColor green = {0.0f, 1.0f, 0.0f, 1.0f};

  for (const auto &food : foodStore.foods) {
    if (food.isEaten) continue;
    Vec2 center = WorldToScreen(food.position, cam, sw, sh);
    float radius = food.radius * cam.zoom;
    int base = (int)verts.size();
    verts.push_back({{center.x, center.y}, green, {0, 0}});
    for (int i = 0; i <= SEGS; ++i) {
      float a = (float)i / SEGS * 2.0f * 3.14159265f;
      verts.push_back({{center.x + std::cos(a) * radius, center.y + std::sin(a) * radius}, green, {0, 0}});
    }
    for (int i = 0; i < SEGS; ++i) {
      indices.push_back(base);
      indices.push_back(base + i + 1);
      indices.push_back(base + i + 2);
    }
  }

  if (!verts.empty())
    SDL_RenderGeometry(ren, nullptr, verts.data(), (int)verts.size(), indices.data(), (int)indices.size());
}

static void DrawFPS(SDL_Renderer *ren, float fps) {
  static TTF_Font *font = nullptr;
  if (!font) {
    TTF_Init();
    font = TTF_OpenFont("DejaVuSans.ttf", 14);
  }
  if (!font) return;

  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.0f fps", fps);

  SDL_Color black = {0, 0, 0, 255};
  SDL_Surface *surf = TTF_RenderText_Blended(font, buf, 0, black);
  if (!surf) return;
  SDL_Texture *tex = SDL_CreateTextureFromSurface(ren, surf);
  SDL_DestroySurface(surf);
  if (!tex) return;

  float w, h;
  SDL_GetTextureSize(tex, &w, &h);
  SDL_FRect dst = {8.0f, 8.0f, w, h};
  SDL_RenderTexture(ren, tex, nullptr, &dst);
  SDL_DestroyTexture(tex);
}

void DrawRigidBodies(SDL_Renderer *ren, const RigidBodyStore &store, const Camera &cam) {
  int iw, ih;
  SDL_GetRenderOutputSize(ren, &iw, &ih);
  float sw = (float)iw, sh = (float)ih;

  for (const auto &b : store.bodies) {
    Vec2 center = WorldToScreen(b.pos, cam, sw, sh);

    if (b.shape == ShapeType::Circle) {
      SDL_FColor col = {0.2f, 0.55f, 0.9f, 1.0f};
      DrawFilledCircle(ren, center, b.radius * cam.zoom, col);
      // rotation indicator
      float ex = center.x + std::cos(b.angle) * b.radius * cam.zoom;
      float ey = center.y + std::sin(b.angle) * b.radius * cam.zoom;
      SDL_SetRenderDrawColorFloat(ren, 1.0f, 1.0f, 1.0f, 1.0f);
      SDL_RenderLine(ren, center.x, center.y, ex, ey);
    } else {
      // Build rotated quad in screen space
      float c = std::cos(b.angle), s = std::sin(b.angle);
      float ex = b.halfExtents.x * cam.zoom, ey = b.halfExtents.y * cam.zoom;
      Vec2 ax = { c*ex,  s*ex};
      Vec2 ay = {-s*ey,  c*ey};
      SDL_FColor col = {0.9f, 0.6f, 0.15f, 1.0f};
      SDL_Vertex verts[4] = {
        {{center.x + ax.x + ay.x, center.y + ax.y + ay.y}, col, {0,0}},
        {{center.x - ax.x + ay.x, center.y - ax.y + ay.y}, col, {0,0}},
        {{center.x - ax.x - ay.x, center.y - ax.y - ay.y}, col, {0,0}},
        {{center.x + ax.x - ay.x, center.y + ax.y - ay.y}, col, {0,0}},
      };
      int indices[] = {0,1,2, 0,2,3};
      SDL_RenderGeometry(ren, nullptr, verts, 4, indices, 6);
      // rotation indicator (along local x-axis)
      SDL_SetRenderDrawColorFloat(ren, 1.0f, 1.0f, 1.0f, 1.0f);
      SDL_RenderLine(ren, center.x, center.y, center.x + ax.x, center.y + ax.y);
    }
  }
}

void RenderFrame(SDL_Renderer *ren, const Scene &scene, const Camera &cam, float fps) {
  SDL_SetRenderDrawColor(ren, 255, 255, 255, 255);
  SDL_RenderClear(ren);
  DrawRigidBodies(ren, scene.rigidBodies, cam);
  DrawRope(ren, scene.ropes, cam);
  DrawFood(ren, scene.foodStore, cam);
  DrawFPS(ren, fps);
  SDL_RenderPresent(ren);
}
