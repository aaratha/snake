#include "renderer.hpp"
#include "scene.hpp"
#include <SDL3/SDL.h>

static Vec2 WorldToScreen(Vec2 w, const Camera &cam, float sw, float sh) {
  return {
      (w.x - cam.pos.x) * cam.zoom + sw * 0.5f,
      (w.y - cam.pos.y) * cam.zoom + sh * 0.5f,
  };
}

void DrawRope(SDL_Renderer *ren, const RopeStore &ropes, const Camera &cam) {
  int iw, ih;
  SDL_GetRenderOutputSize(ren, &iw, &ih);
  float sw = (float)iw, sh = (float)ih;

  for (size_t r = 0; r < ropes.ropeStart.size(); ++r) {
    int segs = ropes.segCount[r];
    size_t base = r * MAX_SEGMENTS_PER_ROPE;

    SDL_SetRenderDrawColor(ren, 0, 0, 0, 255);
    for (int i = 0; i < segs - 1; ++i) {
      Vec2 a = WorldToScreen(ropes.c_pos[base + i], cam, sw, sh);
      Vec2 b = WorldToScreen(ropes.c_pos[base + i + 1], cam, sw, sh);
      SDL_RenderLine(ren, a.x, a.y, b.x, b.y);
    }
  }
}

void RenderFrame(SDL_Renderer *ren, const Scene &scene, const Camera &cam) {
  SDL_SetRenderDrawColor(ren, 255, 255, 255, 255);
  SDL_RenderClear(ren);
  DrawRope(ren, scene.ropes, cam);
  SDL_RenderPresent(ren);
}
