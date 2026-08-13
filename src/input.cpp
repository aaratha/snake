#include "input.hpp"
#include "physics.hpp"
#include <SDL3/SDL.h>
#include <cmath>

static constexpr float GRAB_RADIUS = 20.0f;
static constexpr float DRAG_RATE   = 4.0f;  // convergence rate per second

static Vec2 ScreenToWorld(float sx, float sy, const Camera &cam, int winW, int winH) {
  return {(sx - winW * 0.5f) / cam.zoom + cam.pos.x,
          (sy - winH * 0.5f) / cam.zoom + cam.pos.y};
}

void ProcessEvents(InputState &input, Scene &scene, const Camera &cam, SDL_Window *window) {
  int winW, winH;
  SDL_GetWindowSize(window, &winW, &winH);

  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    if (event.type == SDL_EVENT_QUIT) input.quit = true;
    if (event.type == SDL_EVENT_KEY_DOWN &&
        event.key.scancode == SDL_SCANCODE_ESCAPE)
      input.quit = true;

    if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
      Vec2 mw = ScreenToWorld(event.button.x, event.button.y, cam, winW, winH);
      for (size_t r = 0; r < scene.ropes.ropeStart.size() && !input.dragging; ++r) {
        int    segs = scene.ropes.segCount[r];
        size_t base = r * MAX_SEGMENTS_PER_ROPE;
        for (size_t ep : {base, base + (size_t)(segs - 1)}) {
          Vec2  p  = scene.ropes.c_pos[ep];
          float dx = p.x - mw.x, dy = p.y - mw.y;
          if (dx * dx + dy * dy < GRAB_RADIUS * GRAB_RADIUS) {
            input.dragging = true;
            input.dragIdx  = ep;
            scene.ropes.invMass[ep] = 0.0f;
            break;
          }
        }
      }
    }

    if (event.type == SDL_EVENT_MOUSE_BUTTON_UP && input.dragging) {
      scene.ropes.invMass[input.dragIdx] = 1.0f;
      input.dragging = false;
    }
  }
}

void UpdateDrag(InputState &input, Scene &scene, const Camera &cam, SDL_Window *window, float dt) {
  if (!input.dragging) return;
  int winW, winH;
  SDL_GetWindowSize(window, &winW, &winH);
  float mx, my;
  SDL_GetMouseState(&mx, &my);
  Vec2  target = ScreenToWorld(mx, my, cam, winW, winH);
  Vec2 &pos    = scene.ropes.c_pos[input.dragIdx];
  Vec2 &prev   = scene.ropes.p_pos[input.dragIdx];

  prev = pos;
  float alpha = 1.0f - std::exp(-DRAG_RATE * dt);
  float nx = (target.x - pos.x) * alpha;
  float ny = (target.y - pos.y) * alpha;
  float speed2 = nx*nx + ny*ny;
  if (speed2 > MAX_VELOCITY * MAX_VELOCITY) {
    float scale = MAX_VELOCITY / std::sqrt(speed2);
    nx *= scale;
    ny *= scale;
  }
  pos.x += nx;
  pos.y += ny;
}
