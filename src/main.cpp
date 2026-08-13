#include <SDL3/SDL.h>
#include <chrono>

#include "camera.hpp"
#include "input.hpp"
#include "physics.hpp"
#include "renderer.hpp"
#include "rope.hpp"
#include "scene.hpp"

using Clock    = std::chrono::steady_clock;
using Duration = std::chrono::duration<float>;

constexpr float FIXED_DT            = 1.0f / 60.0f;
constexpr int   MAX_STEPS_PER_FRAME = 5;

int main(int, char **) {
  if (!SDL_Init(SDL_INIT_VIDEO)) {
    SDL_Log("Failed to initialize SDL: %s", SDL_GetError());
    return 1;
  }

  SDL_Window   *window = SDL_CreateWindow("Snake", 1280, 720, SDL_WINDOW_RESIZABLE);
  SDL_Renderer *ren    = SDL_CreateRenderer(window, nullptr);

  Scene scene;
  InitRopeStore(scene.ropes);
  InitFoodStore(scene.foodStore, 100);

  Camera     cam{{0.0f, 0.0f}, 1.0f};
  InputState input;

  SpawnRope(scene.ropes, {-500.0f, 0.0f}, {500.0f, 0.0f}, 64);

  float accumulator  = 0.0f;
  float smoothFps    = 0.0f;
  bool  growByLength = false;
  auto  lastTime     = Clock::now();

  while (!input.quit) {
    auto  now       = Clock::now();
    float frameTime = Duration(now - lastTime).count();
    lastTime = now;
    if (frameTime > 0.0f)
      smoothFps += (1.0f / frameTime - smoothFps) * 0.1f;

    ProcessEvents(input, scene, cam, window);
    UpdateDrag(input, scene, cam, window, frameTime);

    accumulator += frameTime;
    int steps = 0;
    while (accumulator >= FIXED_DT && steps < MAX_STEPS_PER_FRAME) {
      StepPhysics(scene, FIXED_DT);
      accumulator -= FIXED_DT;
      ++steps;
    }

    if (!scene.ropes.ropeStart.empty()) {
      size_t base  = 0;
      Vec2   head  = scene.ropes.c_pos[base];
      for (auto &food : scene.foodStore.foods) {
        if (food.isEaten) continue;
        float dx = head.x - food.position.x, dy = head.y - food.position.y;
        if (dx*dx + dy*dy < food.radius * food.radius) {
          food.isEaten = true;
          int    segs  = scene.ropes.segCount[0];
          bool   atMax = segs >= (int)MAX_SEGMENTS_PER_ROPE;
          if (!growByLength && !atMax) {
            addRopeSegment(scene.ropes, static_cast<RopeId>(0));
          } else {
            size_t cbase       = 0;
            float  restLen     = scene.ropes.constraints[cbase].restLength;
            float  addPerSeg   = restLen / (segs - 1);
            for (int i = 0; i < segs - 1; ++i)
              scene.ropes.constraints[cbase + i].restLength += addPerSeg;
          }
          growByLength = !growByLength;
        }
      }
    }

    int wi, hi;
    SDL_GetWindowSize(window, &wi, &hi);
    UpdateCamera(cam, scene.ropes, input, (float)wi, (float)hi, frameTime);

    RenderFrame(ren, scene, cam, smoothFps);
  }

  SDL_DestroyRenderer(ren);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}
