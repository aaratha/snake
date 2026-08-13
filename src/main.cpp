#include <SDL3/SDL.h>
#include <chrono>

#include "camera.hpp"
#include "food.hpp"
#include "input.hpp"
#include "physics.hpp"
#include "renderer.hpp"
#include "rigidbody.hpp"
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

  scene.rigidBodies.bodies.push_back(MakeCircle   ({-200.0f, 200.0f}, {0.0f, 0.0f}, 40.0f));
  scene.rigidBodies.bodies.push_back(MakeRectangle({ 150.0f, 250.0f}, {0.0f, 0.0f}, 100.0f, 60.0f, 0.4f));
  scene.rigidBodies.bodies.push_back(MakeCircle   ({ 300.0f, 150.0f}, {0.0f, 0.0f}, 30.0f));

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
      StepRigidBodies(scene.rigidBodies, FIXED_DT);
      ResolveRopeRigidBodyCollisions(scene.ropes, scene.rigidBodies);
      CheckFoodCollisions(scene, growByLength);
      accumulator -= FIXED_DT;
      ++steps;
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
