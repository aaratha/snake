#include <SDL3/SDL.h>
#include <algorithm>

#include "scene.hpp"
#include "renderer.hpp"
#include "rope.hpp"

int main(int, char **) {
  if (!SDL_Init(SDL_INIT_VIDEO)) {
    SDL_Log("Failed to initialize SDL: %s", SDL_GetError());
    return 1;
  }

  SDL_Window *window = SDL_CreateWindow("Snake", 1280, 720, SDL_WINDOW_RESIZABLE);
  SDL_Renderer *ren = SDL_CreateRenderer(window, nullptr);

  Scene scene;
  InitRopeStore(scene.ropes);

  Camera cam{{0.0f, 0.0f}, 1.0f};

  // Spawn a rope hanging from world origin downward
  RopeId rope0 = SpawnRope(scene.ropes, {0.0f, -150.0f}, {0.0f, 150.0f}, 20);
  (void)rope0;
  // Pin the anchor (first point is fixed)
  scene.ropes.invMass[0] = 0.0f;

  bool running = true;
  while (running) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      if (event.type == SDL_EVENT_QUIT) running = false;
      if (event.type == SDL_EVENT_KEY_DOWN &&
          event.key.scancode == SDL_SCANCODE_ESCAPE)
        running = false;
    }

    RenderFrame(ren, scene, cam);
  }

  SDL_DestroyRenderer(ren);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}
