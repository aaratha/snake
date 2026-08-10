#pragma once

#include <cstddef>
#include "scene.hpp"
#include "types.hpp"

struct SDL_Window;

struct InputState {
  bool   quit     = false;
  bool   dragging = false;
  size_t dragIdx  = 0;
};

void ProcessEvents(InputState &input, Scene &scene, const Camera &cam, SDL_Window *window);
void UpdateDrag(InputState &input, Scene &scene, const Camera &cam, SDL_Window *window);
