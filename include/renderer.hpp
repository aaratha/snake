#pragma once

#include "types.hpp"
#include "rope.hpp"
#include "food.hpp"
#include "rigidbody.hpp"

struct SDL_Renderer;
struct Scene;

void DrawRope        (SDL_Renderer *ren, const RopeStore      &ropes,       const Camera &cam);
void DrawFood        (SDL_Renderer *ren, const FoodStore       &foodStore,   const Camera &cam);
void DrawRigidBodies (SDL_Renderer *ren, const RigidBodyStore  &rigidBodies, const Camera &cam);
void RenderFrame     (SDL_Renderer *ren, const Scene &scene, const Camera &cam, float fps);
