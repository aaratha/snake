#pragma once

#include "input.hpp"
#include "rope.hpp"
#include "types.hpp"

void UpdateCamera(Camera &cam, const RopeStore &ropes,
                  const InputState &input, float sw, float sh, float dt);
