#include "camera.hpp"
#include <cmath>

static constexpr float CAM_POS_RATE    = 0.6f;  // convergence rate per second
static constexpr float CAM_ZOOM_RATE   = 0.6f;
static constexpr float CAM_ZOOM_MARGIN = 0.50f;
static constexpr float CAM_ZOOM_MIN    = 0.01f;
static constexpr float CAM_ZOOM_MAX    = 0.8f;
static constexpr float CAM_DRAG_SKEW   = 0.7f;
static constexpr float CAM_DEADZONE    = 0.15f; // fraction from edge that triggers camera movement
static constexpr float CAM_RELEASE     = 0.30f; // camera stops only once all points are this far inside

void UpdateCamera(Camera &cam, const RopeStore &ropes,
                  const InputState &input, float sw, float sh, float dt) {
  if (ropes.ropeStart.empty()) return;
  size_t base = 0;
  int    segs = ropes.segCount[0];
  if (segs < 2) return;

  float minX = ropes.c_pos[base].x, maxX = minX;
  float minY = ropes.c_pos[base].y, maxY = minY;
  for (int i = 1; i < segs; ++i) {
    float x = ropes.c_pos[base + i].x, y = ropes.c_pos[base + i].y;
    if (x < minX) minX = x; if (x > maxX) maxX = x;
    if (y < minY) minY = y; if (y > maxY) maxY = y;
  }

  Vec2  bboxCenter = { (minX + maxX) * 0.5f, (minY + maxY) * 0.5f };
  float bboxW      = std::fmax(maxX - minX, 1.0f);
  float bboxH      = std::fmax(maxY - minY, 1.0f);

  Vec2 target = bboxCenter;
  if (input.dragging) {
    Vec2 drag = ropes.c_pos[input.dragIdx];
    target.x  = bboxCenter.x * (1.0f - CAM_DRAG_SKEW) + drag.x * CAM_DRAG_SKEW;
    target.y  = bboxCenter.y * (1.0f - CAM_DRAG_SKEW) + drag.y * CAM_DRAG_SKEW;
  }

  float targetZoom = std::fmin(sw * CAM_ZOOM_MARGIN / bboxW,
                               sh * CAM_ZOOM_MARGIN / bboxH);
  targetZoom = std::fmax(CAM_ZOOM_MIN, std::fmin(CAM_ZOOM_MAX, targetZoom));

  // hysteresis deadzone: activate when any point exits outer band, deactivate only
  // when all points are within the inner band — prevents chattering at the boundary
  static bool camActive = false;
  float trigX = sw * CAM_DEADZONE,  trigY = sh * CAM_DEADZONE;
  float relX  = sw * CAM_RELEASE,   relY  = sh * CAM_RELEASE;
  bool anyOutside   = false;
  bool allInsideRel = true;
  for (int i = 0; i < segs; ++i) {
    float sx = (ropes.c_pos[base + i].x - cam.pos.x) * cam.zoom + sw * 0.5f;
    float sy = (ropes.c_pos[base + i].y - cam.pos.y) * cam.zoom + sh * 0.5f;
    if (sx < trigX || sx > sw-trigX || sy < trigY || sy > sh-trigY) anyOutside   = true;
    if (sx < relX  || sx > sw-relX  || sy < relY  || sy > sh-relY)  allInsideRel = false;
  }
  if (anyOutside)             camActive = true;
  else if (allInsideRel)      camActive = false;
  if (!camActive) return;

  float posAlpha  = 1.0f - std::exp(-CAM_POS_RATE  * dt);
  float zoomAlpha = 1.0f - std::exp(-CAM_ZOOM_RATE * dt);
  cam.pos.x += (target.x - cam.pos.x) * posAlpha;
  cam.pos.y += (target.y - cam.pos.y) * posAlpha;
  cam.zoom  += (targetZoom - cam.zoom) * zoomAlpha;
}
