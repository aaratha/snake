#pragma once
#include <cstdint>

struct SpatialHash {
  static constexpr int      CAPACITY = 4096;
  static constexpr uint64_t EMPTY    = ~0ULL;
  struct Slot { uint64_t key; int seg; };
  Slot slots[CAPACITY];

  void clear() { for (auto &s : slots) s.key = EMPTY; }

  static uint64_t encode(int x, int y) {
    return ((uint64_t)(uint32_t)x << 32) | (uint32_t)y;
  }
  static uint32_t bucket(uint64_t k) {
    k ^= k >> 33; k *= 0xff51afd7ed558ccdULL; k ^= k >> 33;
    return (uint32_t)k & (CAPACITY - 1);
  }

  void insert(int cx, int cy, int seg) {
    uint64_t k = encode(cx, cy);
    uint32_t h = bucket(k);
    while (slots[h].key != EMPTY) h = (h + 1) & (CAPACITY - 1);
    slots[h] = {k, seg};
  }

  template<typename F>
  void query(int cx, int cy, F &&f) const {
    uint64_t k = encode(cx, cy);
    uint32_t h = bucket(k);
    while (slots[h].key != EMPTY) {
      if (slots[h].key == k) f(slots[h].seg);
      h = (h + 1) & (CAPACITY - 1);
    }
  }
};
