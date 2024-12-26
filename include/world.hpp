#pragma once
#include "particle.hpp"

namespace world {
constexpr inline size_t n_landmarks = 8;

inline std::array<Point, n_landmarks> landmarks = {
    {{20.0, 20.0}, {20.0, 80.0}, {20.0, 50.0}, {50.0, 20.0}, {50.0, 80.0}, {80.0, 80.0}, {80.0, 20.0}, {80.0, 50.0}}};

// Map size in meters [0, 100] X [0, 100]
constexpr inline double world_size = 100.0;

constexpr inline double wall_size = 1.0;
}; // namespace world