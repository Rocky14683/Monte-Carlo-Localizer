#pragma once

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "particle.hpp"

constexpr inline auto DT = 0.1;

template <size_t N_particle> class MonteCarloLocalization {
    public:
        MonteCarloLocalization(const Pose& init_pos, const std::array<float, 3>& std_deviation) {

        }


    private:
        std::array<Particle, N_particle> particles;
        std::array<float, N_particle> weights;
};
