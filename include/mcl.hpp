#pragma once

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <ranges>

#include "particle.hpp"
#include "world.hpp"
#include "robot.hpp"
#include "rerun.hpp"

constexpr inline auto DT = 0.1;

template <size_t N_particle> class MonteCarloLocalization {
    public:
        MonteCarloLocalization(std::shared_ptr<Robot> robot, const std::array<float, 3>& std_deviation);

        inline std::array<Particle, N_particle> get_particles() const { return particles; }

        inline std::array<Pose, N_particle> get_particles_pose() const {
            std::array<Pose, N_particle> pose;
            std::transform(particles.begin(), particles.end(), pose.begin(),
                           [](const Particle& p) { return static_cast<Pose>(p); });
            return pose;
        }

        inline std::array<rerun::Position2D, N_particle> get_rr_particles_pose() const {
            std::array<rerun::Position2D, N_particle> pose;
            std::transform(particles.begin(), particles.end(), pose.begin(), [](const Robot& p) {
                p.show_pose();
                return rerun::Position2D {static_cast<float>(p.pose.x), static_cast<float>(p.pose.y)};
            });
            return pose;
        }

        inline static double gen_real_random(std::mt19937& rand_gen);

        Pose get_robot_pose() const { return robot->pose; }

    private:
        std::random_device rd;
        std::mt19937 gen {rd()};

        std::shared_ptr<Robot> robot;
        std::array<Robot, N_particle> particles;
        std::array<double, N_particle> weights;
};

template <size_t N_particle>
MonteCarloLocalization<N_particle>::MonteCarloLocalization(std::shared_ptr<Robot> robot,
                                                           const std::array<float, 3>& std_deviation)

    : robot(robot), particles(), weights() {
    Pose pose;

    for (int i = 0; i < N_particle; ++i) {
        pose.x = gen_real_random(this->gen) * world::world_size; // robot's x coordinate
        pose.y = gen_real_random(this->gen) * world::world_size; // robot's y coordinate
        pose.theta = gen_real_random(this->gen) * 2.0 * M_PI;
        particles[i].set_pose(pose);
        particles[i].set_noise(this->robot->forward_noise, this->robot->turn_noise, this->robot->sense_noise);
    }
}

template <size_t N_particle> double MonteCarloLocalization<N_particle>::gen_real_random(std::mt19937& rand_gen) {
    std::uniform_real_distribution<double> real_dist(0.0, 1.0); // Real
    return real_dist(rand_gen);
}
