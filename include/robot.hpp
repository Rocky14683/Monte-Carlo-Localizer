// modify from https://github.com/udacity/RoboND-MCL-Lab

#pragma once
#include <iostream>
#include <string>
#include <math.h>
#include <random>
#include <vector>
#include "particle.hpp"

// Landmarks
double landmarks[8][2] = {{20.0, 20.0}, {20.0, 80.0}, {20.0, 50.0}, {50.0, 20.0},
                          {50.0, 80.0}, {80.0, 80.0}, {80.0, 20.0}, {80.0, 50.0}};

// Map size in meters
double world_size = 100.0;

// Global Functions
extern inline double mod(double first_term, double second_term);
extern inline double gen_real_random(std::mt19937& rand_gen);
extern inline double norm_delta(double radians);

class Robot {
    public:
        explicit Robot();

        void set(double new_x, double new_y, double new_orient);

        void set_noise(double new_forward_noise, double new_turn_noise, double new_sense_noise);

        std::vector<double> sense();

        Robot&& move(double turn, double forward);

        void show_pose() const;

        void read_sensors();

        double measurement_prob(const std::vector<double>& measurement);

        Pose pose;
        double forward_noise, turn_noise, sense_noise; // robot noises
    private:
        inline double gen_gauss_random(double mean, double variance) {
            // Gaussian random
            std::normal_distribution<double> gauss_dist(mean, variance);
            return gauss_dist(gen);
        }

        inline double gaussian(double mu, double sigma, double x) const {
            // Probability of x for 1-dim Gaussian with mean mu and var. sigma
            return exp(-(pow((mu - x), 2)) / (pow(sigma, 2)) / 2.0) / sqrt(2.0 * M_PI * (pow(sigma, 2)));
        }
    private:
        std::random_device rd;
        std::mt19937 gen;
};