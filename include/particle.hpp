#pragma once
#include "rerun.hpp"

struct Point {
        double x;
        double y;

        const double operator[](int i) const {
            if (i == 0) return x;
            if (i == 1) return y;
            throw std::invalid_argument("Index out of bounds");
        }

        double& operator[](int i) {
            if (i == 0) return x;
            if (i == 1) return y;
            throw std::invalid_argument("Index out of bounds");
        }
};

struct Pose {
        double x;
        double y;
        double theta;

        Pose& operator=(const rerun::Position3D& pos) {
            this->x = pos.x();
            this->y = pos.y();
            this->theta = pos.z();
            return *this;
        }

        rerun::Position2D to_rr_position() { return rerun::Position2D(x, y); }
};

struct Particle : public Pose {
        float weight;
        unsigned int id;

        Particle() = default;
        Particle(const Pose& pose) : Pose(pose), weight(0), id(0) {};
        Particle(const Pose& pose, float weight, unsigned int id) : Pose(pose), weight(weight), id(id) {};
};