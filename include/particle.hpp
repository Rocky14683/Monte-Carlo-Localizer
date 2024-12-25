#pragma once
#include "rerun.hpp"



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

        rerun::Position3D to_position3d() {
            return rerun::Position3D(x, y, theta);
        }
};



struct Particle : public Pose {
        float weight;
        unsigned int id;

        Particle() = default;
        Particle(const Pose& pose) : Pose(pose), weight(0), id(0) {};
        Particle(const Pose& pose, float weight, unsigned int id) : Pose(pose), weight(weight), id(id) {};
};