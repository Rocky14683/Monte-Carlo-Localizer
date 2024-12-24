#pragma once
#include "rerun.hpp"


using Pose = rerun::Position3D;

struct Particle : public Pose {
        float weight;
        unsigned int id;

        Particle() = default;
        Particle(const Pose& pose): Pose(pose), weight(0), id(0) {};
        Particle(const Pose& pose, float weight, unsigned int id): Pose(pose), weight(weight), id(id) {};
};