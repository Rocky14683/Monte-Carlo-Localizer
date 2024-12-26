
#include "robot.hpp"
#include <termios.h>
#include <unistd.h>
#include <print>
#include "mcl.hpp"

inline double mod(double first_term, double second_term) {
    return first_term - (second_term)*floor(first_term / (second_term));
}

inline double norm_delta(double radians) { return std::remainder(radians, 2 * M_PI); }

Robot::Robot(Pose pose, const std::array<double, 3>& noise) {
    this->pose = pose;
    forward_noise = noise[0];
    turn_noise = noise[1];
    sense_noise = noise[2];
}

void Robot::set_pose(double new_x, double new_y, double new_theta) {
    if (new_x < 0 || new_x >= world::world_size) throw std::invalid_argument("X coordinate out of bound");
    if (new_y < 0 || new_y >= world::world_size) throw std::invalid_argument("Y coordinate out of bound");
    if (new_theta < 0 || new_theta >= 2 * M_PI) throw std::invalid_argument("Orientation must be in [0..2pi]");

    this->pose.x = new_x;
    this->pose.y = new_y;
    this->pose.theta = new_theta;
}

void Robot::set_pose(Pose new_pose) {
    if (new_pose.x < 0 || new_pose.x >= world::world_size) throw std::invalid_argument("X coordinate out of bound");
    if (new_pose.y < 0 || new_pose.y >= world::world_size) throw std::invalid_argument("Y coordinate out of bound");
    if (new_pose.theta < 0 || new_pose.theta >= 2 * M_PI)
        throw std::invalid_argument("Orientation must be in [0..2pi]");

    this->pose = new_pose;
}

void Robot::set_noise(double new_forward_noise, double new_turn_noise, double new_sense_noise) {
    forward_noise = new_forward_noise;
    turn_noise = new_turn_noise;
    sense_noise = new_sense_noise;
}

std::vector<double> Robot::sense() {
    std::vector<double> z(sizeof(world::landmarks) / sizeof(world::landmarks[0]));
    double dist;

    for (int i = 0; i < sizeof(world::landmarks) / sizeof(world::landmarks[0]); i++) {
        dist = sqrt(pow((pose.x - world::landmarks[i][0]), 2) + pow((pose.y - world::landmarks[i][1]), 2));
        dist += gen_gauss_random(0.0, sense_noise);
        z[i] = dist;
    }
    return std::move(z);
}

Robot Robot::move(double turn, double forward) {
    if (forward < 0) throw std::invalid_argument("Robot cannot move backward");

    // turn, and add randomness to the turning command
    pose.theta = pose.theta + turn + gen_gauss_random(0.0, turn_noise);
    pose.theta = mod(pose.theta, 2 * M_PI);

    // move, and add randomness to the motion command
    double dist = forward + gen_gauss_random(0.0, forward_noise);
    pose.x = pose.x + (cos(pose.theta) * dist);
    pose.y = pose.y + (sin(pose.theta) * dist);

    // cyclic truncate
    pose.x = mod(pose.x, world::world_size);
    pose.y = mod(pose.y, world::world_size);

    return {pose, {forward_noise, turn_noise, sense_noise}};
}

void Robot::show_pose() const { printf("[ x= %.2f | y= %.2f | theta= %.2f ]\n", pose.x, pose.y, pose.theta); }

void Robot::read_sensors() {
    std::vector<double> z = sense();
    std::string readings = "[";
    for (double i : z) { readings += std::to_string(i) + " "; }
    readings[readings.size() - 1] = ']';
    std::cout << readings << std::endl;
}

double Robot::measurement_prob(const std::vector<double>& measurement) {
    double prob = 1.0;
    double dist;

    for (int i = 0; i < sizeof(world::landmarks) / sizeof(world::landmarks[0]); i++) {
        dist = sqrt(pow((pose.x - world::landmarks[i][0]), 2) + pow((pose.y - world::landmarks[i][1]), 2));
        prob *= gaussian(dist, sense_noise, measurement[i]);
    }

    return prob;
}
