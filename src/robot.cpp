
#include "../include/robot.hpp"
#include <termios.h>
#include <unistd.h>

inline double gen_real_random(std::mt19937& rand_gen) {
    std::uniform_real_distribution<double> real_dist(0.0, 1.0); // Real
    return real_dist(rand_gen);
}

inline double mod(double first_term, double second_term) {
    return first_term - (second_term)*floor(first_term / (second_term));
}

inline double norm_delta(double radians) { return std::remainder(radians, 2 * M_PI); }

Robot::Robot() {
    pose.x = gen_real_random(this->gen) * world_size; // robot's x coordinate
    pose.y = gen_real_random(this->gen) * world_size; // robot's y coordinate
    pose.theta = gen_real_random(this->gen) * 2.0 * M_PI; // robot's orientation

    forward_noise = 0.0; // noise of the forward movement
    turn_noise = 0.0; // noise of the turn
    sense_noise = 0.0; // noise of the sensing
}

void Robot::set(double new_x, double new_y, double new_theta) {
    if (new_x < 0 || new_x >= world_size) throw std::invalid_argument("X coordinate out of bound");
    if (new_y < 0 || new_y >= world_size) throw std::invalid_argument("Y coordinate out of bound");
    if (new_theta < 0 || new_theta >= 2 * M_PI) throw std::invalid_argument("Orientation must be in [0..2pi]");

    this->pose.x = new_x;
    this->pose.y = new_y;
    this->pose.theta = new_theta;
}

void Robot::set_noise(double new_forward_noise, double new_turn_noise, double new_sense_noise) {
    forward_noise = new_forward_noise;
    turn_noise = new_turn_noise;
    sense_noise = new_sense_noise;
}

std::vector<double> Robot::sense() {
    std::vector<double> z(sizeof(landmarks) / sizeof(landmarks[0]));
    double dist;

    for (int i = 0; i < sizeof(landmarks) / sizeof(landmarks[0]); i++) {
        dist = sqrt(pow((pose.x - landmarks[i][0]), 2) + pow((pose.y - landmarks[i][1]), 2));
        dist += gen_gauss_random(0.0, sense_noise);
        z[i] = dist;
    }
    return std::move(z);
}

Robot&& Robot::move(double turn, double forward) {
    if (forward < 0) throw std::invalid_argument("Robot cannot move backward");

    // turn, and add randomness to the turning command
    pose.theta = pose.theta + turn + gen_gauss_random(0.0, turn_noise);
    pose.theta = mod(pose.theta, 2 * M_PI);

    // move, and add randomness to the motion command
    double dist = forward + gen_gauss_random(0.0, forward_noise);
    pose.x = pose.x + (cos(pose.theta) * dist);
    pose.y = pose.y + (sin(pose.theta) * dist);

    // cyclic truncate
    pose.x = mod(pose.x, world_size);
    pose.y = mod(pose.y, world_size);

    // set particle
    Robot res;
    res.set(pose.x, pose.y, pose.theta);
    res.set_noise(forward_noise, turn_noise, sense_noise);

    return std::move(res);
}

void Robot::show_pose() const {
    auto out =
        "[x=" + std::to_string(pose.x) + " y=" + std::to_string(pose.y) + " orient=" + std::to_string(pose.theta) + "]";
    std::cout << out << std::endl;
}

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

    for (int i = 0; i < sizeof(landmarks) / sizeof(landmarks[0]); i++) {
        dist = sqrt(pow((pose.x - landmarks[i][0]), 2) + pow((pose.y - landmarks[i][1]), 2));
        prob *= gaussian(dist, sense_noise, measurement[i]);
    }

    return prob;
}
