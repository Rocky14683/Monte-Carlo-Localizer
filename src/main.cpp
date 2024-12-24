#include "rerun.hpp"
#include "rerun/demo_utils.hpp"
#include "mcl.cpp"
#include <random>
#include <unistd.h>

namespace rr = rerun;

int main() {
    std::srand(std::time(NULL));
    // Create a new `RecordingStream` which sends data over TCP to the viewer process.
    const auto rec = rerun::RecordingStream("mcl_visualizer");
    // Try to spawn a new viewer instance.
    rec.spawn().exit_on_failure();
    rr::Position3D pos;

    while (true) {
        pos = rr::Position3D {static_cast<float>(std::rand() % 100), static_cast<float>(std::rand() % 100),
                              static_cast<float>(std::rand() % 100)};
        rec.log("dot", rerun::Points3D(pos).with_radii(10).with_colors(0xFF0000FF));
//        sleep(0.1);
    }

    return 0;
}