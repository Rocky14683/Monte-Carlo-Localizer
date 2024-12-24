cmake_minimum_required(VERSION 3.16...3.27)
project(mcl LANGUAGES CXX)

add_executable(mcl src/main.cpp
        src/mcl.cpp
        include/particle.hpp)

# Download the rerun_sdk
include(FetchContent)
FetchContent_Declare(rerun_sdk URL
        https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

# Rerun requires at least C++17, but it should be compatible with newer versions.
set_property(TARGET mcl PROPERTY CXX_STANDARD 17)

# Link against rerun_sdk.
target_link_libraries(mcl PRIVATE rerun_sdk)