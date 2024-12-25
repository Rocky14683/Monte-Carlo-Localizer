cmake_minimum_required(VERSION 3.28)
project(mcl LANGUAGES CXX)

add_executable(mcl src/main.cpp
        src/mcl.cpp
        src/robot.cpp)

target_include_directories(mcl PRIVATE include)
# Download the rerun_sdk
include(FetchContent)
FetchContent_Declare(rerun_sdk URL
        https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

# Rerun requires at least C++17, but it should be compatible with newer versions.
set_property(TARGET mcl PROPERTY CXX_STANDARD 23)

# Link against rerun_sdk.
target_link_libraries(mcl PRIVATE rerun_sdk)