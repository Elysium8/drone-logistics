cmake_minimum_required(VERSION 3.10)
project(DroneLogistics)

# Use modern C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Optimize for speed (crucial for pathfinding performance)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# Tell CMake where your header files live
include_directories(include)

# Compile main.cpp and Environment.cpp into an executable named 'mapf_solver'
add_executable(mapf_solver 
    src/main.cpp 
    src/Environment.cpp
)