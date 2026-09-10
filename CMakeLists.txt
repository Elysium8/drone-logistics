cmake_minimum_required(VERSION 3.10)
project(DroneLogistics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

include_directories(include)

add_executable(mapf_solver 
    src/main.cpp 
    src/Environment.cpp
)