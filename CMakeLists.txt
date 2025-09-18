cmake_minimum_required(VERSION 3.20)

project(ToyPhysics)

find_package(SDL3 CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)

add_subdirectory(physics)
add_subdirectory(sandbox)