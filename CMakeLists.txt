cmake_minimum_required(VERSION 3.25)
project(ToyPhysics)

add_subdirectory(NickelEngine/engine)
add_subdirectory(toy_engine)

option(TOY_ENGINE_BUILD_SANDBOX "build sandbox" OFF)

if (TOY_ENGINE_BUILD_SANDBOX)
    add_subdirectory(sandbox)
endif()