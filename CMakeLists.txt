cmake_minimum_required(VERSION 4.0)

project(ToyPhysics)

option(TOYPHYSICS_BUILD_SANDBOX "build toy physics sandbox" OFF)
option(TOYPHYSICS_BUILD_TEST "build toy physics unittests" OFF)
option(TOYPHYSICS_USE_DOUBLE "use double floating-point" OFF)

add_subdirectory(3rdlibs)
add_subdirectory(physics)

if(PROJECT_IS_TOP_LEVEL)
    include(FetchContent)
    add_subdirectory(sandbox)
    add_subdirectory(tests)
endif()
