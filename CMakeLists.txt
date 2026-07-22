cmake_minimum_required(VERSION 4.0)

project(ToyPhysics)

option(TOYPHYSICS_BUILD_SANDBOX "build toy physics sandbox" OFF)
option(TOYPHYSICS_BUILD_TEST "build toy physics unittests" OFF)
option(TOYPHYSICS_USE_DOUBLE "use double floating-point" OFF)

if(NOT DEFINED ENV{CPM_SOURCE_CACHE})
    set(CPM_SOURCE_CACHE ${CMAKE_CURRENT_SOURCE_DIR}/.deps CACHE PATH "ToyPhysics dependencies cache")
endif()
include(cmake/CPM.cmake)

add_subdirectory(physics)

if (NOT ANDROID)
    if(PROJECT_IS_TOP_LEVEL)
        add_subdirectory(sandbox)
        add_subdirectory(tests)
    else()
        if(TOYPHYSICS_BUILD_SANDBOX)
            add_subdirectory(sandbox)
        endif()
        if(TOYPHYSICS_BUILD_TEST)
            add_subdirectory(tests)
        endif()
    endif()
endif()
