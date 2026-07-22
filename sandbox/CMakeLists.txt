CPMAddPackage("gh:raysan5/raylib#6.0")
CPMAddPackage("gh:nlohmann/json@3.12.0")
add_subdirectory(raygui)

file(GLOB_RECURSE HEADER *.hpp)
file(GLOB_RECURSE SRC *.cpp)

add_executable(sandbox)
target_sources(sandbox PRIVATE ${HEADER} ${SRC})
target_compile_features(sandbox PRIVATE cxx_std_20)
target_link_libraries(sandbox PRIVATE toy_physics raylib_static raygui nlohmann_json)
target_include_directories(sandbox PRIVATE .)

if (EMSCRIPTEN)
    target_compile_definitions(sandbox PRIVATE "TOY_PHYSICS_PLATFORM_WEB")
    target_link_options(sandbox PRIVATE "-sASYNCIFY")
    set_target_properties(
        sandbox
            PROPERTIES
            PREFIX ""
            SUFFIX ".html")
    # embed assets
    target_link_options(sandbox PRIVATE
            "--preload-file" "${CMAKE_CURRENT_SOURCE_DIR}/assets@/sandbox/assets")
endif()

if (MSVC)
    target_compile_options(sandbox PRIVATE /utf-8)
endif()