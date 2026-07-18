FetchContent_Declare(
    Raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG        dbc56a87da87d973a9c5baa4e7438a9d20121d28 # release-6.0.0
    GIT_PROGRESS   TRUE
)
FetchContent_MakeAvailable(Raylib)

add_subdirectory(raygui)

file(GLOB_RECURSE HEADER *.hpp)
file(GLOB_RECURSE SRC *.cpp)

add_executable(sandbox)
target_sources(sandbox PRIVATE ${HEADER} ${SRC})
target_compile_features(sandbox PRIVATE cxx_std_20)
target_link_libraries(sandbox PRIVATE toy_physics raylib_static raygui)
target_include_directories(sandbox PRIVATE .)

if (MSVC)
    target_compile_options(sandbox PRIVATE /utf-8)
endif()