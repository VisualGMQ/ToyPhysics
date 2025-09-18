file(GLOB_RECURSE HEADER *.hpp)
file(GLOB_RECURSE SRC *.cpp)

add_executable(sandbox)
target_sources(sandbox PRIVATE ${HEADER} ${SRC})
target_compile_features(sandbox PRIVATE cxx_std_20)
target_link_libraries(sandbox PRIVATE toy_physics SDL3::SDL3)

if (MSVC)
    target_compile_options(sandbox PRIVATE /utf-8)
endif()