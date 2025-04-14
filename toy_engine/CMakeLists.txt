file(GLOB_RECURSE SRC src/*.cpp)
file(GLOB_RECURSE INCLUDE include/*.hpp)

add_library(ToyPhysics STATIC ${SRC} ${INCLUDE})
target_include_directories(ToyPhysics PUBLIC include)
target_compile_features(ToyPhysics PRIVATE cxx_std_17)
target_link_libraries(ToyPhysics PRIVATE NickelEngine)