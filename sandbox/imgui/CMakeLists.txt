file(GLOB IMGUI_SRC *.h *.hpp *.cpp)

add_library(imgui STATIC)
target_sources(imgui PRIVATE ${IMGUI_SRC})
target_link_libraries(imgui PRIVATE SDL3::SDL3)
target_include_directories(imgui PUBLIC .)