add_library(raygui STATIC raygui.h raygui.cpp)
target_link_libraries(raygui PUBLIC raylib_static)
target_include_directories(raygui PUBLIC .)