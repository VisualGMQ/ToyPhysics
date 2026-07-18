include(CTest)

FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG         191fa38c9b1596cd2576ab531d4ab4d5e8e05190 # release-3.15.2
    GIT_PROGRESS    TRUE
)
FetchContent_MakeAvailable(Catch2)

macro(AddTest test_name)
    add_executable(${test_name}_test ${test_name}.cpp)
    target_link_libraries(${test_name}_test PRIVATE toy_physics)
    add_test(NAME ${test_name}_test COMMAND $<TARGET_FILE:${test_name}_test>)
endmacro()

AddTest(math)