include(CTest)
CPMAddPackage("gh:catchorg/Catch2@3.15.2")

macro(AddTest test_name)
    add_executable(${test_name}_test ${test_name}.cpp)
    target_link_libraries(${test_name}_test PRIVATE toy_physics)
    add_test(NAME ${test_name}_test COMMAND $<TARGET_FILE:${test_name}_test>)
endmacro()

AddTest(math)