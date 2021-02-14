cmake_minimum_required(VERSION 3.16)
project(mapf-refine CXX)

add_subdirectory(./mapf)
add_subdirectory(./third_party/googletest)

add_executable(app app.cpp)
target_compile_features(app PUBLIC cxx_std_17)
target_link_libraries(app lib-mapf)

# format
add_custom_target(clang-format
  COMMAND clang-format -i
  ../mapf/include/*.hpp
  ../mapf/src/*.cpp
  ../tests/*.cpp
  ../app.cpp)

# test
set(TEST_MAIN_FUNC ./third_party/googletest/googletest/src/gtest_main.cc)
set(TEST_ALL_SRC ${TEST_MAIN_FUNC})
macro(add_test name target)
  add_executable(${name} ${target} ${TEST_MAIN_FUNC})
  target_link_libraries(${name} lib-mapf gtest)
  list(APPEND TEST_ALL_SRC ${target})
endmacro(add_test)

# basic
add_test(test_plan ./tests/test_plan.cpp)
add_test(test_paths ./tests/test_paths.cpp)
add_test(test_solver ./tests/test_solver.cpp)
add_test(test_problem ./tests/test_problem.cpp)
add_test(test_lib_ir ./tests/test_lib_ir.cpp)
# solvers
add_test(test_hca ./tests/test_hca.cpp)
add_test(test_whca ./tests/test_whca.cpp)
add_test(test_revisit_pp ./tests/test_revisit_pp.cpp)
add_test(test_pibt ./tests/test_pibt.cpp)
add_test(test_winpibt ./tests/test_winpibt.cpp)
add_test(test_cbs ./tests/test_cbs.cpp)
add_test(test_icbs ./tests/test_icbs.cpp)
add_test(test_ecbs ./tests/test_ecbs.cpp)
add_test(test_push_and_swap ./tests/test_push_and_swap.cpp)
add_test(test_pibt_complete ./tests/test_pibt_complete.cpp)
add_test(test_ir ./tests/test_ir.cpp)

add_executable(test ${TEST_ALL_SRC})
target_link_libraries(test lib-mapf gtest)
