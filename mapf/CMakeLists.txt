cmake_minimum_required(VERSION 3.1)
project(lib-mapf)
add_library(lib-mapf STATIC
  ./src/graph.cpp
  ./src/problem.cpp
  ./src/paths.cpp
  ./src/plan.cpp
  ./src/solver.cpp
  ./src/pibt.cpp
  ./src/hca.cpp
  ./src/whca.cpp
  ./src/lib_cbs.cpp
  ./src/cbs.cpp
  ./src/ecbs.cpp
  ./src/ir.cpp
  ./src/ir_configs.cpp
  ./src/ir_paths.cpp
  ./src/icbs.cpp
  ./src/cbs_refine.cpp
  ./src/icbs_refine.cpp
  ./src/pibt_icbs.cpp
  ./src/ir_tester.cpp
  )
target_compile_options(lib-mapf PUBLIC -O3 -Wall -mtune=native -march=native)
target_compile_features(lib-mapf PUBLIC cxx_std_17)
target_include_directories(lib-mapf INTERFACE ./include)
