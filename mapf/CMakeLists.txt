cmake_minimum_required(VERSION 3.1)
project(lib-mapf)
add_library(lib-mapf STATIC
  ./src/graph.cpp
  ./src/problem.cpp
  ./src/solver.cpp
  ./src/pibt.cpp
  ./src/hca.cpp)
target_compile_options(lib-mapf PUBLIC -O3 -Wall -mtune=native -march=native)
target_compile_features(lib-mapf PUBLIC cxx_std_17)
target_include_directories(lib-mapf INTERFACE ./include)
