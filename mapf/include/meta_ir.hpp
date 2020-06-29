#pragma once
#include "ir.hpp"

class META_IR : public IR {
public:
  static const std::string SOLVER_NAME;

private:
  int min_threshold_makespan;
  int max_threshold_makespan;
  static const int DEFAULT_MIN_THRESHOLD_MAKESPAN;
  static const int DEFAULT_MAX_THRESHOLD_MAKESPAN;

  void iterativeRefinement();

public:
  META_IR(Problem* _P);
  ~META_IR() {}

  void setParams(int argc, char *argv[]);
  static void printHelp();
};
