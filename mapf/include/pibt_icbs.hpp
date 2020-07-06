#pragma once
#include "solver.hpp"
class PIBT_ICBS : public Solver {
public:
  static const std::string SOLVER_NAME;

  void run();

public:
  PIBT_ICBS(Problem* _P);
  ~PIBT_ICBS();

  static void printHelp();
};
