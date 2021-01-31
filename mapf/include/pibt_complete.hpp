/*
 * Implementation of PIBT-Complete
 */

#pragma once
#include "solver.hpp"

class PIBT_COMPLETE : public Solver
{
private:
  // time required to complement plan, default zero
  double comp_time_complement;

  // complement-solver
  enum struct COMP_SOLVER_TYPE { PUSH_AND_SWAP, ICBS, ECBS };
  COMP_SOLVER_TYPE comp_solver_type;
  std::vector<std::string> option_comp_solver;

public:
  static const std::string SOLVER_NAME;

  void run();

public:
  PIBT_COMPLETE(Problem* _P);
  ~PIBT_COMPLETE() {}

  void makeLog(const std::string& logfile);
  void setParams(int argc, char* argv[]);
  static void printHelp();
};
