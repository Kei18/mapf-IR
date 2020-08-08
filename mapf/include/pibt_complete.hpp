#pragma once
#include "solver.hpp"


class PIBT_COMPLETE : public Solver {
private:
  double comp_time_complement;

public:
  static const std::string SOLVER_NAME;

  void run();

public:
  PIBT_COMPLETE(Problem* _P);
  ~PIBT_COMPLETE();

  void makeLog(const std::string& logfile);
  static void printHelp();
};
