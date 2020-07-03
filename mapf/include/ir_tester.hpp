#pragma once

#include "ir.hpp"

class IR_TESTER : public IR {
public:
  static const std::string SOLVER_NAME;

protected:
  void iterativeRefinement();
  Plan getPickUpRefinement(const Plan& plan);
  Plan MAPFSolver(const Config& config_s,
                  const Config& config_g,
                  const Plan& current_plan);

public:
  IR_TESTER(Problem* _P);
  ~IR_TESTER();
};
