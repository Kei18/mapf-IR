#pragma once
#include "solver.hpp"

class SolverRefine {
protected:
  const Plan old_plan;
  const Paths old_paths;
  const int ub_makespan;
  const int ub_soc;
  const std::vector<int> sample;
  std::vector<int> fixed_agents;

public:
  SolverRefine(const Plan& _old_plan,
                const std::vector<int>& _sample);
  ~SolverRefine() {};
};
