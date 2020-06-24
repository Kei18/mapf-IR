/*
 * Implementation of Conflict-based Search (CBS)
 * for Iterative Refinement
 */

#pragma once
#include "cbs.hpp"

class CBS_REFINE : public CBS {
private:
  const Plan old_plan;
  const Paths old_paths;
  const int ub_makespan;
  const int ub_soc;
  const std::vector<int> sample;
  std::vector<int> fixed_agents;

  void setInitialHighLevelNode(HighLevelNode* n);
  Path getInitialPath(int id);
  CompareHighLevelNodes getObjective();
  Path getConstrainedPath(HighLevelNode* h_node, int id);

public:
  CBS_REFINE(Problem* _P,
             const Plan& _old_plan,
             const std::vector<int>& _sample);
  ~CBS_REFINE() {};
};
