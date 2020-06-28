/*
 * Implementation of Conflict-based Search (CBS)
 * for Iterative Refinement
 */

#pragma once
#include "cbs.hpp"
#include "solver_refine.hpp"

class CBS_REFINE : public CBS, SolverRefine {
private:
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
