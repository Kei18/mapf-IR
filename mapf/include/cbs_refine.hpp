/*
 * Implementation of Conflict-based Search (CBS)
 * for Iterative Refinement
 */

#pragma once
#include "cbs.hpp"
#include "solver_refine.hpp"

class CBS_REFINE : public virtual CBS, public SolverRefine {
protected:
  virtual void setInitialHighLevelNode(HighLevelNode* n);
  virtual Path getConstrainedPath(HighLevelNode* h_node, int id);
  Path getInitialPath(int id);
  CompareHighLevelNodes getObjective();

public:
  CBS_REFINE(Problem* _P,
             const Plan& _old_plan,
             const std::vector<int>& _sample);
  ~CBS_REFINE() {};
};
