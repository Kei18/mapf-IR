/*
 * Implementation of Conflict-based Search (CBS)
 * for Iterative Refinement
 */

#pragma once
#include "cbs.hpp"

class CBS_REFINE : public CBS {
private:
  const int ub_makespan;
  const int ub_soc;

  CompareHighLevelNodes getObjective();
  Path getConstrainedPath(HighLevelNode* h_node, int id);

public:
  CBS_REFINE(Problem* _P, int _ub_makespan, int _ub_soc);
  ~CBS_REFINE() {};

  // void solve();
};
