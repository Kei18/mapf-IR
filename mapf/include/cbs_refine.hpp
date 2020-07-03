/*
 * Implementation of Conflict-based Search (CBS)
 * for Iterative Refinement
 */

#pragma once
#include "cbs.hpp"

class CBS_REFINE : public virtual CBS {
protected:
  const Plan old_plan;
  const Paths old_paths;
  const int ub_makespan;
  const int ub_soc;
  const std::vector<int> sample;
  std::vector<int> fixed_agents;

  bool makespan_prioritized;

  virtual void setInitialHighLevelNode(HighLevelNode_p n);
  virtual Path getConstrainedPath(HighLevelNode_p h_node, int id);
  Path getInitialPath(int id);
  CompareHighLevelNodes getObjective();

public:
  CBS_REFINE(Problem* _P,
             const Plan& _old_plan,
             const std::vector<int>& _sample);
  ~CBS_REFINE() {};

  void setParams(int argc, char *argv[]);
};
