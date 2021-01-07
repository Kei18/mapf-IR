/*
 * Implementation of CBS for Iterative Refinement
 * This cannot be used directly
 */

#pragma once
#include "cbs.hpp"

class CBS_REFINE : public virtual CBS
{
protected:
  const Plan old_plan;                // old plan, equivalent to old_paths
  const Paths old_paths;              // old paths, equivalent to old_plan
  const int ub_makespan;              // makespan in the old plan
  const int ub_soc;                   // sum of costs in the old plan
  const std::vector<int> modif_list;  // a modification list M
  std::vector<int> fixed_agents;      // A \ modif_list

  // true -> makespan optimization, default: false
  // notice: SOC and makespan are Pareto structure.
  bool makespan_prioritized;

  virtual void setInitialHighLevelNode(HighLevelNode_p n);
  virtual Path getConstrainedPath(HighLevelNode_p h_node, int id);
  Path getInitialPath(int id);
  CompareHighLevelNodes getObjective();

public:
  CBS_REFINE(Problem* _P, const Plan& _old_plan,
             const std::vector<int>& _modif_list);
  ~CBS_REFINE(){};

  void setParams(int argc, char* argv[]);
};
