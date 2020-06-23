/*
 * Implementation of Enhanced Conflict-based Search (ECBS)
 * for Iterative Refinement
 */

#pragma once
#include "ecbs.hpp"

class ECBS_REFINE : public ECBS {
private:
  const int ub_makespan;
  const int ub_soc;

  CompareHighLevelNodeECBS getMainObjective();
  CompareHighLevelNodeECBS getFocalObjective();
  void invoke(HighLevelNodeECBS* h_node, int id);
  Path getConstrainedPath(HighLevelNodeECBS* h_node, int id);
  std::tuple<Path, int> getFocalPath(HighLevelNodeECBS* h_node,
                                     int id);

public:
  ECBS_REFINE(Problem* _P, int _ub_makespan, int _ub_soc);
  ~ECBS_REFINE() {};
};
