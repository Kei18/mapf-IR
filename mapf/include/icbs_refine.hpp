/*
 * Implementation of Improved Conflict-based Search (ICBS)
 * for Iterative Refinement
 */

#pragma once
#include "icbs.hpp"
#include "cbs_refine.hpp"

class ICBS_REFINE : public ICBS, CBS_REFINE {
private:
  void setInitialHighLevelNode(HighLevelNode_p n);
  Path getConstrainedPath(HighLevelNode_p h_node, int id);
  LibCBS::Constraints getPrioritizedConflict(HighLevelNode_p h_node);

public:
  ICBS_REFINE(Problem* _P,
              const Plan& _old_plan,
              const std::vector<int>& _sample);
  ~ICBS_REFINE() {};
};
