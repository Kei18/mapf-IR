#include "../include/icbs_refine.hpp"

ICBS_REFINE::ICBS_REFINE(Problem* _P,
                         const Plan& _old_plan,
                         const std::vector<int>& _sample)
  : CBS(_P), ICBS(_P),
    CBS_REFINE(_P, _old_plan, _sample)
{
}

void ICBS_REFINE::setInitialHighLevelNode(HighLevelNode_p n)
{
  if (sample.empty()) {
    ICBS::setInitialHighLevelNode(n);
    return;
  }

  Paths paths(P->getNum());
  LibCBS::MDDs mdds;

  // constraints by fixed agents
  LibCBS::Constraints constraints
    = LibCBS::getConstraintsByFixedPaths(old_plan, fixed_agents);

  auto t_s = Time::now();

  // find paths
  for (int i = 0; i < P->getNum(); ++i) {
    // check time limit
    if (overCompTime()) {
      n->valid = false;
      return;
    }
    Path path;
    LibCBS::MDD_p mdd;
    if (inArray(i, sample)) {
      path = CBS_REFINE::getInitialPath(i);
      if (path.empty()) {
        n->valid = false;
        return;
      }
      // time limit
      int time_limit = max_comp_time - (int)getSolverElapsedTime();
      if (time_limit <= 0) {
        n->valid = false;
        return;
      }
      mdd = std::make_shared<LibCBS::MDD>
        (LibCBS::MDD(path.size()-1, i, P, constraints, time_limit));
      if (!mdd->valid) {
        n->valid = false;
        return;
      }
    } else {  // fixed agents
      path = old_paths.get(i);  // fixed agents
      // mdd is not required
    }
    paths.insert(i, path);
    mdds.push_back(mdd);
  }
  MDDTable[n->id] = mdds;

  n->id = 0;
  n->paths = paths;
  n->constraints = constraints;
  n->makespan = paths.getMakespan();
  n->soc = paths.getSOC();
  n->f = paths.countConflict(sample);
  n->valid = true;  // valid
}

LibCBS::Constraints ICBS_REFINE::getPrioritizedConflict
(HighLevelNode_p h_node)
{
  if (sample.empty()) {
    return LibCBS::getPrioritizedConflict(h_node->paths,
                                          MDDTable[h_node->id]);
  }
  return LibCBS::getPrioritizedConflict(h_node->paths,
                                        MDDTable[h_node->id],
                                        sample);
}

// using MDD
Path ICBS_REFINE::getConstrainedPath(HighLevelNode_p h_node, int id)
{
  Path path = ICBS::getConstrainedPath(h_node, id);
  if (makespan_prioritized && path.size() - 1 > ub_makespan) {
    return {};
  }
  return path;
}
