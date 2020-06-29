#include "../include/icbs_refine.hpp"

ICBS_REFINE::ICBS_REFINE(Problem* _P,
                         const Plan& _old_plan,
                         const std::vector<int>& _sample)
  : CBS(_P), ICBS(_P), CBS_REFINE(_P, _old_plan, _sample)
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

  // find paths
  for (int i = 0; i < P->getNum(); ++i) {
    Path path;
    LibCBS::MDD_p mdd;
    if (inArray(i, sample)) {
      path = CBS_REFINE::getInitialPath(i);
      if (path.empty()) {
        n->valid = false;
        return;
      }
      paths.insert(i, path);
      mdd = std::make_shared<LibCBS::MDD>
        (LibCBS::MDD(path.size()-1, i, P, constraints));
    } else {  // fixed agents
      path = old_paths.get(i);  // fixed agents
      mdd = std::make_shared<LibCBS::MDD>
        (LibCBS::MDD(path.size()-1, i, P, {}));
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
  n->f = paths.countConflict();
  n->valid = true;  // valid
}

// using MDD
Path ICBS_REFINE::getConstrainedPath(HighLevelNode_p h_node, int id)
{
  // set cost limit
  int prev_cost = h_node->paths.costOfPath(id);
  int cost_limit = ub_soc - h_node->paths.getSOC() + prev_cost;
  cost_limit = std::min(ub_makespan, cost_limit);

  Path path;
  LibCBS::MDD mdd = *(MDDTable[h_node->id][id]);
  LibCBS::Constraint_p last_constraint = *(h_node->constraints.end()-1);
  mdd.update({ last_constraint });  // check only last
  if (mdd.valid) {  // use mdd as much as possible
    // update table
    MDDTable[h_node->id][id] = std::make_shared<LibCBS::MDD>(mdd);
    return mdd.getPath();
  } else {
    int c = std::max(mdd.c, last_constraint->t);
    while (c <= cost_limit) {  // different from original
      ++c;
      LibCBS::MDD_p new_mdd(new LibCBS::MDD(c, id, P, h_node->constraints));
      if (new_mdd->valid) {
        MDDTable[h_node->id][id] = new_mdd;
        return new_mdd->getPath();
      }
    }
  }
  return {};
}
