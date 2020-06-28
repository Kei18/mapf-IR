#include "../include/icbs_refine.hpp"

ICBS_REFINE::ICBS_REFINE(Problem* _P,
                         const Plan& _old_plan,
                         const std::vector<int>& _sample)
  : CBS(_P), ICBS(_P), CBS_REFINE(_P, _old_plan, _sample)
{
}

void ICBS_REFINE::setInitialHighLevelNode(HighLevelNode* n)
{
  if (sample.empty()) {
    ICBS::setInitialHighLevelNode(n);
    return;
  }

  Paths paths(P->getNum());
  MDDs mdds;

  // constraints by fixed agents
  Conflict::Constraints constraints;
  int makespan = old_plan.getMakespan();
  for (int i = 0; i < P->getNum(); ++i) {
    if (inArray(i, sample)) continue;
    for (int t = 1; t <= makespan; ++t) {
      // id is complemented later
      Conflict::Constraint* c_vertex = new Conflict::Constraint
        { -1, t, old_plan.get(t, i), nullptr };
      constraints.push_back(c_vertex);
      // notice! be careful to set swap constraints
      Conflict::Constraint* c_swap = new Conflict::Constraint
        { -1, t, old_plan.get(t-1, i), old_plan.get(t, i) };
      constraints.push_back(c_swap);
    }
  }

  // find paths
  for (int i = 0; i < P->getNum(); ++i) {
    Path path;
    MDD_p mdd;
    if (inArray(i, sample)) {
      path = CBS_REFINE::getInitialPath(i);
      paths.insert(i, path);
      mdd = std::make_shared<MDD>(MDD(path.size()-1, i, P, constraints));
    } else {  // fixed agents
      path = old_paths.get(i);  // fixed agents
      mdd = std::make_shared<MDD>(MDD(path.size()-1, i, P, {}));
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
Path ICBS_REFINE::getConstrainedPath(HighLevelNode* h_node, int id)
{
  // set cost limit
  int prev_cost = h_node->paths.costOfPath(id);
  int cost_limit = ub_soc - h_node->paths.getSOC() + prev_cost;
  cost_limit = std::min(ub_makespan, cost_limit);

  Path path;
  MDD mdd = *(MDDTable[h_node->id][id]);
  Conflict::Constraint* last_constraint = *(h_node->constraints.end()-1);
  mdd.update({ last_constraint });  // check only last
  if (mdd.valid) {  // use mdd as much as possible
    MDDTable[h_node->id][id] = std::make_shared<MDD>(mdd);  // update table
    return mdd.getPath();
  } else {
    int c = std::max(mdd.c, last_constraint->t);
    while (c <= cost_limit) {  // different from original
      ++c;
      MDD_p new_mdd = std::make_shared<MDD>
        (MDD(c, id, P, h_node->constraints));
      if (new_mdd->valid) {
        MDDTable[h_node->id][id] = new_mdd;
        return new_mdd->getPath();
      }
    }
  }
  return {};
}
