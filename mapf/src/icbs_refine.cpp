#include "../include/icbs_refine.hpp"

ICBS_REFINE::ICBS_REFINE(Problem* _P,
                         const Plan& _old_plan,
                         const std::vector<int>& _sample)
  : ICBS(_P), SolverRefine(_old_plan, _sample)
{
}

CBS::CompareHighLevelNodes ICBS_REFINE::getObjective()
{
  CompareHighLevelNodes compare =
    [] (HighLevelNode* a, HighLevelNode* b)
    {
      if (a->makespan != b->makespan) return a->makespan > b->makespan;
      if (a->soc != b->soc) return a->soc > b->soc;
      if (a->f != b->f) return a->f > b->f;  // tie-breaker
      return false;
    };
  return compare;
}

void ICBS_REFINE::setInitialHighLevelNode(HighLevelNode* n)
{
  Paths paths(P->getNum());
  MDDs mdds;

  // constraints by fixed agents
  Conflict::Constraints constraints;
  if (!sample.empty()) {
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
  }

  for (int i = 0; i < P->getNum(); ++i) {
    Path path;
    MDD_p mdd;
    if (sample.empty()) {
      path = CBS::getInitialPath(i);
      paths.insert(i, path);
      MDD tmp = MDD(path.size()-1, i, P, constraints);
      mdd = std::make_shared<MDD>(tmp);
    } else if (inArray(i, sample)) {
      path = ICBS_REFINE::getInitialPath(i);
      paths.insert(i, path);
      MDD tmp = MDD(path.size()-1, i, P, constraints);
      mdd = std::make_shared<MDD>(tmp);
      if(!mdd->valid) halt("failed to initialize");
    } else {
      path = old_paths.get(i);  // fixed agents
      MDD tmp = MDD(path.size()-1, i, P, {});
      mdd = std::make_shared<MDD>(tmp);
    }
    paths.insert(i, path);
    mdds.push_back(mdd);
  }

  n->id = 0;
  n->paths = paths;
  n->constraints = constraints;
  n->makespan = paths.getMakespan();
  n->soc = paths.getSOC();
  n->f = Conflict::countConflict(paths);
  n->valid = true;  // valid

  MDDTable[n->id] = mdds;
}

void ICBS_REFINE::invoke(HighLevelNode* h_node, int id)
{
  // sampling
  if (!sample.empty() && !inArray(id, sample)) {
    h_node->valid = false;
    halt("never happen");
    return;
  }

  int prev_cost = h_node->paths.costOfPath(id);
  int cost_limit = ub_soc - h_node->paths.getSOC() + prev_cost;
  cost_limit = std::min(ub_makespan, cost_limit);

  Path path;
  MDD mdd = *(MDDTable[h_node->id][id]);
  Conflict::Constraint* last_constraint = *(h_node->constraints.end()-1);
  mdd.update({ last_constraint });  // check only last

  if (mdd.valid) {  // use mdd as much as possible
    path = mdd.getPath();
    if (path.empty()) halt("invalid MDD");
    MDDTable[h_node->id][id] = std::make_shared<MDD>(mdd);  // update table
  } else {
    int c = std::max(mdd.c, last_constraint->t);
    while (c <= cost_limit) {
      ++c;
      MDD tmp_mdd = MDD(c, id, P, h_node->constraints);
      if (tmp_mdd.valid) {
        path = tmp_mdd.getPath();
        MDD_p new_mdd = std::make_shared<MDD>(tmp_mdd);
        MDDTable[h_node->id][id] = new_mdd;
        break;
      }
    }
  }

  // check cost_limit
  if (path.empty() || path.size() - 1 > cost_limit) {
    h_node->valid = false;
    return;
  }

  Paths paths = h_node->paths;
  paths.insert(id, path);
  // update conflicts counts
  h_node->f = h_node->f
    - Conflict::countConflict(id, h_node->paths.get(id), h_node->paths)
    + Conflict::countConflict(id, paths.get(id), h_node->paths);
  h_node->paths = paths;
  h_node->makespan = h_node->paths.getMakespan();
  h_node->soc = h_node->paths.getSOC();
}

Path ICBS_REFINE::getInitialPath(int id)
{
  Node* s = P->getStart(id);
  Node* g = P->getGoal(id);
  Nodes config_g = P->getConfigGoal();

  int max_constraint_time = 0;
  for (auto i : fixed_agents) {
    for (int t = 1; t <= ub_makespan; ++t) {
      if (old_paths.get(i, t) == g) {
        max_constraint_time = std::max(max_constraint_time, t);
      }
    }
  }

  AstarHeuristics fValue =
    [&] (AstarNode* n) { return n->g + pathDist(n->v, g); };

  CompareAstarNode compare =
    [&] (AstarNode* a, AstarNode* b) {
      if (a->f != b->f) return a->f > b->f;
      // [IMPORTANT!] avoid goal locations of others
      if (a->v != g && inArray(a->v, config_g)) return true;
      if (b->v != g && inArray(b->v, config_g)) return false;
      if (a->g != b->g) return a->g < b->g;
      return false;
    };

  CheckAstarFin checkAstarFin =
    [&] (AstarNode* n) {
      return n->v == g && n->g > max_constraint_time;
    };

  CheckInvalidAstarNode checkInvalidAstarNode =
    [&] (AstarNode* m) {
      if (m->g > ub_makespan) return true;
      for (auto i : fixed_agents) {
        // vertex conflicts
        if (m->v == old_paths.get(i, m->g)) return true;
        // swap conflicts
        if (m->v == old_paths.get(i, m->g-1) &&
            m->p->v == old_paths.get(i, m->g)) return true;
      }
      return false;
    };

  return getTimedPath(s, g,
                      fValue,
                      compare,
                      checkAstarFin,
                      checkInvalidAstarNode);
}
