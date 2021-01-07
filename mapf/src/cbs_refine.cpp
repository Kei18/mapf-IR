#include "../include/cbs_refine.hpp"

CBS_REFINE::CBS_REFINE(Problem* _P, const Plan& _old_plan,
                       const std::vector<int>& _modif_list)
    : CBS(_P),
      old_plan(_old_plan),
      old_paths(planToPaths(_old_plan)),
      ub_makespan(_old_plan.getMakespan()),
      ub_soc(_old_plan.getSOC()),
      modif_list(_modif_list),
      makespan_prioritized(false)
{
  // create fixed_agents set
  if (!modif_list.empty()) {
    if (_old_plan.empty()) return;
    for (int i = 0; i < _old_plan.get(0).size(); ++i) {
      if (!inArray(i, modif_list)) fixed_agents.push_back(i);
    }
  }
}

void CBS_REFINE::setInitialHighLevelNode(HighLevelNode_p n)
{
  if (!modif_list.empty()) {
    // create paths only for agents in modif_list
    Paths paths(P->getNum());
    for (int i = 0; i < P->getNum(); ++i) {
      if (inArray(i, modif_list)) {
        // agents in modif_list
        Path path = CBS_REFINE::getInitialPath(i);
        if (path.empty()) {  // failed
          n->valid = false;
          return;
        }
        paths.insert(i, path);
      } else {
        // for fixed agents
        // use the old path
        paths.insert(i, old_paths.get(i));
      }
    }
    n->paths = paths;
  }

  CBS::setInitialHighLevelNode(n);
}

Path CBS_REFINE::getInitialPath(int id)
{
  Node* s = P->getStart(id);
  Node* g = P->getGoal(id);
  Nodes config_g = P->getConfigGoal();

  // pre processing
  int max_constraint_time = 0;
  for (auto i : fixed_agents) {
    for (int t = 1; t <= ub_makespan; ++t) {
      if (old_paths.get(i, t) == g) {
        max_constraint_time = std::max(max_constraint_time, t);
      }
    }
  }

  AstarHeuristics fValue;
  if (pathDist(id) > max_constraint_time) {
    fValue = [&](AstarNode* n) { return n->g + pathDist(n->v, g); };
  } else {
    fValue = [&](AstarNode* n) {
      return std::max(max_constraint_time + 1, n->g + pathDist(n->v, g));
    };
  }

  CompareAstarNode compare = [&](AstarNode* a, AstarNode* b) {
    if (a->f != b->f) return a->f > b->f;
    // IMPORTANT! avoid goal locations of others
    if (a->v != g && inArray(a->v, config_g)) return true;
    if (b->v != g && inArray(b->v, config_g)) return false;
    if (a->g != b->g) return a->g < b->g;
    return false;
  };

  CheckAstarFin checkAstarFin = [&](AstarNode* n) {
    return n->v == g && n->g > max_constraint_time;
  };

  CheckInvalidAstarNode checkInvalidAstarNode = [&](AstarNode* m) {
    if (m->g > ub_makespan) {
      // cut off low-level nodes
      if (makespan_prioritized) return true;
      if (inArray(m->v, config_g) && m->v != g) return true;
      return false;
    }
    // see conflicts with fixed agents
    for (auto i : fixed_agents) {
      // vertex conflicts
      if (m->v == old_paths.get(i, m->g)) return true;
      // swap conflicts
      if (m->v == old_paths.get(i, m->g - 1) &&
          m->p->v == old_paths.get(i, m->g))
        return true;
    }
    return false;
  };

  return getTimedPath(s, g, fValue, compare, checkAstarFin,
                      checkInvalidAstarNode);
}

CBS::CompareHighLevelNodes CBS_REFINE::getObjective()
{
  CompareHighLevelNodes compare = [&](HighLevelNode_p a, HighLevelNode_p b) {
    if (makespan_prioritized && a->makespan != b->makespan)
      return a->makespan > b->makespan;
    if (a->soc != b->soc) return a->soc > b->soc;
    if (a->f != b->f) return a->f > b->f;
    return false;
  };
  return compare;
}

Path CBS_REFINE::getConstrainedPath(HighLevelNode_p h_node, int id)
{
  Node* s = P->getStart(id);
  Node* g = P->getGoal(id);

  // pre processing
  LibCBS::Constraints constraints;
  int max_constraint_time = 0;
  for (auto c : h_node->constraints) {
    if (c->id == id) {
      constraints.push_back(c);
      if (c->v == g && c->u == nullptr) {
        max_constraint_time = std::max(max_constraint_time, c->t);
      }
    }
  }

  AstarHeuristics fValue = [&](AstarNode* n) {
    return n->g + pathDist(n->v, g);
  };

  Nodes config_g = P->getConfigGoal();
  CompareAstarNode compare = [&](AstarNode* a, AstarNode* b) {
    if (a->f != b->f) return a->f > b->f;
    if (a->g != b->g) return a->g < b->g;
    // avoid other's goal
    if (a->v != g && inArray(a->v, config_g)) return true;
    if (b->v != g && inArray(b->v, config_g)) return false;
    // avoid conflict with others
    for (int i = 0; i < P->getNum(); ++i) {
      if (i == id) continue;
      if (a->g <= h_node->makespan && h_node->paths.get(i, a->g) == a->v)
        return true;
      if (b->g <= h_node->makespan && h_node->paths.get(i, b->g) == b->v)
        return false;
    }
    return false;
  };

  CheckAstarFin checkAstarFin = [&](AstarNode* n) {
    return n->v == g && n->g > max_constraint_time;
  };

  // different from CBS
  int prev_cost = h_node->paths.costOfPath(id);
  int cost_limit = ub_soc - h_node->paths.getSOC() + prev_cost;
  cost_limit = std::min(ub_makespan, cost_limit);

  CheckInvalidAstarNode checkInvalidAstarNode = [&](AstarNode* m) {
    if (m->f > cost_limit) return true;
    // check constraints
    for (auto c : constraints) {
      if (m->g == c->t && m->v == c->v) {
        // vertex or swap conflict
        if (c->u == nullptr || c->u == m->p->v) return true;
      }
    }
    // check collisions with fixed agents
    for (auto i : fixed_agents) {
      // vertex conflicts
      if (m->v == old_paths.get(i, m->g)) return true;
      // swap conflicts
      if (m->v == old_paths.get(i, m->g - 1) &&
          m->p->v == old_paths.get(i, m->g)) {
        return true;
      }
    }
    return false;
  };

  return getTimedPath(s, g, fValue, compare, checkAstarFin,
                      checkInvalidAstarNode);
}

void CBS_REFINE::setParams(int argc, char* argv[])
{
  struct option longopts[] = {
      {"makespan-prioritized", no_argument, 0, 'p'},
      {0, 0, 0, 0},
  };
  optind = 1;  // reset
  int opt, longindex;
  std::string s, s_tmp;

  while ((opt = getopt_long(argc, argv, "p", longopts, &longindex)) != -1) {
    switch (opt) {
      case 'p':
        makespan_prioritized = true;
        break;
      default:
        break;
    }
  }
}
