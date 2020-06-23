#include "../include/cbs_refine.hpp"

CBS_REFINE::CBS_REFINE(Problem* _P,
                       int _ub_makespan,
                       int _ub_soc)
  : CBS(_P), ub_makespan(_ub_makespan), ub_soc(_ub_soc)
{
}

CBS::CompareHighLevelNodes CBS_REFINE::getObjective()
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

Path CBS_REFINE::getConstrainedPath(HighLevelNode* h_node, int id)
{
  Node* s = P->getStart(id);
  Node* g = P->getGoal(id);

  Constraints constraints;
  int max_constraint_time = 0;
  for (auto c : h_node->constraints) {
    if (c->id == id) {
      constraints.push_back(c);
      if (c->v == g && c->u == nullptr) {
        max_constraint_time = std::max(max_constraint_time, c->t);
      }
    }
  }

  int prev_cost = h_node->paths.costOfPath(id);
  int cost_limit = ub_soc - h_node->paths.getSOC() + prev_cost;
  cost_limit = std::min(ub_makespan, cost_limit);

  AstarHeuristics fValue =
    [&] (AstarNode* n) {
      return n->g + pathDist(n->v, g);
    };

  CompareAstarNode compare =
    [&] (AstarNode* a, AstarNode* b) {
      if (a->f != b->f) return a->f > b->f;
      if (a->g != b->g) return a->g < b->g;
      // avoid conflict with others
      for (int i = 0; i < P->getNum(); ++i) {
        if (i == id) continue;
        if (a->g <= h_node->makespan &&
            h_node->paths.get(i, a->g) == a->v) return true;
        if (b->g <= h_node->makespan &&
            h_node->paths.get(i, b->g) == b->v) return false;
      }
      return false;
    };

  CheckAstarFin checkAstarFin =
    [&] (AstarNode* n) {
      return n->v == g && n->g > max_constraint_time;
    };

  // different from CBS
  CheckInvalidAstarNode checkInvalidAstarNode =
    [&] (AstarNode* m) {
      if (m->f > cost_limit) return true;
      for (auto c : constraints) {
        if (m->g == c->t && m->v == c->v) {
          // vertex or swap conflict
          if (c->u == nullptr || c->u == m->p->v) return true;
        }
      }
      return false;
    };

  return getTimedPath(s, g,
                      fValue,
                      compare,
                      checkAstarFin,
                      checkInvalidAstarNode);
}
