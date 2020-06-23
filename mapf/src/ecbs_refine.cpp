#include "../include/ecbs_refine.hpp"

ECBS_REFINE::ECBS_REFINE(Problem* _P,
                         int _ub_makespan,
                         int _ub_soc)
  : ECBS(_P), ub_makespan(_ub_makespan), ub_soc(_ub_soc)
{
}

ECBS::CompareHighLevelNodeECBS ECBS_REFINE::getMainObjective() {
  CompareHighLevelNodeECBS compare =
    [&] (HighLevelNodeECBS* a, HighLevelNodeECBS* b) {
      if (a->makespan != b->makespan) return a->makespan > b->makespan;
      if (a->soc != b->soc) return a->soc > b->soc;
      return false;
    };
  return compare;
}

ECBS::CompareHighLevelNodeECBS ECBS_REFINE::getFocalObjective() {
  CompareHighLevelNodeECBS compare =
    [&] (HighLevelNodeECBS* a, HighLevelNodeECBS* b) {
      if (a->f != b->f) return a->f > b->f;
      if (a->makespan != b->makespan) return a->makespan > b->makespan;
      if (a->soc != b->soc) return a->soc > b->soc;
      return false;
    };
  return compare;
}

void ECBS_REFINE::invoke(HighLevelNodeECBS* h_node, int id)
{
  auto res = getFocalPath(h_node, id);
  Path path = std::get<0>(res);
  int f_min = std::get<1>(res); // lower bound

  if (path.empty()) {
    h_node->valid = false;
    return;
  }

  Paths paths = h_node->paths;
  paths.insert(id, path);
  h_node->paths = paths;
  h_node->makespan = h_node->paths.getMakespan();
  h_node->soc = h_node->paths.getSOC();
  h_node->f = countConflict(h_node->paths);
  h_node->LB = h_node->LB - h_node->f_mins[id] + f_min;
  h_node->f_mins[id] = f_min;

  if (h_node->soc > ub_soc) h_node->valid = false;
}

std::tuple<Path, int> ECBS_REFINE::getFocalPath(HighLevelNodeECBS* h_node,
                                                int id)
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

  FocalHeuristics f1Value =
    [&] (FocalNode* n) {
      return n->g + pathDist(n->v, g);
    };

  FocalHeuristics f2Value =
    [&] (FocalNode* n) {
      return countConflict(id, getPathFromFocalNode(n), h_node->paths);
    };

  CompareFocalNode compareOPEN =
    [&] (FocalNode* a, FocalNode* b) {
      if (a->f1 != b->f1) return a->f1 > b->f1;
      return false;
    };

  CompareFocalNode compareFOCAL =
    [&] (FocalNode* a, FocalNode* b) {
      if (a->f2 != b->f2) return a->f2 > b->f2;
      if (a->f1 != b->f1) return a->f1 > b->f1;
      if (a->g != b->g) return a->g < b->g;
      return false;
    };

  CheckFocalFin checkFocalFin =
    [&] (FocalNode* n) {
      return n->v == g && n->g > max_constraint_time;
    };

  // different from ECBS
  int prev_cost = h_node->paths.costOfPath(id);
  int cost_limit = ub_soc - h_node->paths.getSOC() + prev_cost;
  cost_limit = std::min(ub_makespan, cost_limit);

  CheckInvalidFocalNode checkInvalidFocalNode =
    [&] (FocalNode* m) {
      if (m->f1 > cost_limit) return true;
      for (auto c : constraints) {
        if (m->g == c->t && m->v == c->v) {
          // vertex or swap conflict
          if (c->u == nullptr || c->u == m->p->v) return true;
        }
      }
      return false;
    };
  return getTimedPathByFocalSearch(s,
                                   g,
                                   sub_optimality,
                                   f1Value,
                                   f2Value,
                                   compareOPEN,
                                   compareFOCAL,
                                   checkFocalFin,
                                   checkInvalidFocalNode);
}
