#include "../include/winpibt.hpp"

const std::string winPIBT::SOLVER_NAME = "winPIBT";

winPIBT::winPIBT(Problem* _P)
  : Solver(_P),
    occupied_t(G->getNodesSize(), NIL),
    occupied_a(G->getNodesSize(), NIL)
{
  solver_name = winPIBT::SOLVER_NAME;
}

void winPIBT::run()
{

  // near agent is prioritized
  std::vector<int> ids(P->getNum());
  std::iota(ids.begin(), ids.end(), 0);
  std::sort(ids.begin(), ids.end(),
            [&](int a, int b) { return pathDist(a) < pathDist(b); });

  // prepare
  std::vector<Path> paths;
  for (int i = 0; i < P->getNum(); ++i) {
    Node* s = P->getStart(i);
    paths.push_back({s});
    occupied_t[s->id] = 0;
    occupied_a[s->id] = i;
  }

  int max_len = 0;

  // start planning
  while (true) {
    if (overCompTime() || max_len > max_timestep) return;

    for (int j = 0; j < P->getNum(); ++j) {
      const int i = ids[j];
      const int buf = paths[i].size() - 1;
      info(" ", "elapsed:", getSolverElapsedTime(),
           ", agent-" + std::to_string(i), "starts planning at t=", buf,
           ", progress:", j + 1, "/", P->getNum());
      // trivial case
      if (*(paths[i].end()-1) == P->getGoal(i)) continue;
      Path path = getSinglePath(i, paths);

      int t = buf + 1;
      while (t-buf < path.size()) {
        Node* v_now  = path[t-buf-1];
        Node* v_next = path[t-buf];
        // occupied by someone -> priority inheritance
        if (occupied_a[v_next->id] != NIL) {
          while (occupied_a[v_next->id] != NIL && occupied_t[v_next->id] < t-1) {
            auto k = occupied_a[v_next->id];
            info("   ", "retroactive priority inheritance from", i, "->", k,
                 "at v=", v_next->id, ", t=", occupied_t[v_next->id]+1);
            funcPIBT(k, paths, v_next);
          }
          auto k = occupied_a[v_next->id];
          if (k != i && k != NIL) {
            info("   ", "priority inheritance with backtracking from", i, "->", k,
                 "at v=", v_next->id, ", t=", t);
            if (!funcPIBT(k, paths, v_next, v_now)) {
              info("  ", "failed, replanning");
              // replanning
              auto p = getSinglePath(i, paths);
              // update current path
              path.resize(t-buf);
              for (auto v : p) path.push_back(v);
              if (overCompTime() || max_len > max_timestep) return;
              continue;
            }
          }
        }

        // secure next step
        updateLoc(i, t, v_now, v_next, paths);
        ++t;
      }
    }

    bool check_goal_cond = true;
    for (int i = 0; i < P->getNum(); ++i) {
      int path_size = paths[i].size();
      check_goal_cond &= (paths[i][path_size-1] == P->getGoal(i));
      if (path_size > max_len) max_len = path_size;
    }
    if (check_goal_cond) break;
  }

  // format
  for (int t = 0; t < max_len; ++t) {
    Config c;
    for (int i = 0; i < P->getNum(); ++i) {
      c.push_back((t < paths[i].size()) ? paths[i][t] : *(paths[i].end()-1));
    }
    solution.add(c);
  }
  solved = sameConfig(solution.last(), P->getConfigGoal());
}

void winPIBT::updateLoc(const int id, const int t, Node* v_now, Node* v_next, std::vector<Path>& paths)
{
  occupied_a[v_now->id] = NIL;
  occupied_a[v_next->id] = id;
  occupied_t[v_next->id] = t;
  paths[id].push_back(v_next);
}

bool winPIBT::funcPIBT(const int id, std::vector<Path>& paths, Node* v_other_to, Node* v_other_from)
{
  const int t = paths[id].size()-1;
  Node* v_now = paths[id][t];

  // decide next node
  Node* v = chooseNode(id, paths, v_other_to, v_other_from);
  while (v != nullptr) {
    // someone occupies v
    while (occupied_a[v->id] != NIL && occupied_t[v->id] < t-1) {
      auto k = occupied_a[v->id];
      info("     ", "[one-step] retroactive priority inheritance from", id, "->", k,
           "at v=", v->id, ", t=", occupied_t[v->id]);
      funcPIBT(k, paths, v);
    }
    auto k = occupied_a[v->id];
    if (k != id && k != NIL) {
      info("     ", "[one-step] priority inheritance with backtracking from", id, "->", k,
           "at v=", v->id, ", t=", t);
      if (!funcPIBT(k, paths, v, v_now)) {
        // replan
        info("    ", "failed, replanning");
        v = chooseNode(id, paths, v_other_to, v_other_from);
        continue;
      }
    }
    // success to plan next one step
    updateLoc(id, t+1, v_now, v, paths);
    info("     ", "agent-"+std::to_string(id), "reserves v=", v->id, ", t=", t+1);
    return true;
  }
  // failed to secure node, cope stuck
  updateLoc(id, t+1, v_now, v_now, paths);
  return false;
}

Node* winPIBT::chooseNode(const int id, std::vector<Path>& paths, Node* v_other_to, Node* v_other_from)
{
  const int t = paths[id].size()-1;
  Node* v_now = paths[id][t];
  Nodes C;
  Nodes C_pre = v_now->neighbor;
  C_pre.push_back(v_now);
  for (auto v : C_pre) {
    // avoid future or vertex conflict
    if (occupied_t[v->id] >= t) continue;
    // avoid swap conflict
    if (v == v_other_from && v_now == v_other_to) continue;
    C.push_back(v);
  }

  // correspond to stuck
  if (C.empty()) return nullptr;

  // randomize
  std::shuffle(C.begin(), C.end(), *MT);

  // pickup one node
  Node* v = *std::min_element(C.begin(), C.end(), [&](Node* v, Node* u) {
    // path distance
    int c_v = pathDist(id, v);
    int c_u = pathDist(id, u);
    if (c_v != c_u) return c_v < c_u;
    // occupancy
    int o_v = (int)(occupied_a[v->id] != NIL);
    int o_u = (int)(occupied_a[u->id] != NIL);
    if (o_v != o_u) return o_v < o_u;
    return false;
  });
  return v;
}

Path winPIBT::getSinglePath(const int id, std::vector<Path>& paths)
{
  auto g = P->getGoal(id);
  auto s = *(paths[id].end()-1);
  const int buf = paths[id].size() - 1;
  AstarHeuristics fValue = [&](AstarNode* n) { return n->g + buf + pathDist(id, n->v); };
  CheckAstarFin checkAstarFin = [&](AstarNode* n) { return n->v == g; };

  Nodes config_g = P->getConfigGoal();
  CompareAstarNode compare = [&](AstarNode* a, AstarNode* b) {
    // usual f-value
    if (a->f != b->f) return a->f > b->f;
    // tie-break, avoid goal locations of others
    if (a->v != g && inArray(a->v, config_g)) return true;
    if (b->v != g && inArray(b->v, config_g)) return false;
    // occupancy, not used yet
    // if (occupied_t[a->v->id] == NIL) return false;
    // if (occupied_t[b->v->id] == NIL) return true;
    // usual g-value
    if (a->g != b->g) return a->g < b->g;
    return false;
  };

  CheckInvalidAstarNode checkInvalidAstarNode = [&](AstarNode* m) {
    const int t = m->g + buf;
    // future and vertex conflict
    if (occupied_t[m->v->id] >= t) return true;
    // check swap conflict
    for (int i = 0; i < P->getNum(); ++i) {
      if (paths[i].size()-1 < t) continue;
      if (paths[i][t] == m->p->v && paths[i][t-1] == m->v) return true;
    }
    return false;
  };
  return getTimedPath(s, g, fValue, compare, checkAstarFin, checkInvalidAstarNode);
}


void winPIBT::setParams(int argc, char* argv[])
{
}

void winPIBT::printHelp()
{
  std::cout << winPIBT::SOLVER_NAME << "\n"
            << "  (no option)"
            << std::endl;
}
