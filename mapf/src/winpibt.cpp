#include "../include/winpibt.hpp"

const std::string winPIBT::SOLVER_NAME = "winPIBT";

winPIBT::winPIBT(Problem* _P)
    : Solver(_P),
      window(5),
      occupied_t(G->getNodesSize(), NIL),
      occupied_a(G->getNodesSize(), NIL)
{
  solver_name = winPIBT::SOLVER_NAME + "-" + std::to_string(window);
}

void winPIBT::run()
{
  // prepare
  std::vector<int> ids(P->getNum());
  std::iota(ids.begin(), ids.end(), 0);
  std::vector<Path> paths;
  for (int i = 0; i < P->getNum(); ++i) {
    Node* s = P->getStart(i);
    paths.push_back({s});
    occupied_t[s->id] = 0;
    occupied_a[s->id] = i;
  }

  // the maximum length of paths
  int max_len = 0;

  // start planning
  while (true) {
    if (overCompTime() || max_len > max_timestep) return;

    // far agent is prioritized
    std::sort(ids.begin(), ids.end(), [&](int a, int b) {
      return pathDist(a, *(paths[a].end() - 1)) >
             pathDist(b, *(paths[b].end() - 1));
    });

    for (int j = 0; j < P->getNum(); ++j) {
      const int i = ids[j];
      // trivial case
      if (*(paths[i].end() - 1) == P->getGoal(i)) continue;

      const int buf = paths[i].size() - 1;
      info(" ", "elapsed:", getSolverElapsedTime(),
           ", agent-" + std::to_string(i), "starts planning at t=", buf);

      // get single path
      Path path = getSinglePath(i, paths);
      if (path.empty()) {
        info(" ", "failed to find path");
        return;  // failed
      }

      // reserve paths sequentially
      int t = buf + 1;
      while (t - buf < (int)path.size()) {
        Node* v_now = path[t - buf - 1];
        Node* v_next = path[t - buf];
        // occupied by someone -> priority inheritance
        if (occupied_a[v_next->id] != NIL) {
          // retroactive priority inheritance
          while (occupied_a[v_next->id] != NIL &&
                 occupied_t[v_next->id] < t - 1) {
            auto k = occupied_a[v_next->id];
            info("   ", "retroactive priority inheritance from", i, "->", k,
                 "at v=", v_next->id, ", t=", occupied_t[v_next->id] + 1);
            funcPIBT(k, paths, v_next);
          }

          // priority inheritance with backtracking
          auto k = occupied_a[v_next->id];
          if (k != i && k != NIL) {
            info("   ", "priority inheritance with backtracking from", i, "->",
                 k, "at v=", v_next->id, ", t=", t);
            if (!funcPIBT(k, paths, v_next, v_now)) {
              info("  ", "failed, replanning");
              // replanning
              auto p = getSinglePath(i, paths);
              if (p.empty()) {
                info("  ", "failed to replan the path");
                return;
              }
              // update current path
              path.resize(t - buf);
              for (auto v : p) path.push_back(v);
              // check condition
              if (overCompTime() || (int)path.size() - 1 + buf > max_timestep)
                return;
              continue;
            }
          }
        }

        // secure next step
        updateLoc(i, t, v_now, v_next, paths);
        // check window size
        if (window != -1 && t - buf >= window) break;
        ++t;
      }
    }

    // check goal condition
    bool check_goal_cond = true;
    for (int i = 0; i < P->getNum(); ++i) {
      int path_size = paths[i].size();
      check_goal_cond &= (paths[i][path_size - 1] == P->getGoal(i));
      if (path_size > max_len) max_len = path_size;
    }
    if (check_goal_cond) break;
  }

  // format
  for (int t = 0; t < max_len; ++t) {
    Config c;
    for (int i = 0; i < P->getNum(); ++i) {
      c.push_back((t < (int)paths[i].size()) ? paths[i][t] : *(paths[i].end() - 1));
    }
    solution.add(c);
  }
  solved = sameConfig(solution.last(), P->getConfigGoal());
}

void winPIBT::updateLoc(const int id, const int t, Node* v_now, Node* v_next,
                        std::vector<Path>& paths)
{
  occupied_a[v_now->id] = NIL;
  occupied_a[v_next->id] = id;
  occupied_t[v_next->id] = t;
  paths[id].push_back(v_next);
}

bool winPIBT::funcPIBT(const int id, std::vector<Path>& paths, Node* v_other_to,
                       Node* v_other_from)
{
  const int t = paths[id].size() - 1;
  Node* v_now = paths[id][t];

  // decide next node
  Node* v = chooseNode(id, paths, v_other_to, v_other_from);
  while (v != nullptr) {
    // someone occupies v
    while (occupied_a[v->id] != NIL && occupied_t[v->id] < t - 1) {
      auto k = occupied_a[v->id];
      info("     ", "[one-step] retroactive priority inheritance from", id,
           "->", k, "at v=", v->id, ", t=", occupied_t[v->id]);
      funcPIBT(k, paths, v);
    }
    // priority inheritance with backtracking
    auto k = occupied_a[v->id];
    if (k != id && k != NIL) {
      info("     ", "[one-step] priority inheritance with backtracking from",
           id, "->", k, "at v=", v->id, ", t=", t);
      if (!funcPIBT(k, paths, v, v_now)) {
        // replan
        info("    ", "failed, replanning");
        v = chooseNode(id, paths, v_other_to, v_other_from);
        continue;
      }
    }
    // success to plan next one step
    updateLoc(id, t + 1, v_now, v, paths);
    info("     ", "agent-" + std::to_string(id), "reserves v=", v->id,
         ", t=", t + 1);
    return true;
  }
  // failed to secure node, cope stuck
  updateLoc(id, t + 1, v_now, v_now, paths);
  return false;
}

Node* winPIBT::chooseNode(const int id, std::vector<Path>& paths,
                          Node* v_other_to, Node* v_other_from)
{
  const int t = paths[id].size() - 1;
  Node* v_now = paths[id][t];
  Nodes C = v_now->neighbor;
  C.push_back(v_now);

  // randomize
  std::shuffle(C.begin(), C.end(), *MT);

  Node* v = nullptr;
  for (auto u : C) {
    // avoid future or vertex conflict
    if (occupied_t[u->id] >= t) continue;
    // avoid swap conflict
    if (u == v_other_from && v_now == v_other_to) continue;
    // goal exists -> return immediately
    if (u == P->getGoal(id)) return u;
    // determine the next node
    if (v == nullptr) {
      v = u;
    } else {
      int c_v = pathDist(id, v);
      int c_u = pathDist(id, u);
      if ((c_u < c_v) || (c_u == c_v && occupied_a[v->id] != NIL &&
                          occupied_a[u->id] == NIL)) {
        v = u;
      }
    }
  }
  return v;
}

Path winPIBT::getSinglePath(const int id, std::vector<Path>& paths)
{
  auto g = P->getGoal(id);
  auto s = *(paths[id].end() - 1);
  const int buf = paths[id].size() - 1;
  AstarHeuristics fValue = [&](AstarNode* n) {
    return n->g + pathDist(id, n->v);
  };
  CheckAstarFin checkAstarFin = [&](AstarNode* n) {
    return n->v == g || n->g >= window;
  };

  Nodes config_g = P->getConfigGoal();
  CompareAstarNode compare = [&](AstarNode* a, AstarNode* b) {
    // usual f-value
    if (a->f != b->f) return a->f > b->f;
    // tie-break, avoid goal locations of others
    if (a->v != g && inArray(a->v, config_g)) return true;
    if (b->v != g && inArray(b->v, config_g)) return false;
    // usual g-value
    if (a->g != b->g) return a->g < b->g;
    return false;
  };

  CheckInvalidAstarNode checkInvalidAstarNode = [&](AstarNode* m) {
    // future and vertex conflict
    if (occupied_t[m->v->id] >= m->g + buf) return true;
    return false;
  };
  return getTimedPath(s, g, fValue, compare, checkAstarFin,
                      checkInvalidAstarNode);
}

void winPIBT::setParams(int argc, char* argv[])
{
  struct option longopts[] = {
      {"window", required_argument, 0, 'w'},
      {0, 0, 0, 0},
  };
  optind = 1;  // reset
  int opt, longindex;
  while ((opt = getopt_long(argc, argv, "w:", longopts, &longindex)) != -1) {
    switch (opt) {
      case 'w':
        window = std::atoi(optarg);
        if (window <= 0 && window != -1) halt("invalid window size.");
        solver_name = SOLVER_NAME + "-" + std::to_string(window);
        break;
      default:
        break;
    }
  }
}

void winPIBT::printHelp()
{
  std::cout << winPIBT::SOLVER_NAME << "\n"
            << "  -w --window [INT]             "
            << "window size, default: 5, no window: -1" << std::endl;
}
