#include "../include/hca.hpp"

const std::string HCA::SOLVER_NAME = "HCA";


HCA::HCA(Problem* _P) : Solver(_P)
{
  solver_name = HCA::SOLVER_NAME;
}

void HCA::solve()
{
  start();

  Paths paths(P->getNum());

  // prioritization
  // far agent is prioritized
  std::vector<int> ids(P->getNum());
  std::iota(ids.begin(), ids.end(), 0);
  if (!disable_dist_init) {
    std::sort(ids.begin(), ids.end(),
              [&] (int a, int b)
              { return pathDist(P->getStart(a), P->getGoal(a))
                  > pathDist(P->getStart(b), P->getGoal(b)); });
  }

  // start planning
  bool invalid = false;
  for (int j = 0; j < ids.size(); ++j) {
    int i = ids[j];
    info(" ",
         "elapsed:", getSolverElapsedTime(),
         ", agent-" + std::to_string(i),
         "starts planning,", j+1, "/", P->getNum());
    Nodes path = getPrioritizedPath(i, paths);
    if (path.empty()) {
      invalid = true;
      break;
    }
    paths.insert(i, path);

    // check limitation
    if (overCompTime() || paths.getMakespan() > max_timestep) {
      invalid = true;
      break;
    }
  }

  if (!invalid) {  // success
    solution = pathsToPlan(paths);
    solved = true;
  }
  end();
}

Path HCA::getPrioritizedPath(int id, const Paths& paths)
{
  Node* s = P->getStart(id);
  Node* g = P->getGoal(id);
  return getPrioritizedPath(id, s, g, paths);
}

// get single agent path
Path HCA::getPrioritizedPath(int id,
                             Node* s,
                             Node* g,
                             const Paths& paths)
{
  // pre processing
  Nodes config_s = P->getConfigStart();
  Nodes config_g = P->getConfigGoal();
  int max_constraint_time = 0;
  for (int i = 0; i < P->getNum(); ++i) {
    Path p = paths.get(i);
    if (p.empty()) continue;
    for (int t = 0; t < p.size(); ++t) {
      if (p[t] == g) {
        max_constraint_time = std::max(t, max_constraint_time);
      }
    }
  }

  AstarHeuristics fValue =
    [&] (AstarNode* n) { return n->g + pathDist(n->v, g); };

  CompareAstarNode compare =
    [&] (AstarNode* a, AstarNode* b) {
      if (a->f != b->f) return a->f > b->f;
      // avoid goal locations of others
      if (a->v != g && inArray(a->v, config_g)) return true;
      if (b->v != g && inArray(b->v, config_g)) return false;
      // avoid start locations
      if (a->v != s && inArray(a->v, config_s)) return true;
      if (b->v != s && inArray(b->v, config_s)) return false;
      if (a->g != b->g) return a->g < b->g;
      return false;
    };

  CheckAstarFin checkAstarFin =
    [&] (AstarNode* n) {
      return n->v == g && n->g > max_constraint_time;
    };

  CheckInvalidAstarNode checkInvalidAstarNode =
    [&] (AstarNode* m) {
      for (int i = 0; i < P->getNum(); ++i) {
        Path p = paths.get(i);
        if (p.empty()) continue;
        // last node
        if (m->g >= p.size()) {
          if (*(p.end()-1) == m->v) return true;
          continue;
        }
        // vertex conflict
        if (p[m->g] == m->v) return true;
        // swap conflict
        if (p[m->g] == m->p->v && p[m->g-1] == m->v) return true;
      }
      return false;
    };

  return getTimedPath(s, g,
                      fValue,
                      compare,
                      checkAstarFin,
                      checkInvalidAstarNode);
}

void HCA::setParams(int argc, char *argv[]) {
  struct option longopts[] = {
    { "disable-dist-init", no_argument, 0, 'd' },
    { 0, 0, 0, 0 },
  };
  optind = 1;  // reset
  int opt, longindex;
  while ((opt = getopt_long(argc, argv, "d",
                            longopts, &longindex)) != -1) {
    switch (opt) {
    case 'd':
      disable_dist_init = true;
      break;
    default:
      break;
    }
  }
}

void HCA::printHelp() {
  std::cout << HCA::SOLVER_NAME << "\n"
            << "  -d --disable-dist-init"
            << "        "
            << "disable initialization of priorities "
            << "using distance from starts to goals"
            << std::endl;
}
