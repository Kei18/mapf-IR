#include "../include/whca.hpp"

const std::string WHCA::SOLVER_NAME = "WHCA";
const int WHCA::DEFAULT_WINDOW = 10;

WHCA::WHCA(Problem* _P) : Solver(_P)
{
  window = DEFAULT_WINDOW;
  solver_name = SOLVER_NAME + "-" + std::to_string(window);
}

void WHCA::solve()
{
  start();

  // initialize
  Paths paths(P->getNum());
  for (int i = 0; i < P->getNum(); ++i) {
    paths.insert(i, { P->getStart(i) });
  }

  // initial prioritization
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
  int iteration = 0;
  while (true) {
    info(" ",
         "elapsed:", getSolverElapsedTime(),
         ", timestep:", iteration * window);
    ++iteration;

    bool check_goal_cond = true;
    Paths partial_paths(P->getNum());
    bool invalid = false;
    for (int j = 0; j < ids.size(); ++j) {
      int i = ids[j];
      Node* s = *(paths.get(i).end() - 1);
      Node* g = P->getGoal(i);
      Path path = getPrioritizedPartialPath(i, s, g, partial_paths);
      if (path.empty()) {
        invalid = true;
        break;
      }
      partial_paths.insert(i, path);
      check_goal_cond &= (*(path.end()-1) == g);
    }
    if (invalid) break;
    paths += partial_paths;

    // check goal condition
    if (check_goal_cond) {
      solved = true;
      break;
    }

    // check limitation
    if (overCompTime() || paths.getMakespan() > max_timestep) {
      break;
    }
  }

  solution = pathsToPlan(paths);
  end();
}

Path WHCA::getPrioritizedPartialPath(int id,
                                     Node* s,
                                     Node* g,
                                     const Paths& paths)
{
  // pre processing
  Nodes config_g;
  int max_constraint_time = 0;
  for (int i = 0; i < P->getNum(); ++i) {
    config_g.push_back(P->getGoal(i));
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
      if (a->g != b->g) return a->g < b->g;
      return false;
    };

  // different from HCA*
  CheckAstarFin checkAstarFin =
    [&] (AstarNode* n) {
      return n->g > max_constraint_time &&
        (n->v == g || n->g >= window);
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

  Path path = getTimedPath(s, g,
                           fValue,
                           compare,
                           checkAstarFin,
                           checkInvalidAstarNode);
  // format
  if (!path.empty() && path.size() - 1 > window) path.resize(window+1);
  return path;
}

void WHCA::setParams(int argc, char *argv[]) {
  struct option longopts[] = {
    { "window", required_argument, 0, 'w' },
    { "disable-dist-init", no_argument, 0, 'd' },
    { 0, 0, 0, 0 },
  };
  optind = 1;  // reset
  int opt, longindex;
  while ((opt = getopt_long(argc, argv, "w:d",
                            longopts, &longindex)) != -1) {
    switch (opt) {
    case 'w':
      window = std::atoi(optarg);
      if (window <= 0) halt("invalid window size.");
      solver_name = SOLVER_NAME + "-" + std::to_string(window);
      break;
    case 'd':
      disable_dist_init = true;
      break;
    default:
      break;
    }
  }
}

void WHCA::printHelp() {
  std::cout << WHCA::SOLVER_NAME << "\n"
            << "  -w --window [WINDOW]          "
            << "window size\n"
            << "  -d --disable-dist-init        "
            << "disable initialization of priorities "
            << "using distance from starts to goals"
            << std::endl;
}
