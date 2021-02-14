#include "../include/whca.hpp"

const std::string WHCA::SOLVER_NAME = "WHCA";
const int WHCA::DEFAULT_WINDOW = 10;

WHCA::WHCA(Problem* _P) : Solver(_P), table_goals(G->getNodesSize(), false)
{
  window = DEFAULT_WINDOW;
  solver_name = SOLVER_NAME + "-" + std::to_string(window);
}

void WHCA::run()
{
  // initialize
  Paths paths(P->getNum());
  for (int i = 0; i < P->getNum(); ++i) {
    paths.insert(i, {P->getStart(i)});
    table_goals[P->getGoal(i)->id] = true;
  }

  // initial prioritization, far agent is prioritized
  std::vector<int> ids(P->getNum());
  std::iota(ids.begin(), ids.end(), 0);

  // start planning
  int iteration = 0;
  while (true) {
    info(" ", "elapsed:", getSolverElapsedTime(),
         ", timestep:", iteration * window);
    ++iteration;

    if (!disable_dist_init) {
      std::sort(ids.begin(), ids.end(),
                [&](int a, int b)
                { return pathDist(a, paths.last(a)) > pathDist(b, paths.last(b)); });
    }

    bool check_goal_cond = true;
    Paths partial_paths(P->getNum());
    bool invalid = false;
    for (int j = 0; j < P->getNum(); ++j) {
      int i = ids[j];
      Node* s = *(paths.get(i).end() - 1);
      Node* g = P->getGoal(i);
      Path path = getPrioritizedPartialPath(i, s, g, partial_paths);
      if (path.empty()) {  // failed
        invalid = true;
        info("  ", "failed to find a path");
        break;
      }
      partial_paths.insert(i, path);
      check_goal_cond &= (*(path.end() - 1) == g);
    }
    if (invalid) break;
    paths += partial_paths;

    // clear cache
    clearPathTable(partial_paths);

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
}

Path WHCA::getPrioritizedPartialPath(int id, Node* s, Node* g,
                                     const Paths& paths)
{
  const int makespan = paths.getMakespan();

  // pre processing
  int max_constraint_time = 0;
  for (int i = 0; i < P->getNum(); ++i) {
    Path p = paths.get(i);
    if (p.empty() || i == id) continue;
    for (int t = 0; t <= makespan; ++t) {
      if (paths.get(i, t) == g) {
        max_constraint_time = std::max(t, max_constraint_time);
      }
    }
  }

  // in this case, the greedy f-value fails a lot, different from HCA*
  AstarHeuristics fValue = [&](AstarNode* n) {
    return n->g + pathDist(id, n->v);
  };

  CompareAstarNode compare = [&](AstarNode* a, AstarNode* b) {
    if (a->f != b->f) return a->f > b->f;
    // tie-break, avoid goal locations of others
    if (a->v != g && table_goals[a->v->id]) return true;
    if (b->v != g && table_goals[b->v->id]) return false;
    if (a->g != b->g) return a->g < b->g;
    return false;
  };

  // different from HCA*
  CheckAstarFin checkAstarFin = [&](AstarNode* n) {
    return (n->v == g && n->g > max_constraint_time) || n->g >= window;
  };

  // fast collision checking
  CheckInvalidAstarNode checkInvalidAstarNode = [&](AstarNode* m) {
    if (m->g > window) return true;
    // last node
    if (makespan > 0) {
      if (m->g > makespan) {
        if (PATH_TABLE[makespan][m->v->id] != Solver::NIL) return true;
      } else {
        // vertex conflict
        if (PATH_TABLE[m->g][m->v->id] != Solver::NIL) return true;
        // swap conflict
        if (PATH_TABLE[m->g][m->p->v->id] != Solver::NIL &&
            PATH_TABLE[m->g - 1][m->v->id] == PATH_TABLE[m->g][m->p->v->id])
          return true;
      }
    }
    return false;
  };

  Path path =
      getTimedPath(s, g, fValue, compare, checkAstarFin, checkInvalidAstarNode);
  const int path_size = path.size();
  // format
  if (!path.empty() && path_size - 1 > window) path.resize(window + 1);

  // update path table
  updatePathTableWithoutClear(id, path, paths);

  return path;
}

void WHCA::setParams(int argc, char* argv[])
{
  struct option longopts[] = {
      {"window", required_argument, 0, 'w'},
      {"disable-dist-init", no_argument, 0, 'd'},
      {0, 0, 0, 0},
  };
  optind = 1;  // reset
  int opt, longindex;
  while ((opt = getopt_long(argc, argv, "w:d", longopts, &longindex)) != -1) {
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

void WHCA::printHelp()
{
  std::cout << WHCA::SOLVER_NAME << "\n"
            << "  -w --window [INT]             "
            << "window size\n"
            << "  -d --disable-dist-init        "
            << "disable initialization of priorities "
            << "using distance from starts to goals" << std::endl;
}
