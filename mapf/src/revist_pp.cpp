#include "../include/revisit_pp.hpp"

const std::string RevisitPP::SOLVER_NAME = "RevisitPP";

RevisitPP::RevisitPP(Problem* _P) : Solver(_P) { solver_name = RevisitPP::SOLVER_NAME; }

void RevisitPP::run()
{
  Paths paths(P->getNum());

  // prioritization, near agent is prioritized
  std::vector<int> ids(P->getNum());
  std::iota(ids.begin(), ids.end(), 0);
  if (!disable_dist_init) {
    std::sort(ids.begin(), ids.end(),
              [&](int a, int b) { return pathDist(a) < pathDist(b); });
  }

  // constraints of starts
  std::vector<std::tuple<Node*, int>> constraints;
  for (auto i : ids) constraints.push_back(std::make_tuple(P->getStart(i), -1));

  // start planning
  bool invalid = false;
  for (int j = 0; j < ids.size(); ++j) {
    int i = ids[j];
    info(" ", "elapsed:", getSolverElapsedTime(),
         ", agent-" + std::to_string(i), "starts planning,",
         "init-dist:", pathDist(i), ", progress:", j + 1, "/", P->getNum());

    // remove current start
    constraints.erase(constraints.begin());

    // get path
    Nodes path = getPrioritizedPath(i, paths, constraints);
    if (path.empty()) {  // failed
      invalid = true;
      break;
    }

    // update reservation table
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
}

Path RevisitPP::getPrioritizedPath(int id, const Paths& paths,
                                   const std::vector<std::tuple<Node*, int>> constraints)
{
  Node* s = P->getStart(id);
  Node* g = P->getGoal(id);
  return getPrioritizedPath(id, s, g, paths, constraints);
}

// get single agent path
// failed -> return {}
Path RevisitPP::getPrioritizedPath(int id, Node* s, Node* g, const Paths& paths,
                                   const std::vector<std::tuple<Node*, int>> constraints)
{
  const int ideal_dist = pathDist(id);

  // max timestep that another agent uses the goal
  int max_constraint_time = paths.getMaxConstraintTime(id, g, ideal_dist);

  // setup functions
  AstarHeuristics fValue;
  if (ideal_dist > max_constraint_time) {
    fValue = [&](AstarNode* n) { return n->g + pathDist(id, n->v); };
  } else {
    // when someone occupies its goal
    fValue = [&](AstarNode* n) {
      return std::max(max_constraint_time + 1, n->g + pathDist(id, n->v));
    };
  }

  CheckAstarFin checkAstarFin = [&](AstarNode* n) {
    return n->v == g && n->g > max_constraint_time;
  };

  Nodes config_s = P->getConfigStart();
  Nodes config_g = P->getConfigGoal();

  const int makespan = paths.getMakespan();
  const int num_agents = P->getNum();

  CheckInvalidAstarNode checkInvalidAstarNode = [&](AstarNode* m) {
    if (m->g > max_timestep) return true;

    for (int i = 0; i < num_agents; ++i) {
      if (i == id || paths.empty(i)) continue;
      // last node
      if (m->g > makespan) {
        if (paths.get(i, makespan) == m->v) return true;
        continue;
      }
      // vertex conflict
      if (paths.get(i, m->g) == m->v) return true;
      // swap conflict
      if (paths.get(i, m->g) == m->p->v && paths.get(i, m->g-1) == m->v) return true;

      // check additional constraints
      for (auto c : constraints) {
        const int t = std::get<1>(c);
        if (m->v == std::get<0>(c) && (t == -1 || t == m->g)) return true;
      }
    }

    return false;
  };

  return getTimedPath(s, g, fValue, compareAstarNodeBasic, checkAstarFin, checkInvalidAstarNode);
}

void RevisitPP::setParams(int argc, char* argv[])
{
  struct option longopts[] = {
      {"disable-dist-init", no_argument, 0, 'd'},
      {0, 0, 0, 0},
  };
  optind = 1;  // reset
  int opt, longindex;
  while ((opt = getopt_long(argc, argv, "d", longopts, &longindex)) != -1) {
    switch (opt) {
      case 'd':
        disable_dist_init = true;
        break;
      default:
        break;
    }
  }
}

void RevisitPP::printHelp()
{
  std::cout << RevisitPP::SOLVER_NAME << "\n"
            << "  -d --disable-dist-init"
            << "        "
            << "disable initialization of priorities "
            << "using distance from starts to goals" << std::endl;
}
