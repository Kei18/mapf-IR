#include "../include/revisit_pp.hpp"

const std::string RevisitPP::SOLVER_NAME = "RevisitPP";

RevisitPP::RevisitPP(Problem* _P) : Solver(_P)
{
  solver_name = RevisitPP::SOLVER_NAME;
}

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

// get single agent path
// failed -> return {}
Path RevisitPP::getPrioritizedPath(
    int id, const Paths& paths,
    const std::vector<std::tuple<Node*, int>> constraints)
{
  const auto p = Solver::getPrioritizedPath(
      id, paths, getRemainedTime(),
      max_timestep, constraints, compareAstarNodeBasic, false);

  // update path table
  updatePathTableWithoutClear(id, p, paths);

  return p;
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
