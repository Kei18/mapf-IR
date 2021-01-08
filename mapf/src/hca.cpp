#include "../include/hca.hpp"

const std::string HCA::SOLVER_NAME = "HCA";

HCA::HCA(Problem* _P) : Solver(_P) { solver_name = HCA::SOLVER_NAME; }

void HCA::run()
{
  Paths paths(P->getNum());

  // prioritization, far agent is prioritized
  std::vector<int> ids(P->getNum());
  std::iota(ids.begin(), ids.end(), 0);
  if (!disable_dist_init) {
    std::sort(ids.begin(), ids.end(),
              [&](int a, int b) { return pathDist(a) > pathDist(b); });
  }

  // start planning
  bool invalid = false;
  for (int j = 0; j < ids.size(); ++j) {
    int i = ids[j];
    info(" ", "elapsed:", getSolverElapsedTime(),
         ", agent-" + std::to_string(i), "starts planning,",
         "init-dist:", pathDist(i), ", progress:", j + 1, "/", P->getNum());

    // get path
    Nodes path = getPrioritizedPath(i, paths);
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

Path HCA::getPrioritizedPath(int id, const Paths& paths)
{
  Node* s = P->getStart(id);
  Node* g = P->getGoal(id);
  return getPrioritizedPath(id, s, g, paths);
}

// get single agent path
// failed -> return {}
Path HCA::getPrioritizedPath(int id, Node* s, Node* g, const Paths& paths)
{
  // pre processing
  Nodes config_s = P->getConfigStart();
  Nodes config_g = P->getConfigGoal();

  CompareAstarNode compare = [&](AstarNode* a, AstarNode* b) {
    if (a->f != b->f) return a->f > b->f;
    // tie-break, avoid goal locations of others
    if (a->v != g && inArray(a->v, config_g)) return true;
    if (b->v != g && inArray(b->v, config_g)) return false;
    // tie-break, avoid start locations
    if (a->v != s && inArray(a->v, config_s)) return true;
    if (b->v != s && inArray(b->v, config_s)) return false;
    if (a->g != b->g) return a->g < b->g;
    return false;
  };

  return getBasicPrioritizedPath(id, P, paths, getRemainedTime(), compare);
}

void HCA::setParams(int argc, char* argv[])
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

void HCA::printHelp()
{
  std::cout << HCA::SOLVER_NAME << "\n"
            << "  -d --disable-dist-init"
            << "        "
            << "disable initialization of priorities "
            << "using distance from starts to goals" << std::endl;
}
