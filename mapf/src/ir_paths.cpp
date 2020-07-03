#include "../include/ir_paths.hpp"
#include<set>


const std::string IR_PATHS::SOLVER_NAME = "IR_PATHS";


IR_PATHS::IR_PATHS(Problem* _P)
  : IR(_P)
{
  solver_name = IR_PATHS::SOLVER_NAME;
  find_all_interacting_agents = false;
}

IR_PATHS::~IR_PATHS()
{
}


Plan IR_PATHS::refinePlan(const Config& config_s,
                          const Config& config_g,
                          const Plan& old_plan)
{
  Paths old_paths = planToPaths(old_plan);
  auto gap =
    [&] (int i) {
      return old_paths.costOfPath(i) - pathDist(i);
    };

  // find agent with largest gap
  int id_largest_gap = -1;
  int gap_largest = 0;
  for (int i = 0; i < P->getNum(); ++i) {
    if (inArray(i, CLOSE_GAP)) continue;
    int gap_i = gap(i);
    if (gap_i > gap_largest) {
      id_largest_gap = i;
      gap_largest = gap_i;
    }
  }
  if (id_largest_gap == -1) {
    CLOSE_GAP.clear();
    return old_plan;
  }

  // find interacting agents
  std::vector<int> sample_vec;
  if (find_all_interacting_agents) {
    sample_vec = getAllInteractingAgents(old_paths, id_largest_gap);
  } else {
    sample_vec = getDirectInteractingAgents(old_paths, id_largest_gap);
  }

  info("   ", "id=", id_largest_gap,
       ", gap=", gap_largest,
       ", interacting size:", sample_vec.size());

  int comp_time_limit = std::min(max_comp_time
                                 - (int)getSolverElapsedTime(),
                                 timeout_refinement);
  if (comp_time_limit <= 0) return old_plan;
  Problem* _P = new Problem(P,
                            config_s,
                            config_g,
                            comp_time_limit,
                            max_timestep);
  Solver* solver = new ICBS_REFINE(_P, old_plan, sample_vec);
  solver->setVerbose(verbose_underlying_solver);

  solver->solve();

  if (solver->succeed()) {
    Plan plan = solver->getSolution();
    int old_soc = old_plan.getSOC();
    int new_soc = plan.getSOC();
    if (new_soc == old_soc) {
      CLOSE_GAP.push_back(id_largest_gap);
    } else if (new_soc > old_soc) {
      halt("error, something wrong");
    }
    return plan;
  }

  CLOSE_GAP.push_back(id_largest_gap);
  return old_plan;
}

std::vector<int> IR_PATHS::getDirectInteractingAgents
(const Paths& old_paths, const int id_largest_gap)
{
  int cost_largest_gap = old_paths.costOfPath(id_largest_gap);
  int dist_largest_gap = pathDist(id_largest_gap);
  std::set<int> sample = { id_largest_gap };
  Node* g = P->getGoal(id_largest_gap);
  for (int t = cost_largest_gap - 1; t >= dist_largest_gap; --t) {
    for (int i = 0; i < P->getNum(); ++i) {
      if (i == id_largest_gap) continue;
      if (old_paths.get(i, t) == g) sample.insert(i);
    }
  }
  std::vector<int> sample_vec(sample.begin(), sample.end());
  return sample_vec;
}


// find interacting agents, seems to be slow
std::vector<int> IR_PATHS::getAllInteractingAgents
(const Paths& old_paths, const int id_largest_gap)
{
  if (id_largest_gap == -1) return {};
  auto gap =
    [&] (int i) {
      return old_paths.costOfPath(i) - pathDist(i);
    };

  auto compare = [&] (int i, int j) { return gap(i) < gap(j); };
  std::priority_queue<int, std::vector<int>,
                      decltype(compare)> OPEN(compare);
  std::vector<int> CLOSE;
  OPEN.push(id_largest_gap);

  while (!OPEN.empty()) {
    int id = OPEN.top();
    OPEN.pop();
    if (inArray(id, CLOSE)) continue;
    CLOSE.push_back(id);
    Node* g = P->getGoal(id);
    int cost = old_paths.costOfPath(id);
    int dist = pathDist(id);
    for (int t = cost - 1; t >= dist; --t) {
      for (int i = 0; i < P->getNum(); ++i) {
        if (i == id) continue;
        if (old_paths.get(i, t) == g) OPEN.push(i);
      }
    }
  }
  return CLOSE;
}

void IR_PATHS::setParams(int argc, char *argv[])
{
  struct option longopts[] = {
    { "find-all-interacting-agents", no_argument, 0, 'f' },
    { 0, 0, 0, 0 },
  };

  // copy
  char *argv_copy[argc+1];
  for (int i = 0; i < argc; ++i) argv_copy[i] = argv[i];

  IR::setParams(argc, argv_copy);
  optind = 1;  // reset
  int opt, longindex;
  while ((opt = getopt_long(argc, argv, "f",
                            longopts, &longindex)) != -1) {
    switch (opt) {
    case 'f':
      find_all_interacting_agents = true;
      break;
    default:
      break;
    }
  }
}

void IR_PATHS::printHelp()
{
  std::cout << IR_PATHS::SOLVER_NAME << "\n"
            << "  -f --find-all-interacting-agents"
            << "\n"

            << "  (other: same as IR)"
            << std::endl;
}
