#include "../include/ir_paths.hpp"
#include<set>


const std::string IR_PATHS::SOLVER_NAME = "IR_PATHS";
const int IR_PATHS::DEFAULT_MAX_ITERATION = 100;

IR_PATHS::IR_PATHS(Problem* _P)
  : IR(_P)
{
  solver_name = IR_PATHS::SOLVER_NAME;

  max_iteration = DEFAULT_MAX_ITERATION;
  find_all_interacting_agents = false;
}

IR_PATHS::~IR_PATHS()
{
}

bool IR_PATHS::stopRefinement()
{
  return HIST.size() >= max_iteration;
}

Plan IR_PATHS::refinePlan(const Config& config_s,
                          const Config& config_g,
                          const Plan& current_plan)
{
  // ============================================
  // sampling
  Paths current_paths = planToPaths(current_plan);
  auto gap =
    [&] (int i) {
      return current_paths.costOfPath(i) - pathDist(i);
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
    return current_plan;
  }

  // find interacting agents
  std::vector<int> sample;
  if (find_all_interacting_agents) {
    sample = getAllInteractingAgents(current_paths, id_largest_gap);
  } else {
    sample = getDirectInteractingAgents(current_paths, id_largest_gap);
  }
  info("   ", "id=", id_largest_gap,
       ", gap=", gap_largest,
       ", interacting size:", sample.size());

  // ============================================
  // create problem
  int comp_time_limit
    = std::min(max_comp_time - (int)getSolverElapsedTime(),
               timeout_refinement);
  if (comp_time_limit <= 0) return current_plan;
  Problem* _P = new Problem(P,
                            config_s,
                            config_g,
                            comp_time_limit,
                            max_timestep);

  // solve
  auto res = getOptimalPlan(_P, current_plan, sample);
  Plan plan = std::get<1>(res);
  if (!std::get<0>(res) || plan.getSOC() == current_plan.getSOC()) {
    CLOSE_GAP.push_back(id_largest_gap);
  }
  return plan;
}

Ints IR_PATHS::getDirectInteractingAgents
(const Paths& current_paths, const int id_largest_gap)
{
  int cost_largest_gap = current_paths.costOfPath(id_largest_gap);
  int dist_largest_gap = pathDist(id_largest_gap);
  std::set<int> sample = { id_largest_gap };
  Node* g = P->getGoal(id_largest_gap);
  for (int t = cost_largest_gap - 1; t >= dist_largest_gap; --t) {
    for (int i = 0; i < P->getNum(); ++i) {
      if (i == id_largest_gap) continue;
      if (current_paths.get(i, t) == g) sample.insert(i);
    }
  }
  std::vector<int> sample_vec(sample.begin(), sample.end());
  return sample_vec;
}

Ints IR_PATHS::getAllInteractingAgents
(const Paths& current_paths, const int id_largest_gap)
{
  auto gap =
    [&] (int i) {
      return current_paths.costOfPath(i) - pathDist(i);
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
    int cost = current_paths.costOfPath(id);
    int dist = pathDist(id);
    for (int t = cost - 1; t >= dist; --t) {
      for (int i = 0; i < P->getNum(); ++i) {
        if (i == id) continue;
        if (current_paths.get(i, t) == g) OPEN.push(i);
      }
    }
  }
  return CLOSE;
}

void IR_PATHS::setParams(int argc, char *argv[])
{
  struct option longopts[] = {
    { "find-all-interacting-agents", no_argument, 0, 'f' },
    { "max-iteration", required_argument, 0, 'n' },
    { 0, 0, 0, 0 },
  };

  char *argv_copy[argc+1];
  for (int i = 0; i < argc; ++i) argv_copy[i] = argv[i];
  IR::setParams(argc, argv_copy);

  optind = 1;  // reset
  int opt, longindex;
  while ((opt = getopt_long(argc, argv, "fn:",
                            longopts, &longindex)) != -1) {
    switch (opt) {
    case 'f':
      find_all_interacting_agents = true;
      break;
    case 'n':
      max_iteration = std::atoi(optarg);
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

            << "  -n --max-iteration [INT]"
            << "      "
            << "max iteration\n"

            << "  (other: same as IR)\n";
}
