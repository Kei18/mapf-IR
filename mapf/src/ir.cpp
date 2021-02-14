#include "../include/ir.hpp"

#include <fstream>
#include <set>

// solver name
const std::string IR::SOLVER_NAME = "IR";
const std::string IR_SINGLE_PATHS::SOLVER_NAME = "IR_SINGLE_PATHS";
const std::string IR_FIX_AT_GOALS::SOLVER_NAME = "IR_FIX_AT_GOALS";
const std::string IR_FOCUS_GOALS::SOLVER_NAME = "IR_FOCUS_GOALS";
const std::string IR_MDD::SOLVER_NAME = "IR_MDD";
const std::string IR_BOTTLENECK::SOLVER_NAME = "IR_BOTTLENECK";
const std::string IR_HYBRID::SOLVER_NAME = "IR_HYBRID";

IR::IR(Problem* _P)
    : Solver(_P),
      init_solver(DEFAULT_INIT_SOLVER),
      refine_solver(DEFAULT_REFINE_SOLVER),
      current_iteration(0),
      max_iteration(DEFAULT_MAX_ITERATION),
      output_file(DEFAULT_OUTPUT_FILE),
      make_log_every_itr(false),
      timeout_refinement(DEFAULT_TIMEOUT_REFINEMENT),
      verbose_underlying_solver(false),
      sampling_num(std::min(DEFAULT_SAMPLING_NUM, P->getNum()))
{
  solver_name = IR::SOLVER_NAME;
}

IR::~IR() {}

void IR::run()
{
  // get initial plan
  solution = getInitialPlan();
  solved = !solution.empty();
  if (!solved) return;  // failure
  const int init_plan_soc = solution.getSOC();
  const int init_plan_makespan = solution.getMakespan();
  HIST.push_back(std::make_tuple(getSolverElapsedTime(), init_plan_soc,
                                 init_plan_makespan));
  info("  init plan", ", comp_time:", getSolverElapsedTime(),
       ", soc:", init_plan_soc, ", makespan:", init_plan_makespan);
  last_soc = init_plan_soc;
  last_makespan = init_plan_makespan;

  if (make_log_every_itr) makeLog(output_file);

  // refinement
  refinePlan();

  // print final info
  const int soc = solution.getSOC();
  const int makespan = solution.getMakespan();
  info("  refinement results, soc:", init_plan_soc, "->", soc,
       " (improved:", init_plan_soc - soc, ")",
       ", makespan:", init_plan_makespan, "->", solution.getMakespan(),
       " (improved:", init_plan_makespan - makespan, ")");
}

void IR::updateSolution(const Plan& plan)
{
  ++current_iteration;

  solution = plan;
  const int soc = solution.getSOC();
  const int makespan = solution.getMakespan();
  HIST.push_back(std::make_tuple(getSolverElapsedTime(), soc, makespan));

  if (make_log_every_itr) makeLog(output_file);

  printProcessInfo();

  last_soc = soc;
  last_makespan = makespan;
}

void IR::printProcessInfo()
{
  const int soc = solution.getSOC();
  const int makespan = solution.getMakespan();

  info("  iter: ", current_iteration, ", comp_time:", getSolverElapsedTime(),
       ", soc:", soc, "(improved: ", last_soc - soc,
       ", LB: ", getLowerBoundSOC(), ")", ", makespan:", makespan,
       "(improved: ", last_makespan - makespan,
       ", LB: ", getLowerBoundMakespan(), ")");
}

void IR::setInitialPlan(const Plan& plan) { solution = plan; }

// failed -> return empty plan
Plan IR::getInitialPlan()
{
  if (!solution.empty()) return solution;

  // set problem
  Problem _P = Problem(P, max_comp_time);

  // set solver
  std::shared_ptr<Solver> solver;
  switch (init_solver) {
    case INIT_SOLVER_TYPE::HCA:
      solver = std::make_shared<HCA>(&_P);
      break;
    case INIT_SOLVER_TYPE::WHCA:
      solver = std::make_shared<WHCA>(&_P);
      break;
    case INIT_SOLVER_TYPE::ECBS:
      solver = std::make_shared<ECBS>(&_P);
      break;
    case INIT_SOLVER_TYPE::PIBT:
      solver = std::make_shared<PIBT>(&_P);
      break;
    case INIT_SOLVER_TYPE::RevisitPP:
      solver = std::make_shared<RevisitPP>(&_P);
      break;
    case INIT_SOLVER_TYPE::PushAndSwap:
      solver = std::make_shared<PushAndSwap>(&_P);
      break;
    case INIT_SOLVER_TYPE::PIBT_COMPLETE:
    default:
      solver = std::make_shared<PIBT_COMPLETE>(&_P);
      break;
  }

  // set solver options
  setSolverOption(solver, option_init_solver);
  solver->setVerbose(verbose_underlying_solver);
  solver->setDistanceTable(&distance_table);

  // solve
  solver->solve();

  // success
  Plan plan;
  if (solver->succeed()) plan = solver->getSolution();

  return plan;
}

void IR::refinePlan() { updateByRandom(); }

std::tuple<bool, Plan> IR::getOptimalPlan(Problem* _P, const Plan& current_plan,
                                          const std::vector<int>& sample)
{
  // set solver
  std::shared_ptr<Solver> solver;
  switch (refine_solver) {
    case OPTIMAL_SOLVER_TYPE::CBS_NORMAL:
      solver = std::make_shared<CBS>(_P);
      break;
    case OPTIMAL_SOLVER_TYPE::CBS:
      solver = std::make_shared<CBS_REFINE>(_P, current_plan, sample);
      break;
    case OPTIMAL_SOLVER_TYPE::ICBS_NORMAL:
      solver = std::make_shared<ICBS>(_P);
      break;
    case OPTIMAL_SOLVER_TYPE::ICBS:
    default:
      solver = std::make_shared<ICBS_REFINE>(_P, current_plan, sample);
      break;
  }

  // set solver option
  setSolverOption(solver, option_optimal_solver);
  solver->setVerbose(verbose_underlying_solver);
  solver->setDistanceTable(&distance_table);

  // solve
  solver->solve();

  Plan plan = current_plan;
  bool success = false;

  // success
  if (solver->succeed()) {
    plan = solver->getSolution();
    success = true;
  }

  return std::make_tuple(success, plan);
}

// ====================================================
void IR::updateByRandom()
{
  Plan plan = solution;
  std::vector<int> A(P->getNum());
  std::iota(A.begin(), A.end(), 0);
  while (!overCompTime() && current_iteration < max_iteration) {
    std::shuffle(A.begin(), A.end(), *MT);
    std::vector<int> modif_list(sampling_num);
    std::copy(A.begin(), A.begin() + sampling_num, modif_list.begin());
    Problem _P = Problem(P, getRefineTimeLimit());
    plan = std::get<1>(getOptimalPlan(&_P, plan, modif_list));
    updateSolution(plan);
  }
}

void IR::updatePlanFocusOneAgent(std::function<void(const int, Plan&, IR*)> fn)
{
  Plan plan = solution;
  int last_itr_soc = plan.getSOC();

  // fix at goal
  do {
    last_itr_soc = plan.getSOC();
    for (int i = 0; i < P->getNum(); ++i) {
      if (overCompTime() || current_iteration >= max_iteration) break;
      if (plan.getPathCost(i) - pathDist(i) == 0) continue;
      fn(i, plan, this);
    }
  } while (last_itr_soc != plan.getSOC() && !overCompTime() &&
           current_iteration < max_iteration);
}

// ----------------------------
// define refinement rules
// ----------------------------
void IR::updateBySinglePaths(const int i, Plan& plan, IR* const solver)
{
  // filtering
  const int cost = plan.getPathCost(i);
  if (cost == solver->pathDist(i)) return;

  // get new path
  auto paths = planToPaths(plan);
  const auto path = solver->getPrioritizedPath
    (i, paths, solver->getRefineTimeLimit(), solver->getMaxTimestep());
  if (path.empty() || getPathCost(path) >= cost) return;

  // update paths
  paths.insert(i, path);
  plan = pathsToPlan(paths);
  solver->updateSolution(plan);
}

void IR::updateByFixAtGoals(const int i, Plan& plan, IR* const solver)
{
  auto t_s = Time::now();
  const auto P = solver->getP();
  Node* g = P->getGoal(i);
  const int cost = plan.getPathCost(i);
  const int dist = solver->pathDist(i);
  if (cost <= dist + 1) return;

  auto paths = planToPaths(plan);
  Path path = paths.get(i);
  bool stop_flg = false;

  for (int t = cost - 1; t > dist; --t) {
    if (path[t] == g) continue;
    if (path[t - 1] != g || path[t + 1] != g) break;

    for (int j = 0; j < P->getNum(); ++j) {
      if (i == j) continue;
      if (paths.get(j, t) != g) continue;

      // create temporal paths
      auto tmp_path = path;
      auto tmp_paths = paths;
      tmp_path.resize(t);
      tmp_paths.insert(i, tmp_path);

      const int original_costs = paths.costOfPath(i) + paths.costOfPath(j);
      const int upper_bound = original_costs - tmp_paths.costOfPath(i) - 1;

      // constraints
      std::tuple<Node*, int> constraint = std::make_tuple(g, t);

      // get refined plan for j
      const auto refined_path_j = solver->getPrioritizedPath
        (j, tmp_paths, solver->getRefineTimeLimit() - getElapsedTime(t_s),
         upper_bound, {constraint});
      if (refined_path_j.empty()) {
        stop_flg = true;
        break;
      }
      tmp_paths.insert(j, refined_path_j);

      // check update or not
      paths = tmp_paths;
      path = paths.get(i);
      break;
    }
    if (stop_flg) break;
  }

  plan = pathsToPlan(paths);
  solver->updateSolution(plan);
}

void IR::updateByFocusGoals(const int i, Plan& plan, IR* const solver)
{
  const auto P = solver->getP();
  const auto modif_list = identifyAgentsAtGoal(i, plan, P->getGoal(i), solver->pathDist(i));
  if (modif_list.empty()) return;
  Problem _P = Problem(P, solver->getRefineTimeLimit());
  plan = std::get<1>(solver->getOptimalPlan(&_P, plan, modif_list));
  solver->updateSolution(plan);
}

void IR::updateByBottleneck(const int i, Plan& plan, IR* const solver)
{
  const auto modif_list = std::get<1>(IR::identifyBottleneckAgentsWithScore
                                      (i, planToPaths(plan), solver, solver->getRefineTimeLimit()));
  if (modif_list.empty()) return;
  Problem _P = Problem(solver->getP(), solver->getRefineTimeLimit());
  plan = std::get<1>(solver->getOptimalPlan(&_P, plan, modif_list));
  solver->updateSolution(plan);
}

void IR::updateByMDD(const int i, Plan& plan, IR* const solver)
{
  const auto P = solver->getP();
  const auto modif_list = IR::identifyInteractingSetByMDD(
      i, plan, solver, true, solver->getRefineTimeLimit(), P->getMT());
  if (modif_list.empty()) return;
  Problem _P = Problem(P, solver->getRefineTimeLimit());
  plan = std::get<1>(solver->getOptimalPlan(&_P, plan, modif_list));
  solver->updateSolution(plan);
}

// ----------------------------
// utilities for refinement rules
// ----------------------------
std::vector<int> IR::identifyInteractingSetByMDD
(const int i, const Plan& plan, Solver* const solver,
 bool whole_duration, const int time_limit, std::mt19937* MT)
{
  auto t_start = Time::now();

  // basic info
  const int cost = plan.getPathCost(i);
  const int dist = solver->pathDist(i);

  // filtering
  if (cost == dist) return {};

  std::vector<int> agents(plan.get(0).size());
  std::iota(agents.begin(), agents.end(), 0);
  if (MT != nullptr) std::shuffle(agents.begin(), agents.end(), *MT);

  // modification set
  std::set<int> modif_set;

  for (int t = dist; t < cost; ++t) {
    // make mdd with small cost
    auto mdd = LibCBS::MDD(t, i, solver);
    if (time_limit == -1) {
      mdd.build();
    } else {
      int _t_limit = time_limit - getElapsedTime(t_start);
      if (_t_limit < 0) return {};
      mdd.build(_t_limit);
    }

    // create modif list
    for (auto j : agents) {
      if (i == j) continue;
      if (mdd.forceUpdate(LibCBS::getConstraintsByFixedPaths(plan, {j})) ||
          !mdd.valid) {
        modif_set.insert(j);
      }
      if (!mdd.valid) break;
    }

    if (!whole_duration) break;
  }

  if (modif_set.empty()) return {};

  modif_set.insert(i);
  std::vector<int> moidf_list(modif_set.begin(), modif_set.end());
  return moidf_list;
}

std::vector<int> IR::identifyAgentsAtGoal(const int i, const Plan& plan, const Node* g, const int dist)
{
  const int cost = plan.getPathCost(i);
  const int num = plan.get(0).size();
  if (cost == dist) return {};

  std::set<int> modif_set = {i};
  for (int t = cost - 1; t >= dist; --t) {
    for (int j = 0; j < num; ++j) {
      if (j == i) continue;
      if (plan.get(t, j) == g) modif_set.insert(j);
    }
  }
  std::vector<int> moidf_list(modif_set.begin(), modif_set.end());
  return moidf_list;
}

std::tuple<int, std::vector<int>> IR::identifyBottleneckAgentsWithScore
(const int i, const Paths& original_paths, Solver* const solver, const int time_limit)
{
  int score = 0;
  std::vector<int> modif_list;
  auto paths = original_paths;
  paths.clear(i);

  const int num = paths.size();

  for (int j = 0; j < num; ++j) {
    if (i == j) continue;
    const int dist = solver->pathDist(j);
    const int original_cost = paths.costOfPath(j);
    if (original_cost == dist) continue;
    const auto path = solver->getPrioritizedPath(j, paths, time_limit);
    if (path.empty()) {
      modif_list.clear();
      return std::make_tuple(0, modif_list);
    }
    const int cost = getPathCost(path);
    if (cost < original_cost) {
      score += original_cost - cost;
      modif_list.push_back(j);
    }
  }

  if (!modif_list.empty()) modif_list.push_back(i);
  return std::make_tuple(score, modif_list);
}



void IR::setParams(int argc, char* argv[])
{
  struct option longopts[] = {
      {"timeout-refinement", required_argument, 0, 't'},
      {"log-every-iter", required_argument, 0, 'l'},
      {"init-solver", required_argument, 0, 'x'},
      {"optimal-solver", required_argument, 0, 'y'},
      {"option-init-solver", required_argument, 0, 'X'},
      {"option-optimal-solver", required_argument, 0, 'Y'},
      {"verbose-underlying", no_argument, 0, 'V'},
      {"max-iteration", required_argument, 0, 'n'},
      {"sampling-num", required_argument, 0, 'S'},
      {0, 0, 0, 0},
  };
  optind = 1;  // reset
  int opt, longindex, s_size;
  std::string s, s_tmp;

  while ((opt = getopt_long(argc, argv, "o:lt:x:y:X:Y:Vn:S:", longopts,
                            &longindex)) != -1) {
    switch (opt) {
      case 'o':
        output_file = std::string(optarg);
        break;
      case 'l':
        make_log_every_itr = true;
        break;
      case 't':
        timeout_refinement = std::atoi(optarg);
        if (timeout_refinement < 0 || max_comp_time < timeout_refinement) {
          warn("invalid early-stop time, using max_comp_time");
          timeout_refinement = max_comp_time;
        }
        break;
      case 'x':
        s = std::string(optarg);
        if (s == "PIBT") {
          init_solver = INIT_SOLVER_TYPE::PIBT;
        } else if (s == "HCA") {
          init_solver = INIT_SOLVER_TYPE::HCA;
        } else if (s == "WHCA") {
          init_solver = INIT_SOLVER_TYPE::WHCA;
        } else if (s == "ECBS") {
          init_solver = INIT_SOLVER_TYPE::ECBS;
        } else if (s == "RevisitPP") {
          init_solver = INIT_SOLVER_TYPE::RevisitPP;
        } else if (s == "PIBT_COMPLETE") {
          init_solver = INIT_SOLVER_TYPE::PIBT_COMPLETE;
        } else if (s == "PushAndSwap") {
          init_solver = INIT_SOLVER_TYPE::PushAndSwap;
        } else {
          warn("solver does not exists, use PIBT");
        }
        break;
      case 'X':
        s = std::string(optarg);
        s_size = s.size();
        for (int i = 0; i < s_size; ++i) {
          if (s[i] == ' ') {
            option_init_solver.push_back(s_tmp);
            s_tmp = "";
          } else {
            s_tmp += s[i];
            if (i == s_size - 1) option_init_solver.push_back(s_tmp);
          }
        }
        break;
      case 'y':
        s = std::string(optarg);
        if (s == "CBS") {
          refine_solver = OPTIMAL_SOLVER_TYPE::CBS;
        } else if (s == "CBS_NORMAL") {
          refine_solver = OPTIMAL_SOLVER_TYPE::CBS_NORMAL;
        } else if (s == "ICBS") {
          refine_solver = OPTIMAL_SOLVER_TYPE::ICBS;
        } else if (s == "ICBS_NORMAL") {
          refine_solver = OPTIMAL_SOLVER_TYPE::ICBS_NORMAL;
        } else {
          warn("solver does not exists, use ICBS");
        }
        break;
      case 'Y':
        s = std::string(optarg);
        s_size = s.size();
        for (int i = 0; i < s_size; ++i) {
          if (s[i] == ' ') {
            option_optimal_solver.push_back(s_tmp);
            s_tmp = "";
          } else {
            s_tmp += s[i];
            if (i == s_size) option_optimal_solver.push_back(s_tmp);
          }
        }
        break;
      case 'V':
        verbose_underlying_solver = true;
        break;
      case 'n':
        max_iteration = std::atoi(optarg);
        break;
      case 'S':
        sampling_num = std::min(std::atoi(optarg), P->getNum());
        break;
      default:
        break;
    }
  }
}

void IR::printHelp()
{
  std::cout
      << IR::SOLVER_NAME << "\n"
      << "  -l --log-every-iter"
      << "           "
      << "make log for every iteration\n"

      << "  -t --timeout-refinement [INT]"
      << " "
      << "timeout for refinement\n"

      << "  -x --init-solver [SOLVER]"
      << "     "
      << "init solver: { PIBT, HCA, WHCA, PIBT_COMPLETE }, default: "
         "PIBT_COMPLETE\n"

      << "  -X --option-init-solver [\"OPTION\"]\n"
      << "                                "
      << "option for init-solver\n"

      << "  -y --refine-solver [SOLVER]"
      << "   "
      << "refine solver: { CBS, CBS_USUAL, ICBS, ICBS_USUAL }, default: ICBS\n"

      << "  -Y --option-refine-solver [\"OPTION\"]\n"
      << "                                "
      << "option for refine-solver\n"

      << "  -n --max-iteration [INT]"
      << "      "
      << "max iteration\n"

      << "  -S --sampling-num [INT]"
      << "       "
      << "number of sampling"

      << std::endl;
}

void IR::makeLog(const std::string& logfile)
{
  std::ofstream log;
  log.open(logfile, std::ios::out);
  makeLogBasicInfo(log);

  // record the data of each iteration
  const int HIST_size = HIST.size();
  for (int t = 0; t < HIST_size; ++t) {
    auto hist = HIST[t];
    log << "iter=" << t << ","
        << "comp_time=" << std::get<0>(hist) << ","
        << "soc=" << std::get<1>(hist) << ","
        << "makespan=" << std::get<1>(hist) << "\n";
  }

  makeLogSolution(log);
  log.close();
}

// ---------------------------------
// IR_HYBRID
// ---------------------------------
void IR_HYBRID::refinePlan()
{
  info("", "update by FIX_AT_GOALS");
  updatePlanFocusOneAgent(updateByFixAtGoals);
  info("", "update by FOCUS_GOALS");
  updatePlanFocusOneAgent(updateByFocusGoals);
  info("", "update by MDD");
  updatePlanFocusOneAgent(updateByMDD);
  info("", "update by RANDOM");
  updateByRandom();
}
