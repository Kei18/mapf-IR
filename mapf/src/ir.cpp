#include "../include/ir.hpp"

#include <fstream>
#include <set>

const std::string IR::SOLVER_NAME = "IR";
const IR::INIT_SOLVER_TYPE IR::DEFAULT_INIT_SOLVER =
    IR::INIT_SOLVER_TYPE::PIBT_COMPLETE;
const IR::OPTIMAL_SOLVER_TYPE IR::DEFAULT_REFINE_SOLVER =
    IR::OPTIMAL_SOLVER_TYPE::ICBS;
const int IR::DEFAULT_MAX_ITERATION = 100;

// used for set underlying solver options
static void setSolverOption(Solver* solver,
                            const std::vector<std::string>& option)
{
  if (option.empty()) return;
  int argc = option.size() + 1;
  char* argv[argc + 1];
  for (int i = 0; i < argc; ++i) {
    char* tmp = const_cast<char*>(option[i].c_str());
    argv[i + 1] = tmp;
  }
  solver->setParams(argc, argv);
}

IR::IR(Problem* _P) : Solver(_P)
{
  solver_name = IR::SOLVER_NAME;

  output_file = DEFAULT_OUTPUT_FILE;
  init_solver = DEFAULT_INIT_SOLVER;
  refine_solver = DEFAULT_REFINE_SOLVER;
  max_iteration = DEFAULT_MAX_ITERATION;
  verbose_underlying_solver = false;
  make_log_every_itr = false;
  timeout_refinement = max_comp_time;
  HIST_GROUP_SIZE.push_back(0);
  HIST_GAP.push_back(0);
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
  HIST.push_back(std::make_tuple(getSolverElapsedTime(), solution));
  info("  init plan",
       ", comp_time:", getSolverElapsedTime(),
       ", soc:", init_plan_soc,
       ", makespan:", init_plan_makespan);

  int last_soc = init_plan_soc;

  // start refinement
  for (int i = 0; i < max_iteration; ++i) {
    if (make_log_every_itr) makeLog(output_file);
    if (overCompTime()) break;

    // refine plan
    solution = refinePlan(solution);
    const int soc = solution.getSOC();
    const int makespan = solution.getMakespan();
    HIST.push_back(std::make_tuple(getSolverElapsedTime(), solution));

    // print info
    info("  iter: ", i + 1,
         ", comp_time:", getSolverElapsedTime(),
         ", soc:", soc, "(improved: ", last_soc - soc, ")"
         ", makespan:", makespan);
    last_soc = soc;
  }

  // print final info
  info("  refinement results, soc:", init_plan_soc, "->",
       solution.getSOC(), ", makespan:", init_plan_makespan, "->",
       solution.getMakespan());
}

// failed -> return empty plan
Plan IR::getInitialPlan()
{
  // set problem
  Problem* _P = new Problem(P, P->getConfigStart(), P->getConfigGoal(),
                            max_comp_time, max_timestep);

  // set solver
  Solver* solver;
  switch (init_solver) {
    case INIT_SOLVER_TYPE::HCA:
      solver = new HCA(_P);
      break;
    case INIT_SOLVER_TYPE::WHCA:
      solver = new WHCA(_P);
      break;
    case INIT_SOLVER_TYPE::ECBS:
      solver = new ECBS(_P);
      break;
    case INIT_SOLVER_TYPE::PIBT:
      solver = new PIBT(_P);
      break;
    case INIT_SOLVER_TYPE::PIBT_COMPLETE:
    default:
      solver = new PIBT_COMPLETE(_P);
      break;
  }

  // set solver options
  setSolverOption(solver, option_init_solver);
  solver->setVerbose(verbose_underlying_solver);

  // solve
  solver->solve();

  // success
  Plan plan;
  if (solver->succeed()) plan = solver->getSolution();

  // memory management
  delete solver;
  delete _P;

  return plan;
}

Plan IR::refinePlan(const Plan& current_plan)
{
  Plan plan = current_plan;
  int last_soc = plan.getSOC();

  const Config& starts = P->getConfigStart();
  const Config& goals = P->getConfigGoal();

  // single refine
  do {
    for (int i = 0; i < P->getNum(); ++i) {
      const int comp_time_limit = std::min(getRemainedTime(), timeout_refinement);
      if (comp_time_limit < 0) return plan;
      plan = LibIR::refineSinglePath(i, plan, P, comp_time_limit);
      info(" ", i, plan.getSOC());
    }
    const int current_soc = plan.getSOC();
    if (current_soc >= last_soc) break;
    last_soc = current_soc;
  } while (true);

  // refinement at goals
  do {
    for (int i = 0; i < P->getNum(); ++i) {
      const int comp_time_limit = std::min(getRemainedTime(), timeout_refinement);
      if (comp_time_limit < 0) return plan;
      plan = LibIR::refineTwoPathsAtGoal(i, plan, P, comp_time_limit);
      info(" ", i, plan.getSOC());
    }
    const int current_soc = plan.getSOC();
    if (current_soc >= last_soc) break;
    last_soc = current_soc;
  } while (true);

  // old IR
  do {
    for (int i = 0; i < P->getNum(); ++i) {
      const auto modif_list = LibIR::identifyAgentsAtGoal(i, plan, P);
      if (modif_list.empty()) continue;
      const int comp_time_limit = std::min(getRemainedTime(), timeout_refinement);
      if (comp_time_limit < 0) return plan;
      Problem _P = Problem(P, starts, goals, comp_time_limit, max_timestep);
      plan = std::get<1>(getOptimalPlan(&_P, plan, modif_list));
      info(" oldIR", i, ", soc=", plan.getSOC(), ", |M|=", modif_list.size());
    }
    const int current_soc = plan.getSOC();
    if (current_soc >= last_soc) break;
    last_soc = current_soc;
  } while (true);

  // MDD
  do {
    for (int i = 0; i < P->getNum(); ++i) {
      int time_limit = std::min(getRemainedTime(), timeout_refinement);
      const auto modif_list = LibIR::identifyInteractingSetByMDD(i, plan, P, true, time_limit, MT);
      if (modif_list.empty()) continue;
      const int comp_time_limit = std::min(getRemainedTime(), timeout_refinement);
      if (comp_time_limit < 0) return plan;
      Problem _P = Problem(P, starts, goals, comp_time_limit, max_timestep);
      plan = std::get<1>(getOptimalPlan(&_P, plan, modif_list));
      info(" MDD", i, ", soc=", plan.getSOC(), ", |M|=", modif_list.size());
    }
    const int current_soc = plan.getSOC();
    if (current_soc >= last_soc) break;
    last_soc = current_soc;
  } while (true);

  // bottle neck
  do {
    for (int i = 0; i < P->getNum(); ++i) {
      const auto modif_list = std::get<1>(LibIR::identifyBottleneckAgentsWithScore(i, plan, P));
      if (modif_list.empty()) continue;
      const int comp_time_limit = std::min(getRemainedTime(), timeout_refinement);
      if (comp_time_limit < 0) return plan;
      Problem _P = Problem(P, starts, goals, comp_time_limit, max_timestep);
      plan = std::get<1>(getOptimalPlan(&_P, plan, modif_list));
      info(" Bottle neck", i, ", soc=", plan.getSOC(), ", |M|=", modif_list.size());
    }
    const int current_soc = plan.getSOC();
    if (current_soc >= last_soc) break;
    last_soc = current_soc;
  } while (true);

  return plan;
}

std::tuple<bool, Plan> IR::getOptimalPlan(Problem* _P, const Plan& current_plan,
                                          const std::vector<int>& sample)
{
  // set solver
  Solver* solver;
  switch (refine_solver) {
    case OPTIMAL_SOLVER_TYPE::CBS_NORMAL:
      solver = new CBS(_P);
      break;
    case OPTIMAL_SOLVER_TYPE::CBS:
      solver = new CBS_REFINE(_P, current_plan, sample);
      break;
    case OPTIMAL_SOLVER_TYPE::ICBS_NORMAL:
      solver = new ICBS(_P);
      break;
    case OPTIMAL_SOLVER_TYPE::ICBS:
    default:
      solver = new ICBS_REFINE(_P, current_plan, sample);
      break;
  }

  // set solver option
  setSolverOption(solver, option_optimal_solver);
  solver->setVerbose(verbose_underlying_solver);

  // solve
  solver->solve();

  Plan plan = current_plan;
  bool success = false;

  // success
  if (solver->succeed()) {
    plan = solver->getSolution();
    success = true;
  }
  delete solver;

  return std::make_tuple(success, plan);
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
      {0, 0, 0, 0},
  };
  optind = 1;  // reset
  int opt, longindex;
  std::string s, s_tmp;

  while ((opt = getopt_long(argc, argv, "o:lt:x:y:X:Y:Vn:", longopts,
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
        } else if (s == "PIBT_COMPLETE") {
          init_solver = INIT_SOLVER_TYPE::PIBT_COMPLETE;
        } else {
          warn("solver does not exists, use PIBT");
        }
        break;
      case 'X':
        s = std::string(optarg);
        for (int i = 0; i < s.size(); ++i) {
          if (s[i] == ' ') {
            option_init_solver.push_back(s_tmp);
            s_tmp = "";
          } else {
            s_tmp += s[i];
            if (i == s.size() - 1) option_init_solver.push_back(s_tmp);
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
        for (int i = 0; i < s.size(); ++i) {
          if (s[i] == ' ') {
            option_optimal_solver.push_back(s_tmp);
            s_tmp = "";
          } else {
            s_tmp += s[i];
            if (i == s.size() - 1) option_optimal_solver.push_back(s_tmp);
          }
        }
        break;
      case 'V':
        verbose_underlying_solver = true;
        break;
      case 'n':
        max_iteration = std::atoi(optarg);
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

      << std::endl;
}

void IR::makeLog(const std::string& logfile)
{
  std::ofstream log;
  log.open(logfile, std::ios::out);
  makeLogBasicInfo(log);

  // record the data of each iteration
  for (int t = 0; t < HIST.size(); ++t) {
    Plan plan = std::get<1>(HIST[t]);
    log << "iter=" << t << ","
        << "comp_time=" << std::get<0>(HIST[t]) << ","
        << "soc=" << plan.getSOC() << ","
        << "makespan=" << plan.getMakespan() << ","
        << "group=" << HIST_GROUP_SIZE[t] << ","
        << "gap=" << HIST_GAP[t] << "\n";
  }

  makeLogSolution(log);
  log.close();
}
