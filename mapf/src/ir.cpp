#include "../include/ir.hpp"
#include "../include/problem.hpp"


const std::string IR::SOLVER_NAME = "IR";
const IR::INIT_SOLVER_TYPE IR::DEFAULT_INIT_SOLVER =
  IR::INIT_SOLVER_TYPE::PIBT;
const IR::OPTIMAL_SOLVER_TYPE IR::DEFAULT_REFINE_SOLVER =
  IR::OPTIMAL_SOLVER_TYPE::ICBS;


IR::IR(Problem* _P) : Solver(_P)
{
  solver_name = IR::SOLVER_NAME;

  output_file = DEFAULT_OUTPUT_FILE;
  init_solver = DEFAULT_INIT_SOLVER;
  refine_solver = DEFAULT_REFINE_SOLVER;

  verbose_underlying_solver = false;
  make_log_every_itr = false;
  timeout_refinement = max_comp_time;
}

IR::~IR()
{
}

void IR::run()
{
  solution = getInitialPlan();
  solved = !solution.empty();
  if (!solved) return;  // failure

  // start refinement
  Plans hist = { solution };
  while (true) {
    if (make_log_every_itr) makeLog(output_file);
    if (overCompTime()) break;
    info("  iter: ", hist.size(),
         ", comp_time:", getSolverElapsedTime(),
         ", soc:", solution.getSOC(),
         ", makespan:", solution.getMakespan());
    solution = refinePlan(P->getConfigStart(),
                          P->getConfigGoal(),
                          solution);
    hist.push_back(solution);
    if (stopRefinement(hist)) break;
  }

  info("  refinement results, soc:", hist.begin()->getSOC(),
       "->", solution.getSOC(),
       ", makespan:", hist.begin()->getMakespan(),
       "->", solution.getMakespan());
}

Plan IR::getInitialPlan()
{
  // set problem
  Problem* _P = new Problem(P,
                            P->getConfigStart(),
                            P->getConfigGoal(),
                            max_comp_time,
                            max_timestep);
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
  default:
    solver = new PIBT(_P);
    break;
  }
  setSolverOption(solver, option_init_solver);
  solver->setVerbose(verbose_underlying_solver);

  // solve
  solver->solve();

  // success
  if (solver->succeed()) return solver->getSolution();

  // failed
  Plan dummy;
  return dummy;
}

std::tuple<bool, Plan> IR::getOptimalPlan(Problem* _P,
                                          const Plan& current_plan,
                                          const Ints& sample={})
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
  setSolverOption(solver, option_optimal_solver);
  solver->setVerbose(verbose_underlying_solver);

  // solve
  solver->solve();

  // success
  if (solver->succeed()) {
    return std::make_tuple(true, solver->getSolution());
  }

  // failed
  return std::make_tuple(false, current_plan);
}

Ints IR::sampling(const Plan& current_plan)
{
  Ints sample;
  for (int i = 0; i < P->getNum(); ++i) {
    if (getRandomBoolean(MT)) sample.push_back(i);
  }
  return sample;
}

void IR::setParams(int argc, char *argv[])
{
  struct option longopts[] = {
    { "timeout-refinement", required_argument, 0, 't' },
    { "log-every-iter", required_argument, 0, 'l' },
    { "init-solver", required_argument, 0, 'x' },
    { "optimal-solver", required_argument, 0, 'y' },
    { "option-init-solver", required_argument, 0, 'X' },
    { "option-optimal-solver", required_argument, 0, 'Y' },
    { "verbose-underlying", no_argument, 0, 'V' },
    { 0, 0, 0, 0 },
  };
  optind = 1;  // reset
  int opt, longindex;
  std::string s, s_tmp;

  while ((opt = getopt_long(argc, argv, "o:lt:x:y:X:Y:V",
                            longopts, &longindex)) != -1) {
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
    default:
      break;
    }
  }
}

void IR::printHelp()
{
  std::cout << IR::SOLVER_NAME << "\n"
            << "  -l --log-every-iter"
            << "              "
            << "make log for every iteration\n"

            << "  -t --timeout-refinement [INT]"
            << " "
            << "timeout for refinement\n"

            << "  -x --init-solver [SOLVER]"
            << "     "
            << "init solver, { PIBT, HCA, WHCA }\n"

            << "  -X --option-init-solver [\"OPTION\"]\n"
            << "                                "
            << "option for init-solver\n"

            << "  -y --refine-solver [SOLVER]"
            << "   "
            << "refine solver, { CBS, CBS_USUAL }\n"

            << "  -Y --option-refine-solver [\"OPTION\"]\n"
            << "                                "
            << "option for refine-solver\n"

            << std::endl;
}
