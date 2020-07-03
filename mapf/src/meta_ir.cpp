#include "../include/meta_ir.hpp"

const std::string META_IR::SOLVER_NAME = "META_IR";
const int META_IR::DEFAULT_MIN_THRESHOLD_MAKESPAN = 3;
const int META_IR::DEFAULT_MAX_THRESHOLD_MAKESPAN = 10;


META_IR::META_IR(Problem* _P) : IR(_P)
{
  solver_name = META_IR::SOLVER_NAME;

  min_threshold_makespan = DEFAULT_MIN_THRESHOLD_MAKESPAN;
  max_threshold_makespan = DEFAULT_MAX_THRESHOLD_MAKESPAN;
  threshold_makespan = min_threshold_makespan;
}

void META_IR::iterativeRefinement()
{
  int iteration = 0;
  solution = getInitialPlan();
  solved = !solution.empty();
  if (!solved) return;  // failure

  // start refinement
  Plans hist = { solution };
  while (true) {
    makeLog(output_file);  // anytime planning
    if (overCompTime()) break;

    info("  iter: ", iteration++,
         ", threshold:", threshold_makespan,
         ", comp_time:", getSolverElapsedTime(),
         ", soc:", solution.getSOC(),
         ", makespan:", solution.getMakespan());
    Plan new_plan = refinePlan(P->getConfigStart(),
                               P->getConfigGoal(),
                               solution);
    hist.push_back(new_plan);
    if (stopRefinement(new_plan, hist)) {
      if (threshold_makespan >= max_threshold_makespan) {
        break;
      } else {
        hist.clear();
        hist.push_back(solution);
        ++threshold_makespan;
      }
    }
    solution = new_plan;
  }

  info("  refinement results, soc:", hist.begin()->getSOC(),
       "->", solution.getSOC(),
       ", makespan:", hist.begin()->getMakespan(),
       "->", solution.getMakespan());
}

void META_IR::setParams(int argc, char *argv[])
{
  struct option longopts[] = {
    { "min-threshold-makespan", required_argument, 0, 'm' },
    { "max-threshold-makespan", required_argument, 0, 'M' },
    { 0, 0, 0, 0 },
  };

  // copy
  char *argv_copy[argc+1];
  for (int i = 0; i < argc; ++i) argv_copy[i] = argv[i];

  IR::setParams(argc, argv_copy);
  optind = 1;  // reset
  int opt, longindex;
  while ((opt = getopt_long(argc, argv, "m:M:",
                            longopts, &longindex)) != -1) {
    switch (opt) {
    case 'm':
      min_threshold_makespan = std::stoi(optarg);
      if (min_threshold_makespan < 1) {
        warn("min threshold makespan value must be > 1, using default value");
        min_threshold_makespan = DEFAULT_MIN_THRESHOLD_MAKESPAN;
      }
      threshold_makespan = min_threshold_makespan;
      break;
    case 'M':
      max_threshold_makespan = std::stoi(optarg);
      if (max_threshold_makespan < 1) {
        warn("max threshold makespan value must be > 1, using default value");
        max_threshold_makespan = DEFAULT_MAX_THRESHOLD_MAKESPAN;
      }
      break;
    default:
      break;
    }
  }
}

void META_IR::printHelp()
{
  std::cout << META_IR::SOLVER_NAME << "\n"
            << "  -m --min-threshold-makespan [INT]\n"
            << "                                "
            << "min threshold makespan\n"

            << "  -M --max-threshold-makespan [INT]\n"
            << "                                "
            << "max threshold makespan\n"

            << "  (other: same as IR)"
            << std::endl;
}
