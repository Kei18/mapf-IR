#include "../include/pibt_complete.hpp"

#include <fstream>
#include <memory>

#include "../include/ecbs.hpp"
#include "../include/icbs.hpp"
#include "../include/pibt.hpp"
#include "../include/push_and_swap.hpp"

const std::string PIBT_COMPLETE::SOLVER_NAME = "PIBT_COMPLETE";

PIBT_COMPLETE::PIBT_COMPLETE(Problem* _P)
    : Solver(_P), comp_solver_type(COMP_SOLVER_TYPE::PUSH_AND_SWAP)
{
  solver_name = SOLVER_NAME;
  comp_time_complement = 0;
}

void PIBT_COMPLETE::run()
{
  // find lower bound of makespan
  int LB_makespan = 0;
  for (int i = 0; i < P->getNum(); ++i) {
    if (pathDist(i) > LB_makespan) LB_makespan = pathDist(i);
  }

  // solve by PIBT
  Problem _P = Problem(P, P->getConfigStart(), P->getConfigGoal(),
                       max_comp_time, LB_makespan);
  std::unique_ptr<Solver> init_solver = std::make_unique<PIBT>(&_P);
  init_solver->setDistanceTable((DistanceTable_p == nullptr) ? &DistanceTable
                                                             : DistanceTable_p);
  info(" ", "run PIBT until timestep", LB_makespan);
  init_solver->solve();
  solution = init_solver->getSolution();

  if (init_solver->succeed()) {  // PIBT success
    solved = true;

  } else {  // PIBT failed

    auto t_complement = Time::now();

    // solved by ICBS
    Problem _Q = Problem(P, solution.last(), P->getConfigGoal(),
                         getRemainedTime(), max_timestep - LB_makespan);
    std::shared_ptr<Solver> comp_solver;
    switch (comp_solver_type) {
      case COMP_SOLVER_TYPE::ICBS:
        comp_solver = std::make_shared<ICBS>(&_Q);
        break;
      case COMP_SOLVER_TYPE::ECBS:
        comp_solver = std::make_shared<ECBS>(&_Q);
        break;
      default:
        comp_solver = std::make_shared<PushAndSwap>(&_Q);
        break;
    }

    // set solver options
    setSolverOption(comp_solver, option_comp_solver);
    comp_solver->setDistanceTable(
        (DistanceTable_p == nullptr) ? &DistanceTable : DistanceTable_p);

    info(" ", "elapsed:", getSolverElapsedTime(), ", use",
         comp_solver->getSolverName(), "to complement the remain");

    // solve
    comp_solver->solve();
    solution += comp_solver->getSolution();
    if (comp_solver->succeed()) solved = true;

    comp_time_complement = getElapsedTime(t_complement);
  }
}

void PIBT_COMPLETE::setParams(int argc, char* argv[])
{
  struct option longopts[] = {
      {"comp-solver", required_argument, 0, 'x'},
      {"option-comp-solver", required_argument, 0, 'X'},
      {0, 0, 0, 0},
  };
  optind = 1;  // reset
  int opt, longindex, s_size;
  std::string s, s_tmp;

  while ((opt = getopt_long(argc, argv, "x:X:", longopts, &longindex)) != -1) {
    switch (opt) {
      case 'x':
        s = std::string(optarg);
        if (s == "PushAndSwap") {
          comp_solver_type = COMP_SOLVER_TYPE::PUSH_AND_SWAP;
        } else if (s == "ECBS") {
          comp_solver_type = COMP_SOLVER_TYPE::ECBS;
        } else if (s == "ICBS") {
          comp_solver_type = COMP_SOLVER_TYPE::ICBS;
        } else {
          warn("solver does not exists, use PushAndSwap");
        }
        break;
      case 'X':
        s = std::string(optarg);
        s_size = s.size();
        for (int i = 0; i < s_size; ++i) {
          if (s[i] == ' ') {
            option_comp_solver.push_back(s_tmp);
            s_tmp = "";
          } else {
            s_tmp += s[i];
            if (i == s_size - 1) option_comp_solver.push_back(s_tmp);
          }
        }
        break;
    }
  }
}

void PIBT_COMPLETE::printHelp()
{
  std::cout
      << PIBT_COMPLETE::SOLVER_NAME << "\n"
      << "  -x --comp-solver [SOLVER]"
      << "     "
      << "init solver: { PushAndSwap, ECBS, ICBS }, default: PushAndSwap\n"

      << "  -X --option-comp-solver [\"OPTION\"]\n"
      << "                                "
      << "option for comp-solver\n"

      << std::endl;
}

void PIBT_COMPLETE::makeLog(const std::string& logfile)
{
  std::ofstream log;
  log.open(logfile, std::ios::out);
  makeLogBasicInfo(log);

  // print additional info
  log << "comp_time_complement=" << comp_time_complement << "\n";

  makeLogSolution(log);
  log.close();
}
