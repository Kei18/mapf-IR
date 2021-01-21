#include "../include/pibt_complete.hpp"

#include <fstream>
#include <memory>

#include "../include/icbs.hpp"
#include "../include/pibt.hpp"

const std::string PIBT_COMPLETE::SOLVER_NAME = "PIBT_COMPLETE";

PIBT_COMPLETE::PIBT_COMPLETE(Problem* _P) : Solver(_P)
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
  Problem _P = Problem(P, P->getConfigStart(), P->getConfigGoal(), max_comp_time, LB_makespan);
  std::unique_ptr<Solver> init_solver = std::make_unique<PIBT>(&_P);
  init_solver->setDistanceTable((DistanceTable_p == nullptr) ? &DistanceTable : DistanceTable_p);
  info(" ", "run PIBT until timestep", LB_makespan);
  init_solver->solve();
  solution = init_solver->getSolution();

  if (init_solver->succeed()) {  // PIBT success
    solved = true;

  } else {  // PIBT failed

    info(" ", "elapsed:", getSolverElapsedTime(),
         ", use ICBS to complement the remain");

    auto t_complement = Time::now();

    // solved by ICBS
    Problem _Q = Problem(P, solution.last(), P->getConfigGoal(),
                         getRemainedTime(), max_timestep - LB_makespan);
    std::unique_ptr<Solver> second_solver = std::make_unique<ICBS>(&_Q);
    second_solver->setDistanceTable((DistanceTable_p == nullptr) ? &DistanceTable : DistanceTable_p);
    second_solver->solve();
    solution += second_solver->getSolution();
    if (second_solver->succeed()) solved = true;

    comp_time_complement = getElapsedTime(t_complement);
  }
}

void PIBT_COMPLETE::printHelp()
{
  std::cout << PIBT_COMPLETE::SOLVER_NAME << "\n"
            << "  (no option)" << std::endl;
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
