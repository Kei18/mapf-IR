#include "../include/pibt_icbs.hpp"
#include "../include/pibt.hpp"
#include "../include/icbs.hpp"

const std::string PIBT_ICBS::SOLVER_NAME = "PIBT_ICBS";

PIBT_ICBS::PIBT_ICBS(Problem* _P)
  : Solver(_P)
{
  solver_name = SOLVER_NAME;
}

PIBT_ICBS::~PIBT_ICBS()
{
}

void PIBT_ICBS::run()
{
  // find lower bound of makespan
  int LB_makespan = 0;
  for (int i = 0; i < P->getNum(); ++i) {
    if (pathDist(i) > LB_makespan) LB_makespan = pathDist(i);
  }

  // solve by PIBT
  Problem* _P = new Problem(P,
                            P->getConfigStart(),
                            P->getConfigGoal(),
                            max_comp_time,
                            LB_makespan);
  Solver* init_solver = new PIBT(_P);
  info(" ", "try PIBT");
  init_solver->solve();
  solution = init_solver->getSolution();

  if (init_solver->succeed()) {
    solved = true;
  } else {
    info(" ", "elapsed:", getSolverElapsedTime(),
         ", PIBT failed, try ICBS");

    // solved by ICBS
    int comp_time_limit
      = max_comp_time - (int)getSolverElapsedTime();
    Problem* _Q = new Problem(P,
                              solution.last(),
                              P->getConfigGoal(),
                              comp_time_limit,
                              max_timestep - LB_makespan);
    Solver* second_solver = new ICBS(_Q);
    second_solver->solve();
    solution += second_solver->getSolution();
    if (second_solver->succeed()) solved = true;

    delete second_solver;
    delete _Q;
  }

  delete init_solver;
  delete _P;
}

void PIBT_ICBS::printHelp()
{
  std::cout << PIBT_ICBS::SOLVER_NAME << "\n"
            << "  (no option)"
            << std::endl;
}
