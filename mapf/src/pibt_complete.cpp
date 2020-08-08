#include "../include/pibt_complete.hpp"
#include "../include/pibt.hpp"
#include "../include/icbs.hpp"
#include <fstream>


const std::string PIBT_COMPLETE::SOLVER_NAME = "PIBT_COMPLETE";

PIBT_COMPLETE::PIBT_COMPLETE(Problem* _P)
  : Solver(_P)
{
  solver_name = SOLVER_NAME;
  comp_time_complement = 0;
}

PIBT_COMPLETE::~PIBT_COMPLETE()
{
}

void PIBT_COMPLETE::run()
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

    auto t_complement = Time::now();

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

    comp_time_complement = getElapsedTime(t_complement);

    delete second_solver;
    delete _Q;
  }

  delete init_solver;
  delete _P;
}

void PIBT_COMPLETE::printHelp()
{
  std::cout << PIBT_COMPLETE::SOLVER_NAME << "\n"
            << "  (no option)"
            << std::endl;
}

void PIBT_COMPLETE::makeLog(const std::string& logfile)
{
  Grid* grid = reinterpret_cast<Grid*>(P->getG());

  std::ofstream log;
  log.open(logfile, std::ios::out);
  log << "instance= " << P->getInstanceFileName() << "\n";
  log << "agents=" << P->getNum() << "\n";
  log << "map_file=" << grid->getMapFileName() << "\n";
  log << "solver=" << solver_name << "\n";
  log << "solved=" << solved << "\n";
  log << "soc=" << solution.getSOC() << "\n";
  log << "makespan=" << solution.getMakespan() << "\n";
  log << "comp_time=" << comp_time << "\n";

  // print additional info
  log << "comp_time_complement=" << comp_time_complement << "\n";

  log << "starts=";
  for (int i = 0; i < P->getNum(); ++i) {
    Node* v = P->getStart(i);
    log << "(" << v->pos.x << "," << v->pos.y << "),";
  }
  log << "\ngoals=";
  for (int i = 0; i < P->getNum(); ++i) {
    Node* v = P->getGoal(i);
    log << "(" << v->pos.x << "," << v->pos.y << "),";
  }
  log << "\n";
  log << "solution=\n";
  for (int t = 0; t <= solution.getMakespan(); ++t) {
    log << t << ":";
    auto c = solution.get(t);
    for (auto v : c) {
      log << "(" << v->pos.x << "," << v->pos.y << "),";
    }
    log << "\n";
  }
  log.close();
}
