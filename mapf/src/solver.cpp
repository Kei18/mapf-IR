#include "../include/solver.hpp"

#include <fstream>

#include "../include/lib_cbs.hpp"

Solver::Solver(Problem* _P)
    : P(_P),
      G(_P->getG()),
      MT(_P->getMT()),
      max_timestep(P->getMaxTimestep()),
      max_comp_time(P->getMaxCompTime()),
      LB_soc(0),
      LB_makespan(0),
      DistanceTable(P->getNum(), std::vector<int>(G->getNodesSize(), max_timestep)),
      DistanceTable_p(nullptr)
{
  solved = false;
  verbose = false;

  // for solvers using MDD
  LibCBS::MDD::MT = MT;
}

Solver::~Solver()
{
  if (P->isInitializedInstance()) LibCBS::MDD::PURE_MDD_TABLE.clear();
}

void Solver::solve()
{
  start();

  // preprocessing
  if (DistanceTable_p == nullptr) {
    info("  pre-processing, create distance table by BFS");
    createDistanceTable();
    info("  done, elapsed: ", getSolverElapsedTime());
  }

  run();
  end();
}

Path Solver::getTimedPath
(Node* const s,
 Node* const g,
 AstarHeuristics& fValue,
 CompareAstarNode& compare,
 CheckAstarFin& checkAstarFin,
 CheckInvalidAstarNode& checkInvalidAstarNode)
{
  return getPathBySpaceTimeAstar
    (s, g, fValue, compare, checkAstarFin, checkInvalidAstarNode, getRemainedTime());
}

void Solver::start()
{
  info("  start solving MAPF by", solver_name);
  t_start = std::chrono::system_clock::now();
}

// failed & solution is empty -> add solution the initial configuration
void Solver::end()
{
  comp_time = getSolverElapsedTime();
  info("  finish, elapsed=", comp_time);
  if (!solved && solution.empty()) solution.add(P->getConfigStart());
}

int Solver::getSolverElapsedTime() const { return getElapsedTime(t_start); }
int Solver::getRemainedTime() const {
  return std::max(0, max_comp_time - getSolverElapsedTime());
}

bool Solver::overCompTime() const
{
  return getSolverElapsedTime() >= max_comp_time;
}

void Solver::computeLowerBounds()
{
  LB_soc = 0;
  LB_makespan = 0;

  for (int i = 0; i < P->getNum(); ++i) {
    int d = pathDist(i);
    LB_soc += d;
    if (d > LB_makespan) LB_makespan = d;
  }
}

int Solver::getLowerBoundSOC()
{
  if (LB_soc == 0) computeLowerBounds();
  return LB_soc;
}

int Solver::getLowerBoundMakespan()
{
  if (LB_makespan == 0) computeLowerBounds();
  return LB_makespan;
}

void Solver::createDistanceTable()
{
  for (int i = 0; i < P->getNum(); ++i) {
    // breadth first search
    std::queue<Node*> OPEN;
    Node* n = P->getGoal(i);
    OPEN.push(n);
    DistanceTable[i][n->id] = 0;
    while (!OPEN.empty()) {
      n = OPEN.front();
      OPEN.pop();
      const int d_n = DistanceTable[i][n->id];
      for (auto m : n->neighbor) {
        const int d_m = DistanceTable[i][m->id];
        if (d_n + 1 >= d_m) continue;
        DistanceTable[i][m->id] = d_n + 1;
        OPEN.push(m);
      }
    }
  }
}

int Solver::pathDist(const int i, Node* const s) const {
  if (DistanceTable_p != nullptr) {
    return DistanceTable_p->operator[](i)[s->id];
  }
  return DistanceTable[i][s->id];
}

int Solver::pathDist(const int i) const {
  return pathDist(i, P->getStart(i));
}

void Solver::printResult()
{
  std::cout << "solved=" << solved << ", solver=" << std::right << std::setw(8)
            << solver_name << ", comp_time(ms)=" << std::right << std::setw(8)
            << comp_time << ", soc=" << std::right << std::setw(6)
            << solution.getSOC() << " (LB=" << std::right << std::setw(6)
            << getLowerBoundSOC() << ")"
            << ", makespan=" << std::right << std::setw(4)
            << solution.getMakespan() << " (LB=" << std::right << std::setw(6)
            << getLowerBoundMakespan() << ")" << std::endl;
}

void Solver::makeLog(const std::string& logfile)
{
  std::ofstream log;
  log.open(logfile, std::ios::out);
  makeLogBasicInfo(log);
  makeLogSolution(log);
  log.close();
}

void Solver::makeLogBasicInfo(std::ofstream& log)
{
  Grid* grid = reinterpret_cast<Grid*>(P->getG());
  log << "instance=" << P->getInstanceFileName() << "\n";
  log << "agents=" << P->getNum() << "\n";
  log << "map_file=" << grid->getMapFileName() << "\n";
  log << "solver=" << solver_name << "\n";
  log << "solved=" << solved << "\n";
  log << "soc=" << solution.getSOC() << "\n";
  log << "lb_soc=" << getLowerBoundSOC() << "\n";
  log << "makespan=" << solution.getMakespan() << "\n";
  log << "lb_makespan=" << getLowerBoundMakespan() << "\n";
  log << "comp_time=" << comp_time << "\n";
}

void Solver::makeLogSolution(std::ofstream& log)
{
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
}
