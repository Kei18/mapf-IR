#include "../include/solver.hpp"

#include <fstream>

#include "../include/lib_cbs.hpp"

Solver::Solver(Problem* _P)
    : P(_P),
      G(_P->getG()),
      MT(_P->getMT()),
      max_timestep(P->getMaxTimestep()),
      max_comp_time(P->getMaxCompTime())
{
  solved = false;
  verbose = false;

  // for solvers using MDD
  LibCBS::MDD::MT = MT;
}

void Solver::solve()
{
  start();
  run();
  end();
}

/*
 * Template of Space-Time A*.
 * See the following reference.
 *
 * Cooperative Pathﬁnding.
 * D. Silver.
 * AI Game Programming Wisdom 3, pages 99–111, 2006.
 */
Path Solver::getTimedPath(Node* const s, Node* const g, AstarHeuristics& fValue,
                          CompareAstarNode& compare,
                          CheckAstarFin& checkAstarFin,
                          CheckInvalidAstarNode& checkInvalidAstarNode)
{
  auto getNodeName = [](AstarNode* n) {
    return std::to_string(n->v->id) + "-" + std::to_string(n->g);
  };

  std::vector<AstarNode*> GC;  // garbage collection
  auto createNewNode = [&](Node* v, int g, int f, AstarNode* p) {
    AstarNode* new_node = new AstarNode{v, g, f, p};
    GC.push_back(new_node);
    return new_node;
  };

  // OPEN and CLOSE list
  std::priority_queue<AstarNode*, std::vector<AstarNode*>, CompareAstarNode>
      OPEN(compare);
  std::unordered_map<std::string, bool> CLOSE;

  // initial node
  AstarNode* n = createNewNode(s, 0, 0, nullptr);
  n->f = fValue(n);
  OPEN.push(n);

  // main loop
  bool invalid = true;
  while (!OPEN.empty()) {
    // check time limit
    if (overCompTime()) break;

    // minimum node
    n = OPEN.top();
    OPEN.pop();

    // check CLOSE list
    if (CLOSE.find(getNodeName(n)) != CLOSE.end()) continue;
    CLOSE[getNodeName(n)] = true;

    // check goal condition
    if (checkAstarFin(n)) {
      invalid = false;
      break;
    }

    // expand
    Nodes C = n->v->neighbor;
    C.push_back(n->v);
    for (auto u : C) {
      int g_cost = n->g + 1;
      AstarNode* m = createNewNode(u, g_cost, 0, n);
      m->f = fValue(m);
      // already searched?
      if (CLOSE.find(getNodeName(m)) != CLOSE.end()) continue;
      // check constraints
      if (checkInvalidAstarNode(m)) continue;
      OPEN.push(m);
    }
  }

  Path path;
  if (!invalid) {  // success
    while (n != nullptr) {
      path.push_back(n->v);
      n = n->p;
    }
    std::reverse(path.begin(), path.end());
  }

  // free
  for (auto p : GC) delete p;

  return path;
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

double Solver::getSolverElapsedTime() const { return getElapsedTime(t_start); }

bool Solver::overCompTime() const
{
  return getSolverElapsedTime() >= max_comp_time;
}

// convert Plan to Paths
Paths Solver::planToPaths(const Plan& plan)
{
  if (plan.empty()) halt("invalid operation.");
  int num_agents = plan.get(0).size();
  Paths paths(num_agents);
  int makespan = plan.getMakespan();
  for (int i = 0; i < num_agents; ++i) {
    Path path;
    for (int t = 0; t <= makespan; ++t) {
      path.push_back(plan.get(t, i));
    }
    paths.insert(i, path);
  }
  return paths;
}

// convert Paths to Plan
Plan Solver::pathsToPlan(const Paths& paths)
{
  Plan plan;
  if (paths.empty()) return plan;
  int makespan = paths.getMakespan();
  int num_agents = paths.size();
  for (int t = 0; t <= makespan; ++t) {
    Config c;
    for (int i = 0; i < num_agents; ++i) {
      c.push_back(paths.get(i, t));
    }
    plan.add(c);
  }
  return plan;
}

void Solver::printResult()
{
  int LB_soc = 0;
  int LB_makespan = 0;
  for (int i = 0; i < P->getNum(); ++i) {
    int d = pathDist(i);
    LB_soc += d;
    if (d > LB_makespan) LB_makespan = d;
  }

  std::cout << "solved=" << solved << ", solver=" << std::right << std::setw(8)
            << solver_name << ", comp_time(ms)=" << std::right << std::setw(8)
            << comp_time << ", soc=" << std::right << std::setw(6)
            << solution.getSOC() << " (LB=" << std::right << std::setw(6)
            << LB_soc << ")"
            << ", makespan=" << std::right << std::setw(4)
            << solution.getMakespan() << " (LB=" << std::right << std::setw(6)
            << LB_makespan << ")" << std::endl;
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
  log << "makespan=" << solution.getMakespan() << "\n";
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
