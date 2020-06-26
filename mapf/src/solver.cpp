#include "../include/solver.hpp"
#include <fstream>


Solver::Solver(Problem* _P)
  : P(_P),
    G(_P->getG()),
    MT(_P->getMT()),
    max_timestep(P->getMaxTimestep()),
    max_comp_time(P->getMaxCompTime())
{
  solved = false;
  verbose = false;
}

// pure A* search
Path Solver::getTimedPath(Node* const s,
                          Node* const g,
                          AstarHeuristics& fValue,
                          CompareAstarNode& compare,
                          CheckAstarFin& checkAstarFin,
                          CheckInvalidAstarNode& checkInvalidAstarNode)
{
  auto getNodeName = [] (AstarNode* n)
                     { return std::to_string(n->v->id)
                         + "-" + std::to_string(n->g); };
  // OPEN and CLOSE
  std::priority_queue<AstarNode*,
                      std::vector<AstarNode*>,
                      CompareAstarNode> OPEN(compare);
  std::unordered_map<std::string, AstarNode*> CLOSE;

  // initial node
  AstarNode* n;
  n = new AstarNode { s, 0, 0, nullptr };
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
    CLOSE[getNodeName(n)] = n;

    // check goal condition
    if (checkAstarFin(n)) {
      invalid = false;
      break;
    }

    // expand
    Nodes C = n->v->neighbor;
    C.push_back(n->v);
    for (auto u : C) {
      int g_cost = n->g+1;
      AstarNode* m = new AstarNode { u, g_cost, 0, n };
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
  while (!OPEN.empty()) {
    delete OPEN.top();
    OPEN.pop();
  }
  for (auto itr = CLOSE.begin(); itr != CLOSE.end(); ++itr) {
    delete itr->second;
  }

  return path;
}

void Solver::start() {
  info("  start solving MAPF by", solver_name);
  t_start = std::chrono::system_clock::now();
}

void Solver::end() {
  comp_time = getSolverElapsedTime();
  // format
  if (solved && !solution.validate(P)) {
    warn("failed to solve.");
    solved = false;
  }
  if (!solved && solution.empty()) solution.add(P->getConfigStart());
}

double Solver::getSolverElapsedTime() const {
  return getElapsedTime(t_start);
}

bool Solver::overCompTime() const {
  return getSolverElapsedTime() >= max_comp_time;
}

void Solver::printResult() {
  info("  finish");
  std::cout << "solved=" << solved
            << ", solver=" << std::right << std::setw(8)
            << solver_name
            << ", comp_time(ms)=" << std::right << std::setw(8)
            << comp_time
            << ", soc=" << std::right << std::setw(6)
            << solution.getSOC()
            << ", makespan=" << std::right << std::setw(5)
            << solution.getMakespan()
            << std::endl;
}

void Solver::makeLog(const std::string& logfile) {
  std::ofstream log;
  log.open(logfile, std::ios::out);
  log << "instance= " << P->getInstanceFileName() << "\n";
  log << "agents=" << P->getNum() << "\n";
  log << "map_file=" << P->getG()->getMapFileName() << "\n";
  log << "solver=" << solver_name << "\n";
  log << "solved=" << solved << "\n";
  log << "soc=" << solution.getSOC() << "\n";
  log << "makespan=" << solution.getMakespan() << "\n";
  log << "comp_time=" << comp_time << "\n";
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
