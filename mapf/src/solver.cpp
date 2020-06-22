#include "../include/solver.hpp"
#include "../include/util.hpp"
#include <fstream>

bool Solver::verbose = false;

Solver::Solver(Problem* _P)
  : P(_P),
    G(_P->getG()),
    MT(_P->getMT()),
    max_timestep(P->getMaxTimestep()),
    max_comp_time(P->getMaxCompTime())
{
  VERVOSE = verbose;  // def in util.hpp
  solved = false;
}

Path Solver::getPath(Node* const s, Node* const g)
{
  if (s == g) return {};

  // check cache
  std::string key = getPathTableKey(s, g);
  auto itr = PATH_TABLE.find(key);
  if (itr != PATH_TABLE.end()) return itr->second;

  Path path = getPathOnG(s, g);
  registerPath(path);
  return path;
}

int Solver::pathDist(Node* const s, Node* const g)
{
  if (s == g) return 0;

  // check cache
  std::string key = getPathTableKey(s, g);
  auto itr = PATH_TABLE.find(key);
  if (itr != PATH_TABLE.end()) return itr->second.size() - 1;

  Path path = getPathOnG(s, g);
  registerPath(path);
  return path.size() - 1;
}

void Solver::registerPath(const Path& path)
{
  if (path.empty()) return;
  Nodes tmp = path;
  Node* v;
  Node* g = *(path.end() - 1);
  do {
    v = tmp[0];
    PATH_TABLE[getPathTableKey(v, g)] = tmp;
    tmp.erase(tmp.begin());
  } while (tmp.size() > 2);
}

// A* search but using cache as much as possible
Path Solver::getPathOnG(Node* const s, Node* const g)
{
  auto compare = [&] (AstarNode* a, AstarNode* b) {
                   if (a->f != b->f) return a->f > b->f;
                   if (a->g != b->g) return a->g < b->g;
                   return getRandomBoolean(MT); };

  // OPEN and CLOSE
  std::priority_queue<AstarNode*,
                      std::vector<AstarNode*>,
                      decltype(compare)> OPEN(compare);
  std::unordered_map<int, bool> CLOSE;

  // initial node
  AstarNode* n;
  n = new AstarNode { s, 0, G->dist(s, g), nullptr };
  OPEN.push(n);

  bool invalid = true;
  while (!OPEN.empty()) {
    n = OPEN.top();
    OPEN.pop();

    // check CLOSE list
    if (CLOSE.find(n->v->id) != CLOSE.end()) continue;
    CLOSE[n->v->id] = true;

    // check goal condition
    if (n->v == g) {
      invalid = false;
      break;
    }

    // use cache
    auto itr = PATH_TABLE.find(getPathTableKey(n->v, g));
    if (itr != PATH_TABLE.end()) {
      Path path = itr->second;
      for (int t = 1; t < path.size(); ++t)
        n = new AstarNode { path[t], 0, 0, n };
      invalid = false;
      break;
    }

    // expand
    Nodes C = n->v->neighbor;
    C.push_back(n->v);
    for (auto u : C) {
      // already searched?
      if (CLOSE.find(u->id) != CLOSE.end()) continue;
      int g_value = n->g + 1;
      int h_value = g_value + G->dist(u, g);
      // use real cost whenever available
      auto itr = PATH_TABLE.find(getPathTableKey(u, g));
      if (itr != PATH_TABLE.end()) {
        h_value = g_value + itr->second.size() - 1;
      }
      AstarNode* m = new AstarNode { u, g_value, h_value, n };
      OPEN.push(m);
    }
  }

  if (invalid) halt("graph contains unreachable nodes.");

  Path path;
  while (n != nullptr) {
    path.push_back(n->v);
    n = n->p;
  }
  std::reverse(path.begin(), path.end());
  return path;
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
  std::unordered_map<std::string, bool> CLOSE;

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
  // failed
  if (invalid) return path;
  // success
  while (n != nullptr) {
    path.push_back(n->v);
    n = n->p;
  }
  std::reverse(path.begin(), path.end());
  return path;
}

std::string Solver::getPathTableKey(Node* const s, Node* const g) {
  return std::to_string(s->id) + "-" + std::to_string(g->id);
}

void Solver::start() {
  info("start solving MAPF by", solver_name);
  t_start = std::chrono::system_clock::now();
}

void Solver::end() {
  comp_time = getSolverElapsedTime();
  info("finish");
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

double Solver::getSolverElapsedTime() const {
  return getElapsedTime(t_start);
}

bool Solver::overCompTime() const {
  return getSolverElapsedTime() >= max_comp_time;
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
    auto c = solution.at(t);
    for (auto v : c) {
      log << "(" << v->pos.x << "," << v->pos.y << "),";
    }
    log << "\n";
  }
  log.close();
}
