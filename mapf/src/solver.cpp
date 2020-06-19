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
}

Path Solver::getPath(Node* s, Node* g)
{
  if (s == g) return {};

  // check cache
  std::string key = getPathTableKey(s, g);
  auto itr = PATH_TABLE.find(key);
  if (itr != PATH_TABLE.end()) return itr->second;

  Path path = AstarSearch(s, g);
  registerPath(path);
  return path;
}

int Solver::pathDist(Node* s, Node* g)
{
  if (s == g) return 0;

  // check cache
  std::string key = getPathTableKey(s, g);
  auto itr = PATH_TABLE.find(key);
  if (itr != PATH_TABLE.end()) return itr->second.size() - 1;

  Path path = AstarSearch(s, g);
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

Path Solver::AstarSearch(Node* s, Node* g)
{
  struct AstarNode {
    Node* v;
    int g;
    int f;
    AstarNode* p;  // parent
  };

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

std::string Solver::getPathTableKey(Node* s, Node* g) {
  return std::to_string(s->id) + "-" + std::to_string(g->id);
}

void Solver::start() {
  info("start solving MAPF by", solver_name);
  t_start = std::chrono::system_clock::now();
}

void Solver::end() {
  comp_time = getElapsedTime(t_start);
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
