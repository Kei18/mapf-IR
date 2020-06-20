#pragma once
#include "graph.hpp"
#include "problem.hpp"
#include "util.hpp"
#include <unordered_map>
#include <queue>
#include <getopt.h>
#include <chrono>
#include<functional>


struct Plan {
  std::vector<Config> configs;

  Config at(int t) {
    if (!(0 <= t && t < configs.size())) {
      halt("invalid timestep.");
    }
    return configs[t];
  }

  void add(const Config& c) {
    configs.push_back(c);
  }

  int getMakespan() {
    return configs.size() - 1;
  }

  int getSOC() {
    int makespan = getMakespan();
    if (makespan <= 0) return 0;
    int num_agents = configs[0].size();
    int soc = 0;
    for (int i = 0; i < num_agents; ++i) {
      int c = makespan;
      Node* g = configs[makespan][i];
      while (configs[c][i] == g) {
        --c;
        if (c <= 0) break;
      }
      soc += c + 1;
    }
    return soc;
  }
};

struct Paths {
  std::vector<Path> paths;

  void initialize(int num_agents) {
    paths.clear();
    std::vector<Path> tmp(num_agents, Path(0));
    paths = tmp;
  }

  Path get(int i) const {
    if (!(0 <= i && i < paths.size())) {
      halt("invalid index.");
    }
    return paths[i];
  }

  void insert(int i, const Path& path) {
    if (!(0 <= i && i < paths.size())) {
      halt("invalid index.");
    }
    paths[i] = path;
    format();
  }

  void operator+=(const Paths& other) {
    if (paths.size() != other.paths.size()) halt("invalid operation.");
    if (getMakespan() == 0) {  // empty
      paths = other.paths;
    } else {
      for (int i = 0; i < paths.size(); ++i) {
        if (paths[i].empty()) {
          if (!other.paths[i].empty()) halt("invalid operation");
          continue;
        }
        if (*(paths[i].end()-1) != other.paths[i][0])
          halt("invalid operation");
        for (int t = 1; t < other.paths[i].size(); ++t) {
          paths[i].push_back(other.paths[i][t]);
        }
      }
    }
  }

  int getMakespan() {
    int max_val = 0;
    for (auto p : paths) {
      if (p.empty()) continue;
      max_val = (p.size() - 1 > max_val) ? p.size() - 1 : max_val;
    }
    return max_val;
  }

  int getSOC() {
    int soc = 0;
    for (auto p : paths) {
      int c = p.size();
      auto itr = p.end() - 1;
      Node* g = *itr;
      while (*itr == g) {
        --c;
        if (c <= 0) break;
        --itr;
      }
      soc += c;
    }
    return soc;
  }

  void format() {
    int makespan = getMakespan();
    for (int i = 0; i < paths.size(); ++i) {
      if (paths[i].empty()) continue;
      while (paths[i].size()-1 != makespan) {
        paths[i].push_back(*(paths[i].end()-1));
      }
    }
  }

  Plan toPlan() {
    Plan plan;
    int makespan = getMakespan();
    for (int t = 0; t <= makespan; ++t) {
      Config c;
      for (int i = 0; i < paths.size(); ++i) {
        c.push_back(paths[i][t]);
      }
      plan.add(c);
    }
    return plan;
  }
};

struct AstarNode {
  Node* v;
  int g;  // in getTimedPath, g represents t
  int f;
  AstarNode* p;  // parent
};

using CompareAstarNode = std::function<bool(AstarNode*, AstarNode*)>;
using CheckAstarFin = std::function<bool(AstarNode*)>;
using CheckInvalidAstarNode = std::function<bool(AstarNode*)>;

class Solver {
private:
  // cache
  std::unordered_map<std::string, Path> PATH_TABLE;
  static std::string getPathTableKey(Node* s, Node* g);
  void registerPath(const Path& path);
  Path getPathOnG(Node* s, Node* g);

protected:
  std::string solver_name;

  Problem* P;
  Graph* G;
  std::mt19937* MT;

  const int max_timestep;
  const int max_comp_time;

  static bool verbose;

  Plan solution;
  bool solved;
  std::chrono::system_clock::time_point t_start;
  double comp_time;

  Path getPath(Node* s, Node* g);
  int pathDist(Node* s, Node* g);
  Path getTimedPath(Node* s,
                    Node* g,
                    CompareAstarNode& compare,
                    CheckAstarFin& checkAstarFin,
                    CheckInvalidAstarNode& checkInvalidAstarNode);
  void start();
  void end();
  double getSolverElapsedTime() const;
  bool overCompTime() const;

public:
  Solver(Problem* _P);
  ~Solver() {};

  virtual void solve() {};
  virtual void setParams(int argc, char *argv[]) {};

  static void setVerbose(bool _verbose) { verbose = _verbose; }
  std::string getSolverName() { return solver_name; };
  void makeLog(const std::string& logfile="./result.txt");
};
