#pragma once
#include "graph.hpp"
#include "problem.hpp"
#include "util.hpp"
#include <unordered_map>
#include <queue>
#include <getopt.h>
#include <chrono>

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


class Solver {
private:
  // cache
  std::unordered_map<std::string, Path> PATH_TABLE;
  static std::string getPathTableKey(Node* s, Node* g);
  void registerPath(const Path& path);
  Path AstarSearch(Node* s, Node* g);

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
  void start();
  void end();

public:
  Solver(Problem* _P);
  ~Solver() {};

  virtual void solve() {};
  virtual void setParams(int argc, char *argv[]) {};

  static void setVerbose(bool _verbose) { verbose = _verbose; }
  std::string getSolverName() { return solver_name; };
  void makeLog(const std::string& logfile="./result.txt");
};
