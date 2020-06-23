#pragma once
#include "graph.hpp"
#include "problem.hpp"
#include "default_params.hpp"
#include "util.hpp"
#include <unordered_map>
#include <queue>
#include <getopt.h>
#include <chrono>
#include<functional>


struct Plan {
  std::vector<Config> configs;

  Config at(int t) const {
    if (!(0 <= t && t < configs.size())) {
      halt("invalid timestep.");
    }
    return configs[t];
  }

  Config last() const {
    return configs[configs.size()-1];
  }

  void add(const Config& c) {
    if (!configs.empty() && configs.at(0).size() != c.size()) {
      halt("invalid operation");
    }
    configs.push_back(c);
  }

  bool empty() const {
    return configs.empty();
  }

  int size() const {
    return configs.size();
  }

  int getMakespan() const {
    return configs.size() - 1;
  }

  int getSOC() const {
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

  Plan operator+(const Plan& other) const {
    // check validity
    Config c1 = last();
    Config c2 = other.at(0);
    if (c1.size() != c2.size()) halt("invalid operation");
    for (int i = 0; i < c1.size(); ++i) {
      if (c1[i] != c2[i]) halt("invalid operation.");
    }
    // merge
    Plan new_plan;
    new_plan.configs = configs;
    for (int t = 1; t < other.size(); ++t) new_plan.add(other.at(t));
    return new_plan;
  }

  void operator+=(const Plan& other) {
    if (configs.empty()) {
      configs = other.configs;
      return;
    }
    // check validity
    if (!sameConfig(last(), other.at(0))) halt("invalid operation");
    // merge
    for (int t = 1; t < other.size(); ++t) add(other.at(t));
  }

  Plan getPartialPlan(const Config& config_i, const Config& config_j) const
  {
    int t_s = 0;
    while (!sameConfig(at(t_s), config_i)) {
      ++t_s;
      if (t_s > getMakespan()) halt("invalid operation");
    }

    Plan partial_plan;
    for (int t = t_s; t <= getMakespan(); ++t) {
      partial_plan.add(at(t));
      if (sameConfig(at(t), config_j)) break;
      if (t == getMakespan()) halt("invalid operation");
    }
    return partial_plan;
  }

  Plan getPartialPlan(int i, int j) const
  {
    if (!(0 <= i && i <= j && j <= getMakespan())) halt("invalid index.");
    Plan new_plan;
    for (int t = i; t <= j; ++t) new_plan.add(at(t));
    return new_plan;
  }
};

using Plans = std::vector<Plan>;

struct Paths {
  std::vector<Path> paths;
  int makespan;

  void initialize(int num_agents) {
    paths.clear();
    std::vector<Path> tmp(num_agents, Path(0));
    paths = tmp;
  }

  Path get(int i) const {
    if (!(0 <= i && i < paths.size())) halt("invalid index.");
    return paths[i];
  }

  Node* get(int i, int t) const {
    if (!(0 <= i && i < paths.size()) ||
        !(0 <= t && t <= makespan)) {
      halt("invalid index.");
    }
    return paths[i][t];
  }

  void insert(int i, const Path& path) {
    if (!(0 <= i && i < paths.size())) halt("invalid index.");
    int old_len = paths[i].size();
    paths[i] = path;
    format();
    if (paths[i].size() < old_len) shrink();
    makespan = getMaxLengthPaths();  // update makespan
  }

  void operator+=(const Paths& other) {
    if (paths.size() != other.paths.size()) halt("invalid operation.");
    if (makespan == 0) {// empty
      for (int i = 0; i < paths.size(); ++i) insert(i, other.get(i));
    } else {
      for (int i = 0; i < paths.size(); ++i) {
        if (paths[i].empty()) {
          if (!other.paths[i].empty()) halt("invalid operation");
          continue;
        }
        if (*(paths[i].end()-1) != other.paths[i][0])
          halt("invalid operation");
        Path tmp;
        for (int t = 0; t <= getMakespan(); ++t)
          tmp.push_back(get(i, t));
        for (int t = 1; t <= other.getMakespan(); ++t) {
          tmp.push_back(other.get(i, t));
        }
        insert(i, tmp);
      }
    }
  }

  int getMaxLengthPaths() const {
    int max_val = 0;
    for (auto p : paths) {
      if (p.empty()) continue;
      max_val = (p.size() - 1 > max_val) ? p.size() - 1 : max_val;
    }
    return max_val;
  }

  int getMakespan() const {
    return makespan;
  }

  int costOfPath(int i) const {
    if (!(0 <= i && i < paths.size())) halt("invalid index.");
    int c = paths[i].size();
    auto itr = paths[i].end() - 1;
    Node* g = *itr;
    while (*itr == g) {
      --c;
      if (c <= 0) break;
      --itr;
    }
    return c;
  }

  int getSOC() const {
    int soc = 0;
    for (int i = 0; i < paths.size(); ++i) soc += costOfPath(i);
    return soc;
  }

  void format() {
    int len = getMaxLengthPaths();
    for (int i = 0; i < paths.size(); ++i) {
      if (paths[i].empty()) continue;
      while (paths[i].size()-1 != len) {
        paths[i].push_back(*(paths[i].end()-1));
      }
    }
  }

  void shrink() {
    while (true) {
      bool shrinkable = true;
      for (auto p: paths) {
        if (p.size() <= 1 || *(p.end()-1) != *(p.end()-2)) {
          shrinkable = false;
          break;
        }
      }
      if (!shrinkable) break;
      for (int i = 0; i < paths.size(); ++i) {
        paths[i].resize(paths[i].size()-1);
      }
    }
  }

  Plan toPlan() const {
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


class Solver {
protected:
  std::string solver_name;

  Problem* P;
  Graph* G;
  std::mt19937* MT;

  // limitation
  const int max_timestep;
  const int max_comp_time;
  std::chrono::system_clock::time_point t_start;

  struct AstarNode {
    Node* v;
    int g;  // in getTimedPath, g represents t
    int f;
    AstarNode* p;  // parent
  };
  using CompareAstarNode = std::function<bool(AstarNode*, AstarNode*)>;
  using CheckAstarFin = std::function<bool(AstarNode*)>;
  using CheckInvalidAstarNode = std::function<bool(AstarNode*)>;
  using AstarHeuristics = std::function<int(AstarNode*)>;

  Plan solution;
  bool solved;
  double comp_time;

  Path getPath(Node* const s, Node* const g) { return G->getPath(s, g); };
  int pathDist(Node* const s, Node* const g) { return G->pathDist(s, g); };
  Path getTimedPath(Node* const s,
                    Node* const g,
                    AstarHeuristics& fValue,
                    CompareAstarNode& compare,
                    CheckAstarFin& checkAstarFin,
                    CheckInvalidAstarNode& checkInvalidAstarNode);
  void start();
  void end();
  double getSolverElapsedTime() const;
  bool overCompTime() const;

  // verbose, TODO: -> src
  bool verbose;
  void info() const { if (verbose) std::cout << std::endl; }
  template <class Head, class... Tail>
  void info(Head&& head, Tail&&... tail) const {
    if (!verbose) return;
    std::cout << head << " ";
    info(std::forward<Tail>(tail)...);
  }

public:
  Solver(Problem* _P);
  ~Solver() {};

  virtual void solve() {};
  virtual void setParams(int argc, char *argv[]) {};
  void setVerbose(bool _verbose) { verbose = _verbose; }
  void printResult();
  void makeLog(const std::string& logfile="./result.txt");

  Plan getSolution() const { return solution; };
  bool succeed() const { return solved; };
  std::string getSolverName() { return solver_name; };
};
