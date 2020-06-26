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

struct Paths;

struct Plan {
private:
  std::vector<Config> configs;

public:
  Config get(int t) const {
    if (!(0 <= t && t < configs.size())) {
      halt("invalid timestep.");
    }
    return configs[t];
  }

  Node* get(int t, int i) const {
    if (empty()) halt("invalid operation");
    if (!(0 <= t && t < configs.size())) {
      halt("invalid timestep.");
    }
    if (!(0 <= i && i < configs[0].size())) {
      halt("invalid agent id.");
    }
    return configs[t][i];
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
    Config c2 = other.get(0);
    if (c1.size() != c2.size()) halt("invalid operation");
    for (int i = 0; i < c1.size(); ++i) {
      if (c1[i] != c2[i]) halt("invalid operation.");
    }
    // merge
    Plan new_plan;
    new_plan.configs = configs;
    for (int t = 1; t < other.size(); ++t) new_plan.add(other.get(t));
    return new_plan;
  }

  void operator+=(const Plan& other) {
    if (configs.empty()) {
      configs = other.configs;
      return;
    }
    // check validity
    if (!sameConfig(last(), other.get(0))) halt("invalid operation");
    // merge
    for (int t = 1; t < other.size(); ++t) add(other.get(t));
  }

  Plan getPartialPlan(const Config& config_i, const Config& config_j) const
  {
    int t_s = 0;
    while (!sameConfig(get(t_s), config_i)) {
      ++t_s;
      if (t_s > getMakespan()) halt("invalid operation");
    }

    Plan partial_plan;
    for (int t = t_s; t <= getMakespan(); ++t) {
      partial_plan.add(get(t));
      if (sameConfig(get(t), config_j)) break;
      if (t == getMakespan()) halt("invalid operation");
    }
    return partial_plan;
  }

  Plan getPartialPlan(int i, int j) const
  {
    if (!(0 <= i && i <= j && j <= getMakespan())) halt("invalid index.");
    Plan new_plan;
    for (int t = i; t <= j; ++t) new_plan.add(get(t));
    return new_plan;
  }

  bool validate(Problem* P) const
  {
    if (configs.empty()) return false;
    // start and goal
    if (!sameConfig(P->getConfigStart(), get(0))) return false;
    if (!sameConfig(P->getConfigGoal(), get(getMakespan()))) return false;
    int num_agents = get(0).size();
    for (int t = 1; t < getMakespan(); ++t) {
      if (get(t).size() != num_agents) return false;
      for (int i = 0; i < num_agents; ++i) {
        Node* v_i_t = get(t, i);
        Node* v_i_t_1 = get(t-1, i);
        Nodes cands = v_i_t_1->neighbor;
        cands.push_back(v_i_t_1);
        if (!inArray(v_i_t, cands)) return false;
        for (int j = i+1; j < num_agents; ++j) {
          Node* v_j_t = get(t, j);
          Node* v_j_t_1 = get(t-1, j);
          if (v_i_t == v_j_t) return false;
          if (v_i_t == v_j_t_1 && v_i_t_1 == v_j_t) return false;
        }
      }
    }
    return true;
  }
};

using Plans = std::vector<Plan>;

struct Paths {
private:
  std::vector<Path> paths;
  int makespan;

public:
  Paths() {}
  Paths(int num_agents) {
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
      halt("invalid index, i=" + std::to_string(i)
           + ", t=" + std::to_string(t));
    }
    return paths[i][t];
  }

  bool empty() const {
    return paths.empty();
  }

  void insert(int i, const Path& path) {
    if (!(0 <= i && i < paths.size())) halt("invalid index.");
    int old_len = paths[i].size();
    paths[i] = path;
    format();
    if (paths[i].size() < old_len) shrink();
    makespan = getMaxLengthPaths();  // update makespan
  }

  int size() const {
    return paths.size();
  }

  void operator+=(const Paths& other) {
    if (paths.size() != other.paths.size()) halt("invalid operation.");
    if (makespan == 0) {// empty
      for (int i = 0; i < paths.size(); ++i) insert(i, other.get(i));
    } else {
      std::vector<Path> new_paths(paths.size());
      for (int i = 0; i < paths.size(); ++i) {
        if (paths[i].empty()) halt("invalid operation");
        if (*(paths[i].end()-1) != other.paths[i][0])
          halt("invalid operation");
        Path tmp;
        for (int t = 0; t <= getMakespan(); ++t)
          tmp.push_back(get(i, t));
        for (int t = 1; t <= other.getMakespan(); ++t) {
          tmp.push_back(other.get(i, t));
        }
        new_paths[i] = tmp;
      }
      for (int i = 0; i < paths.size(); ++i) insert(i, new_paths[i]);
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
};

static Paths planToPaths(const Plan& plan) {
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

static Plan pathsToPlan(const Paths& paths) {
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
  virtual ~Solver() {};

  virtual void solve() {};
  virtual void setParams(int argc, char *argv[]) {};
  void setVerbose(bool _verbose) { verbose = _verbose; }
  void printResult();
  void makeLog(const std::string& logfile="./result.txt");

  Plan getSolution() const { return solution; };
  bool succeed() const { return solved; };
  std::string getSolverName() { return solver_name; };
};
