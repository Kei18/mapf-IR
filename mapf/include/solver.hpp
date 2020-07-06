#pragma once
#include "graph.hpp"
#include "problem.hpp"
#include "default_params.hpp"
#include "util.hpp"
#include "paths.hpp"
#include "plan.hpp"
#include <unordered_map>
#include <queue>
#include <getopt.h>
#include <chrono>
#include<functional>

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

  Plan solution;
  double comp_time;
  bool solved;

  bool verbose;

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

  Path getPath(Node* const s, Node* const g) { return G->getPath(s, g); };
  int pathDist(Node* const s, Node* const g) { return G->pathDist(s, g); };
  int pathDist(int i) { return G->pathDist(P->getStart(i),P->getGoal(i));};
  Path getTimedPath(Node* const s,
                    Node* const g,
                    AstarHeuristics& fValue,
                    CompareAstarNode& compare,
                    CheckAstarFin& checkAstarFin,
                    CheckInvalidAstarNode& checkInvalidAstarNode);

  double getSolverElapsedTime() const;
  bool overCompTime() const;

  static Paths planToPaths(const Plan& plan);
  static Plan pathsToPlan(const Paths& paths);

  void info() const { if (verbose) std::cout << std::endl; }
  template <class Head, class... Tail>
  void info(Head&& head, Tail&&... tail) const {
    if (!verbose) return;
    std::cout << head << " ";
    info(std::forward<Tail>(tail)...);
  }

  virtual void run() {}

private:
  void start();
  void end();

public:
  Solver(Problem* _P);
  virtual ~Solver() {};

  void solve();
  virtual void setParams(int argc, char *argv[]) {};
  void setVerbose(bool _verbose) { verbose = _verbose; }
  void printResult();
  virtual void makeLog(const std::string& logfile="./result.txt");

  Plan getSolution() const { return solution; };
  bool succeed() const { return solved; };
  std::string getSolverName() { return solver_name; };
};
