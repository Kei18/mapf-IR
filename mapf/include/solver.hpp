#pragma once
#include <getopt.h>

#include <chrono>
#include <functional>
#include <queue>
#include <unordered_map>

#include "default_params.hpp"
#include "graph.hpp"
#include "paths.hpp"
#include "plan.hpp"
#include "problem.hpp"
#include "lib_solver.hpp"
#include "util.hpp"

class Solver
{
protected:
  std::string solver_name;

  Problem* P;
  Graph* G;
  std::mt19937* MT;

  // limitations
  const int max_timestep;
  const int max_comp_time;

  Time::time_point t_start;  // when to start solving

  Plan solution;     // solution
  double comp_time;  // time for deliberation
  bool solved;       // success -> true, failed -> false (default)

  bool verbose;  // true -> print additional info

  int LB_soc;       // lower bound of soc
  int LB_makespan;  // lower bound of makespan

  Path getPath(Node* const s, Node* const g) const { return G->getPath(s, g); }
  int pathDist(Node* const s, Node* const g) const { return G->pathDist(s, g); }
  // get path distance for a_i
  int pathDist(int i) const { return G->pathDist(P->getStart(i), P->getGoal(i)); }


  // space-time A*
  Path getTimedPath(Node* const s,  // start
                    Node* const g,  // goal
                    AstarHeuristics& fValue,
                    CompareAstarNode& compare,
                    CheckAstarFin& checkAstarFin,
                    CheckInvalidAstarNode& checkInvalidAstarNode);

  int getSolverElapsedTime() const;  // get elapsed time from start
  int getRemainedTime() const;  // get remained time
  bool overCompTime() const;            // check time limit

  // print debug info (only when verbose=true)
  void info() const
  {
    if (verbose) std::cout << std::endl;
  }
  template <class Head, class... Tail>
  void info(Head&& head, Tail&&... tail) const
  {
    if (!verbose) return;
    std::cout << head << " ";
    info(std::forward<Tail>(tail)...);
  }

  // main
  virtual void run() {}

  // for log
  void makeLogBasicInfo(std::ofstream& log);
  void makeLogSolution(std::ofstream& log);

private:
  void start();
  void end();

  void computeLowerBounds();

public:
  Solver(Problem* _P);
  virtual ~Solver();

  // call start -> run -> end
  void solve();

  // getter
  Plan getSolution() const { return solution; };
  bool succeed() const { return solved; };
  std::string getSolverName() { return solver_name; };

  int getLowerBoundSOC();
  int getLowerBoundMakespan();

  // for parameters
  virtual void setParams(int argc, char* argv[]){};
  void setVerbose(bool _verbose) { verbose = _verbose; }

  // show result
  void printResult();

  // for log
  virtual void makeLog(const std::string& logfile = "./result.txt");
};
