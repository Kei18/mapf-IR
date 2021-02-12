#pragma once
#include <getopt.h>

#include <chrono>
#include <functional>
#include <memory>
#include <queue>
#include <unordered_map>

#include "default_params.hpp"
#include "graph.hpp"
#include "lib_solver.hpp"
#include "paths.hpp"
#include "plan.hpp"
#include "problem.hpp"
#include "util.hpp"

class Solver
{
protected:
  std::string solver_name;

  Problem* P;        // problem instance
  Graph* G;          // graph
  std::mt19937* MT;  // seed for randomness

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

protected:
  std::vector<std::vector<int>>
      DistanceTable;  // distance table [agent][node_id]
  std::vector<std::vector<int>>*
      DistanceTable_p;  // pointer, used in nested solvers
public:
  // get path distance for a_i
  int pathDist(const int i) const;
  int pathDist(const int i, Node* const s) const;
  void setDistanceTable(std::vector<std::vector<int>>* p)
  {
    DistanceTable_p = p;
  }
  void createDistanceTable();  // preprocessing
  // use search of original graph with cache
  Path getPath(Node* const s, Node* const g) const { return G->getPath(s, g); }
  int pathDist(Node* const s, Node* const g) const { return G->pathDist(s, g); }

public:
  // space-time A*
  Path getTimedPath(Node* const s,  // start
                    Node* const g,  // goal
                    AstarHeuristics& fValue, CompareAstarNode& compare,
                    CheckAstarFin& checkAstarFin,
                    CheckInvalidAstarNode& checkInvalidAstarNode);
  // prioritized planning
  Path getPrioritizedPath(
      const int id,                // agent id
      Node* const s,               // initial location
      Node* const g,               // goal location
      const Paths& paths,          // already reserved paths
      const int time_limit = -1,   // time limit
      const int upper_bound = -1,  // upper bound of timesteps
      const std::vector<std::tuple<Node*, int>>& constraints =
          {},  // additional constraints, space-time
      CompareAstarNode& compare = compareAstarNodeBasic,  // compare two nodes
      const bool manage_path_table =
          true  // manage path table automatically, conflict check
  );

  Path getPrioritizedPath(
      const int id, const Paths& paths, const int time_limit = -1,
      const int upper_bound = -1,
      const std::vector<std::tuple<Node*, int>>& constraints = {},
      CompareAstarNode& compare = compareAstarNodeBasic);

  // for prioritized planning
protected:
  void updatePathTable(const Paths& paths, const int id);
  void clearPathTable(const Paths& paths);
  void updatePathTableWithoutClear(const int id, const Path& p,
                                   const Paths& paths);
  static constexpr int NIL = -1;
  std::vector<std::vector<int>> PATH_TABLE;

public:
  int getSolverElapsedTime() const;  // get elapsed time from start
  int getRemainedTime() const;       // get remained time
  bool overCompTime() const;         // check time limit

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

protected:
  // main
  virtual void run() {}

  // for log
  void makeLogBasicInfo(std::ofstream& log);
  void makeLogSolution(std::ofstream& log);

  // used for set underlying solver options
  static void setSolverOption(std::shared_ptr<Solver> solver,
                              const std::vector<std::string>& option);

private:
  void start();
  void end();

  // get trivial lower bounds of sum-of-costs and makespan
private:
  void computeLowerBounds();

public:
  int getLowerBoundSOC();
  int getLowerBoundMakespan();

public:
  Solver(Problem* _P);
  virtual ~Solver();

  // call start -> run -> end
  void solve();

  // getter
  Plan getSolution() const { return solution; };
  bool succeed() const { return solved; };
  std::string getSolverName() { return solver_name; };
  int getMaxTimestep() const { return max_timestep; };
  Problem* getP() { return P; }

  // for parameters
  virtual void setParams(int argc, char* argv[]){};
  void setVerbose(bool _verbose) { verbose = _verbose; }

  // show result
  void printResult();

  // for log
  virtual void makeLog(const std::string& logfile = "./result.txt");
};
