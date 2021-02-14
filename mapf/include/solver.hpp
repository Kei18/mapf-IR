#pragma once
#include <getopt.h>

#include <chrono>
#include <functional>
#include <memory>
#include <queue>
#include <unordered_map>
#include <functional>

#include "paths.hpp"
#include "plan.hpp"
#include "problem.hpp"
#include "util.hpp"

class MinimumSolver
{
protected:
  std::string solver_name;       // solver name
  Problem* const P;              // problem instance
  Graph* const G;                // graph
  std::mt19937* const MT;        // seed for randomness
  const int max_timestep;        // maximum makespan
  const int max_comp_time;       // time limit for computation, ms
  Plan solution;                 // solution
  bool solved;                   // success -> true, failed -> false (default)

private:
  int comp_time;             // computation time
  Time::time_point t_start;  // when to start solving

public:
  void solve();  // call start -> run -> end
private:
  void start();
  void end();
protected:
  virtual void exec() {};    // main

public:
  MinimumSolver(Problem* _P);
  virtual ~MinimumSolver() {};

  // getter
  Plan getSolution() const { return solution; };
  bool succeed() const { return solved; };
  std::string getSolverName() const { return solver_name; };
  int getMaxTimestep() const { return max_timestep; };
  int getCompTime() const { return comp_time; }
  int getSolverElapsedTime() const;  // get elapsed time from start
};

// -----------------------------------------------
// base class with utilities
// -----------------------------------------------
class Solver : public MinimumSolver
{
private:
  // useful info
  bool verbose;      // true -> print additional info
  int LB_soc;        // lower bound of soc
  int LB_makespan;   // lower bound of makespan

  // distance to goal
protected:
  using DistanceTable = std::vector<std::vector<int>>;  // [agent][node_id]
  DistanceTable distance_table;     // distance table
  DistanceTable* distance_table_p;  // pointer, used in nested solvers


  // -------------------------------
  // main
private:
  void exec();
protected:
  virtual void run() {} // main

  // -------------------------------
  // utilities for time
public:
  int getRemainedTime() const;       // get remained time
  bool overCompTime() const;         // check time limit

  // -------------------------------
  // utilities for problem instance
public:
  int getLowerBoundSOC();       // get trivial lower bound of sum-of-costs
  int getLowerBoundMakespan();  // get trivial lower bound of makespan
private:
  void computeLowerBounds();    // compute lb_soc and lb_makespan

  // -------------------------------
  // utilities for solution representation
public:
  static Paths planToPaths(const Plan& plan);   // plan -> paths
  static Plan pathsToPlan(const Paths& paths);  // paths -> plan

  // -------------------------------
  // utilities for debug
protected:
  // print debug info (only when verbose=true)
  void info() const;
  template <class Head, class... Tail> void info(Head&& head, Tail&&... tail) const
  {
    if (!verbose) return;
    std::cout << head << " ";
    info(std::forward<Tail>(tail)...);
  }
  void halt(const std::string& msg) const;  // halt program
  void warn(const std::string& msg) const;  // just printing msg

  // -------------------------------
  // log
public:
  virtual void makeLog(const std::string& logfile = "./result.txt");
protected:
  void makeLogBasicInfo(std::ofstream& log);
  void makeLogSolution(std::ofstream& log);

  // -------------------------------
  // utilities for solver options
public:
  virtual void setParams(int argc, char* argv[]){};
  void setVerbose(bool _verbose) { verbose = _verbose; }
protected:
  // used for set underlying solver options
  static void setSolverOption(std::shared_ptr<Solver> solver,
                              const std::vector<std::string>& option);

  // -------------------------------
  // print
public:
  void printResult();
protected:
  static void printHelpWithoutOption(const std::string& solver_name);

  // -------------------------------
  // utilities for distance
public:
  int pathDist(const int i, Node* const s) const;  // get path distance between s -> g_i
  int pathDist(const int i) const;                 // get path distance between s_i -> g_i
  void createDistanceTable();                      // compute distance table
  void setDistanceTable(DistanceTable* p) { distance_table_p = p; }  // used in nested solvers
  // use grid-pathfinding
  int pathDist(Node* const s, Node* const g) const { return G->pathDist(s, g); }


  // -------------------------------
  // utilities for getting path
public:
  // use grid-pathfinding
  Path getPath(Node* const s, Node* const g, bool cache=false) const { return G->getPath(s, g, cache); }

  // space-time A*
  struct AstarNode {
    Node* v;           // location
    int g;             // time
    int f;             // f-value
    AstarNode* p;      // parent
    std::string name;  // name
    AstarNode(Node* _v, int _g, int _f, AstarNode* _p);
    static std::string getName(Node* _v, int _g);
  };
  using CompareAstarNode = std::function<bool(AstarNode*, AstarNode*)>;
  using CheckAstarFin = std::function<bool(AstarNode*)>;
  using CheckInvalidAstarNode = std::function<bool(AstarNode*)>;
  using AstarHeuristics = std::function<int(AstarNode*)>;
  using AstarNodes = std::vector<AstarNode*>;
  /*
   * Template of Space-Time A*.
   * See the following reference.
   *
   * Cooperative Pathﬁnding.
   * D. Silver.
   * AI Game Programming Wisdom 3, pages 99–111, 2006.
   */
  static Path getPathBySpaceTimeAstar
  (Node* const s,                                 // start
   Node* const g,                                 // goal
   AstarHeuristics& fValue,                       // func: f-value
   CompareAstarNode& compare,                     // func: compare two nodes
   CheckAstarFin& checkAstarFin,                  // func: check goal
   CheckInvalidAstarNode& checkInvalidAstarNode,  // func: check invalid nodes
   const int time_limit=-1                        // time limit
   );
  // typical functions
  static CompareAstarNode compareAstarNodeBasic;
  // prioritized planning
  Path getPrioritizedPath(
      const int id,                // agent id
      const Paths& paths,          // already reserved paths
      const int time_limit = -1,   // time limit
      const int upper_bound = -1,  // upper bound of timesteps
      const std::vector<std::tuple<Node*, int>>& constraints =
          {},  // additional constraints, space-time
      CompareAstarNode& compare = compareAstarNodeBasic,  // compare two nodes
      const bool manage_path_table =
          true  // manage path table automatically, conflict check
  );
protected:
  // used for checking conflicts
  void updatePathTable(const Paths& paths, const int id);
  void clearPathTable(const Paths& paths);
  void updatePathTableWithoutClear(const int id, const Path& p, const Paths& paths);
  static constexpr int NIL = -1;
  std::vector<std::vector<int>> PATH_TABLE;

public:
  Solver(Problem* _P);
  virtual ~Solver();

  // other getter
  Problem* getP() { return P; }
};
