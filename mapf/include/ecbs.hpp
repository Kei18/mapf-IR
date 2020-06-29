/*
 * Implementation of Enhanced Conflict-based Search (ECBS)
 *
 * - ref
 * Barer, M., Sharon, G., Stern, R., & Felner, A. (2014).
 * Suboptimal Variants of the Conflict-Based Search Algorithm for the Multi-Agent Pathfinding Problem.
 * In Seventh Annual Symposium on Combinatorial Search.
 */

#pragma once
#include "solver.hpp"
#include "lib_cbs.hpp"
#include <tuple>
#include <memory>


class ECBS : public Solver {
public:
  static const std::string SOLVER_NAME;

protected:
  struct HighLevelNode {
    Paths paths;
    LibCBS::Constraints constraints;
    int makespan;
    int soc;
    int f;
    int LB;  // lower bound
    std::vector<int> f_mins;
    bool valid;
  };
  using HighLevelNode_p = std::shared_ptr<HighLevelNode>;
  using CompareHighLevelNode = std::function<bool(HighLevelNode_p,
                                                  HighLevelNode_p)>;

  struct FocalNode {
    Node* v;
    int g;  // in getTimedPath, g represents t
    int f1;
    int f2;
    FocalNode* p;  // parent
  };
  using CompareFocalNode = std::function<bool(FocalNode*, FocalNode*)>;
  using CheckFocalFin = std::function<bool(FocalNode*)>;
  using CheckInvalidFocalNode = std::function<bool(FocalNode*)>;
  using FocalHeuristics = std::function<int(FocalNode*)>;

  float sub_optimality;
  static const float DEFAULT_SUB_OPTIMALITY;
  void setInitialHighLevelNode(HighLevelNode_p n);
  Path getInitialPath(int id);

  CompareHighLevelNode getMainObjective();
  CompareHighLevelNode getFocalObjective();
  void invoke(HighLevelNode_p h_node, int id);
  std::tuple<Path, int> getFocalPath(HighLevelNode_p h_node, int id);
  std::tuple<Path, int> getTimedPathByFocalSearch
  (Node* const s, Node* const g, float w,  // sub-optimality
   FocalHeuristics& f1Value,
   FocalHeuristics& f2Value,
   CompareFocalNode& compareOPEN,
   CompareFocalNode& compareFOCAL,
   CheckFocalFin& checkFocalFin,
   CheckInvalidFocalNode& checkInvalidFocalNode);

  Path getPathFromFocalNode(FocalNode* _n);


public:
  ECBS(Problem* _P);
  ~ECBS() {};

  void solve();

  void setParams(int argc, char *argv[]);
  static void printHelp();
};
