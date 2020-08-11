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
  // high-level node, see CBS for details
  struct HighLevelNode {
    Paths paths;
    LibCBS::Constraints constraints;
    int makespan;
    int soc;
    int f;
    int LB;                    // lower bound
    std::vector<int> f_mins;   // f_mins value in the low-level search
    bool valid;

    HighLevelNode() {}
    HighLevelNode(Paths _paths, LibCBS::Constraints _c,
                  int _m, int _soc, int _f,
                  int _LB, std::vector<int> _f_mins,
                  bool _valid)
      : paths(_paths), constraints(_c),
        makespan(_m), soc(_soc), f(_f),
        LB(_LB), f_mins(_f_mins), valid(_valid) {}
  };
  using HighLevelNode_p = std::shared_ptr<HighLevelNode>;
  using CompareHighLevelNode = std::function<bool(HighLevelNode_p,
                                                  HighLevelNode_p)>;

  // used in the low-level search
  struct FocalNode {
    Node* v;       // location
    int g;         // in getTimedPath, g represents t
    int f1;        // used in open list
    int f2;        // used in focal list
    FocalNode* p;  // parent
  };
  using CompareFocalNode = std::function<bool(FocalNode*, FocalNode*)>;
  using CheckFocalFin = std::function<bool(FocalNode*)>;
  using CheckInvalidFocalNode = std::function<bool(FocalNode*)>;
  using FocalHeuristics = std::function<int(FocalNode*)>;

  // sub optimality
  float sub_optimality;
  static const float DEFAULT_SUB_OPTIMALITY;

  void setInitialHighLevelNode(HighLevelNode_p n);
  Path getInitialPath(int id);

  // objective for open list
  CompareHighLevelNode getMainObjective();
  // objective for focal list
  CompareHighLevelNode getFocalObjective();

  void invoke(HighLevelNode_p h_node, int id);

  // return path and f-min value
  std::tuple<Path, int> getFocalPath(HighLevelNode_p h_node, int id);
  std::tuple<Path, int> getTimedPathByFocalSearch
  (Node* const s, Node* const g, float w,  // sub-optimality
   FocalHeuristics& f1Value,
   FocalHeuristics& f2Value,
   CompareFocalNode& compareOPEN,
   CompareFocalNode& compareFOCAL,
   CheckFocalFin& checkFocalFin,
   CheckInvalidFocalNode& checkInvalidFocalNode);

  // make path from focal node
  Path getPathFromFocalNode(FocalNode* _n);

  // main
  void run();

public:
  ECBS(Problem* _P);
  ~ECBS() {};

  void setParams(int argc, char *argv[]);
  static void printHelp();
};
