/*
 * Implementation of Improved Conflict-based Search (ICBS)
 *
 * - ref
 * Boyarski, E., Felner, A., Stern, R., Sharon, G., Tolpin, D., Betzalel, O., &
 * Shimony, E. (2015, June). ICBS: improved conflict-based search algorithm for
 * multi-agent pathfinding. In Twenty-Fourth International Joint Conference on
 * Artificial Intelligence.
 */

#pragma once
#include "cbs.hpp"
#include "solver.hpp"

class ICBS : public virtual CBS
{
public:
  static const std::string SOLVER_NAME;

protected:
  /*
   * Lazy invoke of high-level nodes
   * for situations where a new node increase at least
   * some amount of f-value
   */
  // all nodes in the lazy evaluation list have f-value beyond this value
  int LAZY_EVAL_LB_SOC;
  // lazy evaluation list
  std::unordered_map<int, HighLevelNodes> LAZY_EVAL_TABLE;
  // register one node on the lazy evaluation list
  void registerLazyEval(const int LB_SOC, HighLevelNode_p h_node);
  // re-evaluate
  CBS::HighLevelNodes lazyEval();

protected:
  // store MDD_c^i
  std::unordered_map<int, LibCBS::MDDs> MDDTable;

  virtual void setInitialHighLevelNode(HighLevelNode_p n);
  virtual Path getConstrainedPath(HighLevelNode_p h_node, int id);

  // prioritized conflict: { cardinal -> semi-cardinal -> non-cardinal }
  virtual LibCBS::Constraints getPrioritizedConflict(HighLevelNode_p h_node);
  // if possible, find another path following constraints
  bool findBypass(HighLevelNode_p h_node,
                  const LibCBS::Constraints& constraints);

  // main
  void run();

public:
  ICBS(Problem* _P);
  ~ICBS() {}

  static void printHelp();
};
