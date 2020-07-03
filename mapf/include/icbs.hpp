/*
 * Implementation of Improved Conflict-based Search (ICBS)
 *
 * - ref
 * Boyarski, E., Felner, A., Stern, R., Sharon, G., Tolpin, D., Betzalel, O., & Shimony, E. (2015, June).
 * ICBS: improved conflict-based search algorithm for multi-agent pathfinding.
 * In Twenty-Fourth International Joint Conference on Artificial Intelligence.
 *
 * Note:
 * I initially tried to use smart pointer to manage MDDNode,
 * however, I figured out the methods took too much time.
 */

#pragma once
#include "solver.hpp"
#include "cbs.hpp"


class ICBS : public virtual CBS {
public:
  static const std::string SOLVER_NAME;

protected:
  int LAZY_EVAL_LB_SOC;
  std::unordered_map<int, HighLevelNodes> LAZY_EVAL_TABLE;
  void registerLazyEval(const int LB_SOC, HighLevelNode_p h_node);
  CBS::HighLevelNodes lazyEval();

protected:
  std::unordered_map<int, LibCBS::MDDs> MDDTable;  // store MDD_c^i

  virtual void setInitialHighLevelNode(HighLevelNode_p n);
  virtual Path getConstrainedPath(HighLevelNode_p h_node, int id);
  virtual LibCBS::Constraints getPrioritizedConflict(HighLevelNode_p h_node);
  bool findBypass(HighLevelNode_p h_node,
                  const LibCBS::Constraints& constraints);
  void run();

public:
  ICBS(Problem* _P);
  ~ICBS();

  static void printHelp();
};
