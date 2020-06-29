/*
 * Implementation of Conflict-based Search (CBS)
 *
 * - ref
 * Sharon, G., Stern, R., Felner, A., & Sturtevant, N. R. (2015).
 * Conflict-based search for optimal multi-agent pathfinding.
 * Artificial Intelligence, 219, 40–66.
 */

#pragma once
#include "solver.hpp"
#include "lib_cbs.hpp"

class CBS : public Solver {
public:
  static const std::string SOLVER_NAME;

protected:
  struct HighLevelNode {
    int id;  // high level node id
    Paths paths;
    LibCBS::Constraints constraints;
    int makespan;
    int soc;
    int f;   // for tie-break
    bool valid;
  };
  using HighLevelNode_p = std::shared_ptr<HighLevelNode>;
  using CompareHighLevelNodes = std::function<bool(HighLevelNode_p,
                                                   HighLevelNode_p)>;

  virtual void run();
  Path getInitialPath(int id);
  virtual void setInitialHighLevelNode(HighLevelNode_p n);
  virtual void invoke(HighLevelNode_p h_node, int id);
  virtual Path getConstrainedPath(HighLevelNode_p h_node, int id);
  virtual CompareHighLevelNodes getObjective();

public:
  CBS(Problem* _P);
  virtual ~CBS() {};

  static void printHelp();
};
