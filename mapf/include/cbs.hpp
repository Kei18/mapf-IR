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

class CBS : public Solver {
public:
  static const std::string SOLVER_NAME;

protected:
  struct Constraint {
    int id;   // agent id
    int t;
    Node* v;  // at t
    Node* u;  // used only for swap conflict, at t-1
  };
  using Constraints = std::vector<Constraint*>;

  struct HighLevelNode {
    Paths paths;
    Constraints constraints;
    int makespan;
    int soc;
    int f;   // for tie-break
    bool valid;
  };
  using CompareHighLevelNodes = std::function<bool(HighLevelNode*,
                                                   HighLevelNode*)>;

  void setInitialHighLevelNode(HighLevelNode* n);
  Path getInitialPath(int id);
  Constraints getFirstConflict(const Paths& paths);
  void invoke(HighLevelNode* h_node, int id);
  int countConflict(const Paths& paths);
  int countConflict(int id, const Path& path, const Paths& _paths);
  virtual Path getConstrainedPath(HighLevelNode* h_node, int id);
  virtual CompareHighLevelNodes getObjective();

public:
  CBS(Problem* _P);
  ~CBS() {};

  void solve();
  static void printHelp();
};
