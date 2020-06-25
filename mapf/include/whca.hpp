/*
 * Implementation of Hierarchical Cooperative A* (HCA*)
 *
 * - ref
 * Silver, D. (2005).
 * Cooperative pathfinding.
 * In AIIDE’05 Proceedings of the First AAAI Conference on Artificial Intelligence and Interactive Digital Entertainment (pp. 117–122).
 *
 */

#pragma once
#include "solver.hpp"

class WHCA : public Solver {
public:
  static const std::string SOLVER_NAME;

private:
  int window;
  static const int DEFAULT_WINDOW;

  // option
  bool disable_dist_init = false;

  Path getPrioritizedPartialPath(int id, Node* s, Node* g,
                                 const Paths& paths);

public:
  WHCA(Problem* _P);
  ~WHCA() {};

  void solve();

  void setParams(int argc, char *argv[]);
  static void printHelp();
};
