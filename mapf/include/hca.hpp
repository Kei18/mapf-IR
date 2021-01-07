/*
 * Implementation of Hierarchical Cooperative A* (HCA*)
 *
 * - ref
 * Silver, D. (2005).
 * Cooperative pathfinding.
 * In AIIDE’05 Proceedings of the First AAAI Conference on Artificial
 * Intelligence and Interactive Digital Entertainment (pp. 117–122).
 *
 * Initial priorities are determined such that far agents have high priorities.
 * If you dislike it, use -d [--disable-dist-init] option.
 * - ref
 * Berg, J. P. van den, & Overmars, M. H. (2005).
 * Prioritized motion planning for multiple robots.
 * In 2005 IEEE/RSJ International Conference on Intelligent Robots and Systems
 * (pp. 430–435).
 *
 */

#pragma once
#include "solver.hpp"

class HCA : public Solver
{
public:
  static const std::string SOLVER_NAME;

private:
  bool disable_dist_init = false;  // option

  // get one agent path
  Path getPrioritizedPath(int id, Node* s, Node* g, const Paths& paths);
  Path getPrioritizedPath(int id, const Paths& paths);

  void run();

public:
  HCA(Problem* _P);
  ~HCA(){};

  void setParams(int argc, char* argv[]);
  static void printHelp();
};
