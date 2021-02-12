/*
 * Implementation of Revisit Prioritized Planning
 *
 * - ref
 * Čáp, M., Novák, P., Kleiner, A., & Selecký, M. (2015).
 * Prioritized planning algorithms for trajectory coordination of multiple
 * mobile robots. IEEE transactions on automation science and engineering,
 * 12(3), 835-849.
 *
 */

#pragma once
#include "solver.hpp"

class RevisitPP : public Solver
{
public:
  static const std::string SOLVER_NAME;

private:
  bool disable_dist_init = false;  // option

  // get one agent path
  Path getPrioritizedPath(
      int id, const Paths& paths,
      const std::vector<std::tuple<Node*, int>> constraints);

  void run();

public:
  RevisitPP(Problem* _P);
  ~RevisitPP(){};

  void setParams(int argc, char* argv[]);
  static void printHelp();
};
