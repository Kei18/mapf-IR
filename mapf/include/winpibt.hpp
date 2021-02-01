/*
 * Implementation of Windowed Priority Inheritance with Backtracking (winPIBT)
 *
 * - ref
 * Okumura, K., Tamura, Y. & Défago, X. (2020).
 * winPIBT: Extended Prioritized Algorithm for Iterative Multi-agent Path Finding
 * IJCAI Workshop on Multi-Agent Path Finidng (WoMAPF)
 */

#pragma once
#include "solver.hpp"

class winPIBT : public Solver
{
public:
  static const std::string SOLVER_NAME;

private:
  static const int NIL = -1;
  std::vector<int> occupied_t;  // node-id -> timestep
  std::vector<int> occupied_a;  // node-id -> agent

  // main
  void run();

  Path getSinglePath(const int id, std::vector<Path>& paths);
  bool funcPIBT(const int id, std::vector<Path>& paths,
                Node* v_other_to=nullptr, Node* v_other_from=nullptr);
  Node* chooseNode(const int id, std::vector<Path>& paths,
                   Node* v_other_to=nullptr, Node* v_other_from=nullptr);
  void updateLoc(const int id, const int t, Node* v_now, Node* v_next, std::vector<Path>& paths);

public:
  winPIBT(Problem* _P);
  ~winPIBT() {}

  void setParams(int argc, char* argv[]);
  static void printHelp();
};
