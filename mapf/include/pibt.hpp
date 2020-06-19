/*
 * Implementation of Priority Inheritance with Backtracking (PIBT)
 *
 * - ref
 * Okumura, K., Machida, M., Défago, X., & Tamura, Y. (2019).
 * Priority Inheritance with Backtracking for Iterative Multi-agent Path Finding.
 * In Proceedings of the Twenty-Eighth International Joint Conference on Artificial Intelligence (pp. 535–542).
 */

#pragma once
#include "solver.hpp"

struct PIBTAgent {
  int id;
  Node* v_now;
  Node* v_next;
  Node* g;
  int elapsed;
  int init_d;
  float tie_breaker;
};

class PIBT : public Solver {
public:
  static const std::string SOLVER_NAME;

private:
  // option
  bool disable_dist_init = false;

  bool funcPIBT(PIBTAgent* ai,
                std::unordered_map<int, PIBTAgent*>& occupied_now,
                std::unordered_map<int, PIBTAgent*>& occupied_next);
  Node* planOneStep(PIBTAgent* a,
                    std::unordered_map<int, PIBTAgent*>& occupied_now,
                    std::unordered_map<int, PIBTAgent*>& occupied_next);
  Node* chooseNode(PIBTAgent* a,
                   std::unordered_map<int, PIBTAgent*>& occupied_now,
                   std::unordered_map<int, PIBTAgent*>& occupied_next);

public:
  PIBT(Problem* _P);
  ~PIBT() {};

  void solve();

  void setParams(int argc, char *argv[]);
  static void printHelp();
};
