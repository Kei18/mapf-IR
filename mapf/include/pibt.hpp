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

class PIBT : public Solver {
public:
  static const std::string SOLVER_NAME;

private:
  // PIBT agent
  struct Agent {
    int id;
    Node* v_now;   // current location
    Node* v_next;  // next location
    Node* g;       // goal
    int elapsed;   // eta
    int init_d;    // initial distance
    float tie_breaker;  // epsilon, tie-breaker
  };

  // option
  bool disable_dist_init = false;

  // result of priority inheritance: true -> valid, false -> invalid
  bool funcPIBT(Agent* ai,
                std::unordered_map<int, Agent*>& occupied_now,
                std::unordered_map<int, Agent*>& occupied_next);
  // plan next node
  Node* planOneStep(Agent* a,
                    std::unordered_map<int, Agent*>& occupied_now,
                    std::unordered_map<int, Agent*>& occupied_next);
  // chose one node from candidates, used in planOneStep
  Node* chooseNode(Agent* a,
                   std::unordered_map<int, Agent*>& occupied_now,
                   std::unordered_map<int, Agent*>& occupied_next);

  // main
  void run();

public:
  PIBT(Problem* _P);
  ~PIBT() {}

  void setParams(int argc, char *argv[]);
  static void printHelp();
};
