/*
 * Implementation of Push & Swap
 *
 * - ref
 * Luna, R., & Bekris, K. E. (2011, July).
 * Push and swap: Fast cooperative path-finding with completeness guarantees.
 * In IJCAI (pp. 294-300).
 */

#pragma once
#include "solver.hpp"

class PushAndSwap : public Solver
{
public:
  static const std::string SOLVER_NAME;

private:
  bool flg_compress;
  bool disable_dist_init;

  const int NIL = -1;
  Nodes nodes_with_many_neighbors;

  // main
  void run();
  bool push(Plan& plan, const int i, Nodes& U, std::vector<int>& occupied_now);
  bool swap(Plan& plan, const int i, Nodes& U, std::vector<int>& occupied_now);

  // improve solution quality
  Plan compress(const Plan& plan);

  // sub procedures
  bool multiPush(Plan& plan, const int r, const int s, const Path& p,
                 std::vector<int>& occupied_now);
  bool clear(Plan& plan, Node* v, const int r, const int s,
             std::vector<int>& occupied_now);
  void executeSwap(Plan& plan, const int r, const int s,
                   std::vector<int>& occupied_now);
  bool resolve(Plan& plan, const int r, const int s, Nodes& U,
               std::vector<int>& occupied_now);

  // util
  Node* getNearestEmptyNode(Node* v, std::vector<int>& occupied_now,
                            const Nodes& obs);
  Path getShortestPath(const int id, Node* s, std::vector<int>& occupied_now);
  bool pushTowardEmptyNode(Node* v, Plan& plan, std::vector<int>& occupied_now,
                           const Nodes& obs);
  void updatePlan(const int id, Node* next_node, Plan& plan,
                  std::vector<int>& occupied_now);
  void computeNodesWithManyNeighbors();
  void checkConsistency(Plan& plan, std::vector<int>& occupied_now);

public:
  PushAndSwap(Problem* _P);
  ~PushAndSwap() {}

  void setParams(int argc, char* argv[]);
  static void printHelp();
};
