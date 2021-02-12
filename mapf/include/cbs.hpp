/*
 * Implementation of Conflict-based Search (CBS)
 *
 * - ref
 * Sharon, G., Stern, R., Felner, A., & Sturtevant, N. R. (2015).
 * Conflict-based search for optimal multi-agent pathfinding.
 * Artificial Intelligence, 219, 40–66.
 */

#pragma once
#include "lib_cbs.hpp"
#include "solver.hpp"

class CBS : public Solver
{
public:
  static const std::string SOLVER_NAME;

protected:
  // for high-level search
  struct HighLevelNode {
    int id;  // id
    Paths paths;
    LibCBS::Constraints constraints;
    int makespan;  // makespan
    int soc;       // sum of cost
    int f;         // for tie-break
    bool valid;

    HighLevelNode() {}
    HighLevelNode(int _id, Paths _paths, LibCBS::Constraints _c, int _m,
                  int _soc, int _f, bool _valid)
        : id(_id),
          paths(_paths),
          constraints(_c),
          makespan(_m),
          soc(_soc),
          f(_f),
          valid(_valid)
    {
    }
  };
  using HighLevelNode_p = std::shared_ptr<HighLevelNode>;
  using HighLevelNodes = std::vector<HighLevelNode_p>;
  using CompareHighLevelNodes =
      std::function<bool(HighLevelNode_p, HighLevelNode_p)>;

  // get single-agent path for the initial node
  Path getInitialPath(int id);

  // set initial high-level node
  virtual void setInitialHighLevelNode(HighLevelNode_p n);

  // find new paths subject to constraints
  virtual void invoke(HighLevelNode_p h_node, int id);

  // get single-agent path subject to constraints
  virtual Path getConstrainedPath(HighLevelNode_p h_node, int id);

  // objective function: sum of cost
  virtual CompareHighLevelNodes getObjective();

  // main
  virtual void run();

public:
  CBS(Problem* _P);
  virtual ~CBS(){};

  static void printHelp();
};
