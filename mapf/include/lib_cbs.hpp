/*
 * This mainly contains utilities of CBS-based solvers, e.g., MDD.
 */

#pragma once
#include <memory>

#include "solver.hpp"

namespace LibCBS
{
  // ======================================
  // conflict
  struct Constraint;
  using Constraint_p = std::shared_ptr<Constraint>;
  using Constraints = std::vector<Constraint_p>;

  // ======================================
  // MDD
  struct MDDNode;
  using MDDNodes = std::vector<MDDNode*>;
  struct MDD;
  using MDD_p = std::shared_ptr<MDD>;
  using MDDs = std::vector<MDD_p>;

  // ======================================
  // conflict
  struct Constraint {
    int id;     // agent id, -1 -> for all agetnts
    int t;      // at t
    Node* v;    // at t
    Node* u;    // used only for swap conflict, at t-1
    bool stay;  // stay forever from t

    Constraint(int _id, int _t, Node* _v, Node* _u)
        : id(_id), t(_t), v(_v), u(_u), stay(false){};
    Constraint(int _id, int _t, Node* _v, bool _stay)
        : id(_id), t(_t), v(_v), u(nullptr), stay(_stay){};
    void println();
  };

  // for CBS-style solvers
  Constraints getFirstConstraints(const Paths& paths);

  // for ICBS
  void getPrioritizedConflict(const int t, const int i, const int j,
                              const Paths& paths, const MDDs& mdds,
                              Constraints& cardinal_conflicts,
                              Constraints& semi_cardinal_constraints,
                              Constraints& non_cardinal_constraints);
  Constraints getPrioritizedConflict(const Paths& paths, const MDDs& mdds);
  // for limited agents, used in refine-solvers
  Constraints getPrioritizedConflict(const Paths& paths, const MDDs& mdds,
                                     const std::vector<int>& sample);

  // used in refine-solvers
  // create constraints by fixed paths for CBS-style solvers
  Constraints getConstraintsByFixedPaths(const Plan& plan,
                                         const std::vector<int>& fixed_agents);

  // ======================================
  // MDD
  struct MDDNode {
    int t;          // timestep
    Node* v;        // location
    MDDNodes next;  // available nodes at t+1
    MDDNodes prev;  // available nodes at t-1

    MDDNode(int _t, Node* _v) : t(_t), v(_v) {}
    ~MDDNode() {}
    bool operator==(const MDDNode& other) const;
  };

  struct MDD {
    int c;                       // cost
    int i;                       // agent
    Graph* G;                    // original graph
    Node* s;                     // start
    Node* g;                     // goal;
    std::vector<MDDNodes> body;  // t: 0...c
    bool valid;                  // false -> no path from s to g
    MDDNodes GC;                 // for memory management
    Solver* solver;  // solver

    // cache, MDD without any constraints
    static std::unordered_map<std::string, MDD_p> PURE_MDD_TABLE;

    // used in finding paths
    static std::mt19937* MT;

    MDD(int _c, int _i, Graph* _G, Node* _s, Node* _g, bool _valid);
    // time_limit: for situations taking long time to construct one MDD
    MDD(int _c, int _i, Problem* P, Constraints constraints,
        int time_limit = -1);
    MDD(int _c, int _i, Problem* P);
    MDD(int _c, int _i, Solver* solver);
    MDD(int _c, int _i, Problem* P, Solver* _solver, Constraints constraints,
        int time_limit = -1);
    ~MDD();

    MDD(const MDD& other);  // copy
    void copy(const MDD& other);

    // used
    MDDNode* createNewNode(int t, Node* v);

    // create new MDD
    void build(int time_limit = -1);

    // update MDD with new constraints
    void update(const Constraints& _constraints);
    bool forceUpdate(const Constraints& _constraints);

    // sub-procedures used in update
    void deleteForward(MDDNode* node);
    void deleteBackword(MDDNode* node);

    // get one path from MDD
    Path getPath() const;
    Path getPath(Constraint_p const constraint) const;
    Path getPath(const Constraints& _constraints) const;

    // get MDD width at the timestep
    int getWidth(int t) const;

    // print info
    void println() const;

    // used for cache
    std::string getPureMDDName();
  };
};  // namespace LibCBS
