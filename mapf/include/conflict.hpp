#pragma once
#include "solver.hpp"

namespace Conflict {
  struct Constraint {
    int id;   // agent id, -1 -> for all agetnts
    int t;    // at t
    Node* v;  // at t
    Node* u;  // used only for swap conflict, at t-1

    void println() {
      if (u == nullptr) {
        std::cout << "vertex constraint: <id="
                  << id << ", t=" << t << ", v=" << v->id
                  << ">" << std::endl;
      } else {
        std::cout << "edge constraint: <id="
                  << id << ", t=" << t << ", v:"
                  << u->id << "->" << v->id
                  << ">" << std::endl;
      }
    }
  };
  using Constraints = std::vector<Constraint*>;

  static Constraints getFirstConstraints(const Paths& paths)
  {
    Constraints constraints = {};
    int num_agents = paths.size();
    int makespan = paths.getMakespan();
    for (int t = 1; t <= makespan; ++t) {
      for (int i = 0; i < num_agents; ++i) {
        for (int j = i + 1; j < num_agents; ++j) {
          // vertex conflict
          if (paths.get(i, t) == paths.get(j, t)) {
            constraints.push_back(new Constraint
                                  { i, t, paths.get(i, t), nullptr });
            constraints.push_back(new Constraint
                                  { j, t, paths.get(j, t), nullptr });
            return constraints;
          }
          // swap conflict
          if (paths.get(i, t) == paths.get(j, t-1) &&
              paths.get(j, t) == paths.get(i, t-1)) {
            constraints.push_back(new Constraint
                                  { i, t,
                                     paths.get(i, t),
                                     paths.get(i, t-1) });
            constraints.push_back(new Constraint
                                  { j, t,
                                     paths.get(j, t),
                                     paths.get(j, t-1) });
            return constraints;
          }
        }
      }
    }
    return constraints;
  }
}
