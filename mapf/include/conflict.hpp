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

  static int countConflict(const Paths& paths)
  {
    int cnt = 0;
    int num_agents = paths.size();
    int makespan = paths.getMakespan();
    for (int i = 0; i < num_agents; ++i) {
      for (int j = i + 1; j < num_agents; ++j) {
        for (int t = 1; t < makespan; ++t) {
          // vertex conflict
          if (paths.get(i, t) == paths.get(j, t)) ++cnt;
          // swap conflict
          if (paths.get(i, t) == paths.get(j, t-1) &&
              paths.get(j, t) == paths.get(i, t-1)) ++cnt;
        }
      }
    }
    return cnt;
  }

  static int countConflict(int id,
                           const Path& path,
                           const Paths& paths)
  {
    int cnt = 0;
    int makespan = paths.getMakespan();
    int num_agents = paths.size();
    for (int i = 0; i < num_agents; ++i) {
      if (i == id) continue;
      for (int t = 1; t < path.size(); ++t) {
        if (t > makespan) {
          if (path[t] == paths.get(i, makespan)) {
            ++cnt;
            break;
          }
          continue;
        }
        // vertex conflict
        if (paths.get(i, t) == path[t]) ++cnt;
        // swap conflict
        if (paths.get(i, t) == path[t-1] &&
            paths.get(i, t-1) == path[t]) ++cnt;
      }
    }
    return cnt;
  }

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

  static Constraints getAllConstraints(const Paths& paths)
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
          }
        }
      }
    }
    return constraints;
  }
}
