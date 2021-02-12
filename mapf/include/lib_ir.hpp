#pragma once
#include <set>
#include <stack>

#include "lib_cbs.hpp"
#include "lib_solver.hpp"

namespace LibIR
{
  [[maybe_unused]] static std::vector<int> identifyInteractingSetByMDD(
      const int i, const Plan& plan, Solver* solver,
      bool whole_duration = false, const int time_limit = -1,
      std::mt19937* MT = nullptr)
  {
    auto t_start = Time::now();

    // basic info
    const int cost = plan.getPathCost(i);
    const int dist = solver->pathDist(i);

    // filtering
    if (cost == dist) return {};

    std::vector<int> agents(plan.get(0).size());
    std::iota(agents.begin(), agents.end(), 0);
    if (MT != nullptr) std::shuffle(agents.begin(), agents.end(), *MT);

    // modification set
    std::set<int> modif_set;

    for (int t = dist; t < cost; ++t) {
      // make mdd with small cost
      auto mdd = LibCBS::MDD(t, i, solver);
      if (time_limit == -1) {
        mdd.build();
      } else {
        int _t_limit = time_limit - getElapsedTime(t_start);
        if (_t_limit < 0) return {};
        mdd.build(_t_limit);
      }

      // create modif list
      for (auto j : agents) {
        if (i == j) continue;
        if (mdd.forceUpdate(LibCBS::getConstraintsByFixedPaths(plan, {j})) ||
            !mdd.valid) {
          modif_set.insert(j);
        }
        if (!mdd.valid) break;
      }

      if (!whole_duration) break;
    }

    if (modif_set.empty()) return {};

    modif_set.insert(i);
    std::vector<int> moidf_list(modif_set.begin(), modif_set.end());
    return moidf_list;
  }

  [[maybe_unused]] static std::vector<int> identifyAgentsAtGoal(
      const int i, const Plan& plan, Node* g, const int dist)
  {
    const int cost = plan.getPathCost(i);
    const int num = plan.get(0).size();
    if (cost == dist) return {};

    std::set<int> modif_set = {i};
    for (int t = cost - 1; t >= dist; --t) {
      for (int j = 0; j < num; ++j) {
        if (j == i) continue;
        if (plan.get(t, j) == g) modif_set.insert(j);
      }
    }
    std::vector<int> moidf_list(modif_set.begin(), modif_set.end());
    return moidf_list;
  }

  [[maybe_unused]] static std::tuple<int, std::vector<int>>
  identifyBottleneckAgentsWithScore(const int i, const Plan& original_plan,
                                    Solver* solver, const int time_limit = -1)
  {
    int score = 0;
    std::vector<int> modif_list;
    auto paths = planToPaths(original_plan);
    paths.clear(i);

    const int num = paths.size();

    for (int j = 0; j < num; ++j) {
      if (i == j) continue;
      const int dist = solver->pathDist(j);
      const int original_cost = paths.costOfPath(j);
      if (original_cost == dist) continue;
      const auto path = solver->getPrioritizedPath(j, paths, time_limit);
      if (path.empty()) {
        modif_list.clear();
        return std::make_tuple(0, modif_list);
      }
      const int cost = getPathCost(path);
      if (cost < original_cost) {
        score += original_cost - cost;
        modif_list.push_back(j);
      }
    }

    if (!modif_list.empty()) modif_list.push_back(i);
    return std::make_tuple(score, modif_list);
  }
};  // namespace LibIR
