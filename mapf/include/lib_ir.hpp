#pragma once
#include "lib_solver.hpp"
#include "lib_cbs.hpp"
#include <set>
#include <stack>


namespace LibIR
{
  /*
   * refine example: x, y, y, z -> x, y, z
   */
  static Plan refineSinglePaths
  (const Plan& original_plan,
   Graph* G,
   const Config& starts,
   const Config& goals,
   const int time_limit)
  {
    const auto t_start = Time::now();
    const int num = starts.size();
    auto paths = planToPaths(original_plan);

    for (int i = 0; i < num; ++i) {
      const int cost = paths.costOfPath(i);

      // filtering
      if (cost == G->pathDist(starts[i], goals[i])) continue;

      // get new path
      const auto path = getBasicPrioritizedPath
        (i, starts[i], goals[i], G, paths, time_limit - getElapsedTime(t_start), {});
      if (path.empty()) continue;

      // update path
      if (getPathCost(path) < cost) paths.insert(i, path);
    }
    return pathsToPlan(paths);
  }

  [[maybe_unused]]
  static Plan refineSinglePaths(const Plan& original_plan, Problem* P, const int time_limit)
  {
    return refineSinglePaths
      (original_plan, P->getG(), P->getConfigStart(), P->getConfigGoal(), time_limit);
  }

  /*
   * refine example:
   * - x, y, z, y -> x, y, y, y
   * - *, v, z, * -> *, v, *, *
   */
  static Plan refineTwoPathsAtGoal
  (const Plan& original_plan,
   Graph* G,
   const Config& starts,
   const Config& goals,
   const int time_limit)
  {
    const auto t_start = Time::now();
    const int num = starts.size();
    auto paths = planToPaths(original_plan);

    for (int i = 0; i < num; ++i) {
      Node* s = starts[i];
      Node* g = goals[i];
      Path path = paths.get(i);
      const int cost = getPathCost(path);
      const int dist = G->pathDist(s, g);

      // filtering
      if (cost <= dist + 1) continue;

      bool stop_flg = false;
      for (int t = cost - 1; t > dist; --t) {
        if (path[t] == g) continue;
        if (path[t-1] != g || path[t+1] != g) break;

        for (int j = 0; j < num; ++j) {
          if (i == j) continue;
          if (paths.get(j, t) != g) continue;

          // create temporal paths
          auto tmp_path  = path;
          auto tmp_paths = paths;
          tmp_path.resize(t);
          tmp_paths.insert(i, tmp_path);

          const int original_costs = paths.costOfPath(i) + paths.costOfPath(j);
          const int upper_bound = original_costs - tmp_paths.costOfPath(i) - 1;

          // constraints
          std::tuple<Node*, int> constraint = std::make_tuple(g, t);

          // get refined plan for j
          const auto refined_path_j = getBasicPrioritizedPath
            (j, starts[j], goals[j], G, tmp_paths,
             time_limit - getElapsedTime(t_start), { constraint }, upper_bound);
          if (refined_path_j.empty()) {
            stop_flg = true;
            break;
          }
          tmp_paths.insert(j, refined_path_j);

          // check update or not
          paths = tmp_paths;
          path = paths.get(i);
          break;
        }
        if (stop_flg) break;
      }
    }

    return pathsToPlan(paths);
  }

  [[maybe_unused]]
  static Plan refineTwoPathsAtGoal(const Plan& original_plan, Problem* P, const int time_limit)
  {
    return refineTwoPathsAtGoal
      (original_plan, P->getG(), P->getConfigStart(), P->getConfigGoal(), time_limit);
  }

  static std::vector<int> identifyInteractingSetByMDD
  (const int i, const Plan& plan, Graph* G, Node* s, Node* g,
   bool whole_duration=false, const int time_limit=-1, std::mt19937* MT=nullptr)
  {
    auto t_start = Time::now();

    // basic info
    const int cost = plan.getPathCost(i);
    const int dist = G->pathDist(s, g);
    std::vector<int> agents(plan.get(0).size());
    std::iota(agents.begin(), agents.end(), 0);
    if (MT != nullptr) std::shuffle(agents.begin(), agents.end(), *MT);

    // filtering
    if (cost == dist) return {};

    // modification set
    std::set<int> modif_set;

    for (int t = dist; t < cost; ++t) {
      // make mdd with small cost
      auto mdd = LibCBS::MDD(t, i, G, s, g, true);
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
        if (mdd.forceUpdate(LibCBS::getConstraintsByFixedPaths(plan, { j })) || !mdd.valid) {
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

  [[maybe_unused]]
  static std::vector<int> identifyInteractingSetByMDD
  (const int i, const Plan& plan, Problem* P,
   const bool whole_duration=false, const int time_limit=-1, std::mt19937* MT=nullptr)
  {
    return identifyInteractingSetByMDD
      (i, plan, P->getG(), P->getStart(i), P->getGoal(i), whole_duration, time_limit, MT);
  }

  static std::vector<int> identifyInteractingSetByMDDAggressive
  (const int i, const Plan& plan, Graph* G, const Config& starts, const Config& goals,
   bool whole_duration=false, const int time_limit=-1, std::mt19937* MT=nullptr)
  {
    auto init_modif_list = identifyInteractingSetByMDD
      (i, plan, G, starts[i], goals[i], whole_duration, time_limit, MT);
    if (init_modif_list.empty()) return {};

    std::vector<int> modif_list;
    std::stack<int> OPEN;
    std::unordered_map<int, bool> CLOSE;

    for (auto j : init_modif_list) OPEN.push(j);

    while (!OPEN.empty()) {
      auto j = OPEN.top();
      OPEN.pop();
      if (CLOSE.find(j) != CLOSE.end()) continue;
      CLOSE[j] = true;
      modif_list.push_back(j);
      auto list = identifyInteractingSetByMDD
        (j, plan, G, starts[j], goals[j], whole_duration, time_limit, MT);
      for (auto k : list) {
        if (CLOSE.find(k) != CLOSE.end()) continue;
        OPEN.push(k);
      }
    }

    return modif_list;
  }

  [[maybe_unused]]
  static std::vector<int> identifyInteractingSetByMDDAggressive
  (const int i, const Plan& plan, Problem* P,
   const bool whole_duration=false, const int time_limit=-1, std::mt19937* MT=nullptr)
  {
    return identifyInteractingSetByMDDAggressive
      (i, plan, P->getG(), P->getConfigStart(), P->getConfigGoal(), whole_duration, time_limit, MT);
  }

  static std::vector<int> identifyAgentsAtGoal
  (const int i, const Plan& plan, Graph* G, Node* s, Node* g)
  {
    const int cost = plan.getPathCost(i);
    const int dist = G->pathDist(s, g);
    const int num  = plan.get(0).size();

    std::set<int> modif_set;
    for (int t = cost - 1; t >= dist; --t) {
      for (int j = 0; j < num; ++j) {
        if (j == i) continue;
        if (plan.get(t, j) == g) modif_set.insert(j);
      }
    }
    if (!modif_set.empty()) modif_set.insert(i);
    std::vector<int> moidf_list(modif_set.begin(), modif_set.end());
    return moidf_list;
  }

  [[maybe_unused]]
  static std::vector<int> identifyAgentsAtGoal
  (const int i, const Plan& plan, Problem* P)
  {
    return identifyAgentsAtGoal(i, plan, P->getG(), P->getStart(i), P->getGoal(i));
  }

  static std::tuple<int, std::vector<int>> identifyBottleneckAgentsWithScore
  (const int i, const Plan& original_plan, Graph* G, const Config& starts, const Config& goals)
  {
    int score = 0;
    std::vector<int> modif_list;
    auto paths = planToPaths(original_plan);
    paths.clear(i);

    const int num = paths.size();

    for (int j = 0; j < num; ++j) {
      if (i == j) continue;
      Node* s = starts[j];
      Node* g = goals[j];
      const int dist = G->pathDist(s, g);
      const int original_cost = paths.costOfPath(j);
      if (original_cost == dist) continue;
      const auto path = getBasicPrioritizedPath(j, s, g, G, paths);
      const int cost = getPathCost(path);
      if (cost < original_cost) {
        score += original_cost - cost;
        modif_list.push_back(j);
      }
    }

    if (!modif_list.empty()) modif_list.push_back(i);
    return std::make_tuple(score, modif_list);
  }

  [[maybe_unused]]
  static std::tuple<int, std::vector<int>> identifyBottleneckAgentsWithScore
  (const int i, const Plan& plan, Problem* P)
  {
    return identifyBottleneckAgentsWithScore
      (i, plan, P->getG(), P->getConfigStart(), P->getConfigGoal());
  }
};
