#pragma once
#include "lib_solver.hpp"


namespace LibIR
{
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

  [[maybe_unused]]
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
      const Path path = paths.get(i);
      const int cost = getPathCost(path);
      const int dist = G->pathDist(s, g);

      // filtering
      if (cost <= dist + 1) continue;

      for (int t = cost - 1; t > dist; --t) {
        if (!(path[t] != g && path[t-1] == g && path[t+1] == g)) continue;

        for (int j = 0; j < num; ++j) {
          if (i == j) continue;
          if (paths.get(j, t) != g) continue;

          // create temporal paths
          auto tmp_path  = path;
          auto tmp_paths = paths;
          tmp_path.resize(t);
          tmp_paths.insert(i, tmp_path);

          // constraints
          std::tuple<Node*, int> constraint = std::make_tuple(g, t);

          // get refined plan for j
          const auto refined_path_j = getBasicPrioritizedPath
            (j, starts[j], goals[j], G, tmp_paths,
             time_limit - getElapsedTime(t_start), { constraint });
          if (refined_path_j.empty()) continue;
          tmp_paths.insert(j, refined_path_j);

          // check update or not
          const int original_costs = paths.costOfPath(i) + paths.costOfPath(j);
          const int tmp_costs = tmp_paths.costOfPath(i) + tmp_paths.costOfPath(j);
          if (tmp_costs < original_costs) {
            paths = tmp_paths;
          } else {
            continue;
          }
        }
      }
    }

    return pathsToPlan(paths);
  }
};
