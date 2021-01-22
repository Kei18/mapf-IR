#include "../include/ir_fix_at_goals.hpp"

const std::string IR_FixAtGoals::SOLVER_NAME = "IR_FIX_AT_GOALS";

IR_FixAtGoals::IR_FixAtGoals(Problem* _P)
  : IR_FOCUS_ONE_AGENT(_P)
{
  solver_name = IR_FixAtGoals::SOLVER_NAME;
}

void IR_FixAtGoals::updatePlanFocusOneAgent(const int i, Plan& plan)
{
  const auto t_s = Time::now();

  Node* g = P->getGoal(i);
  const int cost = plan.getPathCost(i);
  const int dist = pathDist(i);
  if (cost <= dist + 1) return;

  auto paths = planToPaths(plan);
  Path path = paths.get(i);
  bool stop_flg = false;

  for (int t = cost - 1; t > dist; --t) {
    if (path[t] == g) continue;
    if (path[t-1] != g || path[t+1] != g) break;

    for (int j = 0; j < P->getNum(); ++j) {
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
      const auto refined_path_j = getPrioritizedPath
        (j, tmp_paths, getRefineTimeLimit() - getElapsedTime(t_s), upper_bound, { constraint });
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

  plan = pathsToPlan(paths);
  updateSolution(plan);
}

void IR_FixAtGoals::printHelp()
{
  std::cout
    << IR_FixAtGoals::SOLVER_NAME << "\n"
    << "  (no option)"
    << std::endl;
}
