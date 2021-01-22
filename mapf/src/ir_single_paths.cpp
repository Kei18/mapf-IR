#include "../include/ir_single_paths.hpp"

const std::string IR_SinglePaths::SOLVER_NAME = "IR_SINGLE_PATHS";

IR_SinglePaths::IR_SinglePaths(Problem* _P)
  : IR_FOCUS_ONE_AGENT(_P)
{
  solver_name = IR_SinglePaths::SOLVER_NAME;
}

void IR_SinglePaths::updatePlanFocusOneAgent(const int i, Plan& plan)
{
  // filtering
  const int cost = plan.getPathCost(i);
  if (cost == pathDist(i)) return;

  // get new path
  auto paths = planToPaths(plan);
  const auto path = getPrioritizedPath(i, paths, getRefineTimeLimit(), max_timestep);
  if (path.empty() || getPathCost(path) >= cost) return;

  // update paths
  paths.insert(i, path);
  plan = pathsToPlan(paths);
  updateSolution(plan);
}

void IR_SinglePaths::printHelp()
{
  std::cout
    << IR_SinglePaths::SOLVER_NAME << "\n"
    << "  (no option)"
    << std::endl;
}
