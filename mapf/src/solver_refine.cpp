#include "../include/solver_refine.hpp"


SolverRefine::SolverRefine(const Plan& _old_plan,
                           const std::vector<int>& _sample)
  : old_plan(_old_plan),
    old_paths(planToPaths(_old_plan)),
    ub_makespan(_old_plan.getMakespan()),
    ub_soc(_old_plan.getSOC()),
    sample(_sample)
{
  if (!sample.empty()) {
    if (_old_plan.empty()) return;
    for (int i = 0; i < _old_plan.get(0).size(); ++i) {
      if (!inArray(i, sample)) fixed_agents.push_back(i);
    }
  }
}
