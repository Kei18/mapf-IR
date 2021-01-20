#include "../include/ir_focus_one_agent.hpp"

IR_FOCUS_ONE_AGENT::IR_FOCUS_ONE_AGENT(Problem* _P)
  : IR(_P)
{
}

void IR_FOCUS_ONE_AGENT::refinePlan()
{
  Plan plan = solution;
  int last_itr_soc = plan.getSOC();

  // fix at goal
  do {
    last_itr_soc = plan.getSOC();
    for (int i = 0; i < P->getNum(); ++i) {
      if (overCompTime() || current_iteration >= max_iteration) break;
      if (plan.getPathCost(i) - pathDist(i) == 0) continue;
      updatePlanFocusOneAgent(i, plan);
    }
  } while (last_itr_soc != plan.getSOC() && !overCompTime() && current_iteration < max_iteration);
}
