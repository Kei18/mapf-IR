#include "../include/ir_focus_goals.hpp"

const std::string IR_FocusGoals::SOLVER_NAME = "IR_FOCUS_GOALS";

IR_FocusGoals::IR_FocusGoals(Problem* _P)
  : IR_FOCUS_ONE_AGENT(_P)
{
  solver_name = IR_FocusGoals::SOLVER_NAME;
}

void IR_FocusGoals::updatePlanFocusOneAgent(const int i, Plan& plan)
{
  const auto modif_list = LibIR::identifyAgentsAtGoal(i, plan, P->getGoal(i), pathDist(i));
  if (modif_list.empty()) return;
  Problem _P = Problem(P, getRefineTimeLimit());
  plan = std::get<1>(getOptimalPlan(&_P, plan, modif_list));
  updateSolution(plan);
}

void IR_FocusGoals::printHelp()
{
  std::cout
    << IR_FocusGoals::SOLVER_NAME << "\n"
    << "  (no option)"
    << std::endl;
}
