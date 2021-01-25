#include "../include/ir_focus_goals.hpp"

const std::string IR_FocusGoals::SOLVER_NAME = "IR_FOCUS_GOALS";

IR_FocusGoals::IR_FocusGoals(Problem* _P) : IR(_P)
{
  solver_name = IR_FocusGoals::SOLVER_NAME;
}

void IR_FocusGoals::refinePlan()
{
  updatePlanFocusOneAgent(updateByFocusGoals);
}

void IR_FocusGoals::printHelp()
{
  std::cout
    << IR_FocusGoals::SOLVER_NAME << "\n"
    << "  (no option)"
    << std::endl;
}
