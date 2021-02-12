#include "../include/ir_focus_goals.hpp"

const std::string IR_FOCUS_GOALS::SOLVER_NAME = "IR_FOCUS_GOALS";

IR_FOCUS_GOALS::IR_FOCUS_GOALS(Problem* _P) : IR(_P)
{
  solver_name = IR_FOCUS_GOALS::SOLVER_NAME;
}

void IR_FOCUS_GOALS::refinePlan()
{
  updatePlanFocusOneAgent(updateByFocusGoals);
}

void IR_FOCUS_GOALS::printHelp()
{
  std::cout
    << IR_FOCUS_GOALS::SOLVER_NAME << "\n"
    << "  (no option)"
    << std::endl;
}
