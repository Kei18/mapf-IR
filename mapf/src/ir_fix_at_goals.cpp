#include "../include/ir_fix_at_goals.hpp"

const std::string IR_FixAtGoals::SOLVER_NAME = "IR_FIX_AT_GOALS";

IR_FixAtGoals::IR_FixAtGoals(Problem* _P) : IR(_P)
{
  solver_name = IR_FixAtGoals::SOLVER_NAME;
}

void IR_FixAtGoals::refinePlan()
{
  updatePlanFocusOneAgent(updateByFixAtGoals);
}

void IR_FixAtGoals::printHelp()
{
  std::cout
    << IR_FixAtGoals::SOLVER_NAME << "\n"
    << "  (no option)"
    << std::endl;
}
