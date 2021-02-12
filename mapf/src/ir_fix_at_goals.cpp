#include "../include/ir_fix_at_goals.hpp"

const std::string IR_FIX_AT_GOALS::SOLVER_NAME = "IR_FIX_AT_GOALS";

IR_FIX_AT_GOALS::IR_FIX_AT_GOALS(Problem* _P) : IR(_P)
{
  solver_name = IR_FIX_AT_GOALS::SOLVER_NAME;
}

void IR_FIX_AT_GOALS::refinePlan()
{
  updatePlanFocusOneAgent(updateByFixAtGoals);
}

void IR_FIX_AT_GOALS::printHelp()
{
  std::cout << IR_FIX_AT_GOALS::SOLVER_NAME << "\n"
            << "  (no option)" << std::endl;
}
