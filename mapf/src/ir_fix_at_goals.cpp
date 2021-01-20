#include "../include/ir_fix_at_goals.hpp"

const std::string IR_FixAtGoals::SOLVER_NAME = "IR_FIX_AT_GOALS";

IR_FixAtGoals::IR_FixAtGoals(Problem* _P)
  : IR_FOCUS_ONE_AGENT(_P)
{
  solver_name = IR_FixAtGoals::SOLVER_NAME;
}

void IR_FixAtGoals::updatePlanFocusOneAgent(const int i, Plan& plan)
{
  plan = LibIR::refineTwoPathsAtGoal(i, plan, P, getRefineTimeLimit());
  updateSolution(plan);
}

void IR_FixAtGoals::printHelp()
{
  std::cout
    << IR_FixAtGoals::SOLVER_NAME << "\n"
    << "  (no option)"
    << std::endl;
}
