#include "../include/ir_hybrid.hpp"

const std::string IR_HYBRID::SOLVER_NAME = "IR_HYBRID";

IR_HYBRID::IR_HYBRID(Problem* _P) : IR(_P)
{
  solver_name = IR_HYBRID::SOLVER_NAME;
}

void IR_HYBRID::refinePlan()
{
  info("", "update by FIX_AT_GOALS");
  updatePlanFocusOneAgent(updateByFixAtGoals);
  info("", "update by FOCUS_GOALS");
  updatePlanFocusOneAgent(updateByFocusGoals);
  info("", "update by MDD");
  updatePlanFocusOneAgent(updateByMDD);
  info("", "update by RANDOM");
  updateByRandom();
}

void IR_HYBRID::printHelp()
{
  std::cout
    << IR_HYBRID::SOLVER_NAME << "\n"
    << "  (no option)"
    << std::endl;
}
