#include "../include/ir_mdd.hpp"

const std::string IR_MDD::SOLVER_NAME = "IR_MDD";

IR_MDD::IR_MDD(Problem* _P) : IR(_P)
{
  solver_name = IR_MDD::SOLVER_NAME;
}

void IR_MDD::refinePlan()
{
  updatePlanFocusOneAgent(updateByMDD);
}

void IR_MDD::printHelp()
{
  std::cout
    << IR_MDD::SOLVER_NAME << "\n"
    << "  (no option)"
    << std::endl;
}
