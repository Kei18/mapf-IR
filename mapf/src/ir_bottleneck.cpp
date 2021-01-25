#include "../include/ir_bottleneck.hpp"

const std::string IR_Bottleneck::SOLVER_NAME = "IR_BOTTLENECK";

IR_Bottleneck::IR_Bottleneck(Problem* _P) : IR(_P)
{
  solver_name = IR_Bottleneck::SOLVER_NAME;
}

void IR_Bottleneck::refinePlan()
{
  updatePlanFocusOneAgent(updateByBottleneck);
}

void IR_Bottleneck::printHelp()
{
  std::cout
    << IR_Bottleneck::SOLVER_NAME << "\n"
    << "  (no option)"
    << std::endl;
}
