#include "../include/ir_bottleneck.hpp"

const std::string IR_BOTTLENECK::SOLVER_NAME = "IR_BOTTLENECK";

IR_BOTTLENECK::IR_BOTTLENECK(Problem* _P) : IR(_P)
{
  solver_name = IR_BOTTLENECK::SOLVER_NAME;
}

void IR_BOTTLENECK::refinePlan()
{
  updatePlanFocusOneAgent(updateByBottleneck);
}

void IR_BOTTLENECK::printHelp()
{
  std::cout
    << IR_BOTTLENECK::SOLVER_NAME << "\n"
    << "  (no option)"
    << std::endl;
}
