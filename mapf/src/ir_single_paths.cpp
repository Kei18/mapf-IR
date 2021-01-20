#include "../include/ir_single_paths.hpp"

const std::string IR_SinglePaths::SOLVER_NAME = "IR_SINGLE_PATHS";

IR_SinglePaths::IR_SinglePaths(Problem* _P)
  : IR_FOCUS_ONE_AGENT(_P)
{
  solver_name = IR_SinglePaths::SOLVER_NAME;
}

void IR_SinglePaths::updatePlanFocusOneAgent(const int i, Plan& plan)
{
  plan = LibIR::refineSinglePath(i, plan, P, getRefineTimeLimit());
  updateSolution(plan);
}

void IR_SinglePaths::printHelp()
{
  std::cout
    << IR_SinglePaths::SOLVER_NAME << "\n"
    << "  (no option)"
    << std::endl;
}
