#include "../include/ir_single_paths.hpp"

const std::string IR_SINGLE_PATHS::SOLVER_NAME = "IR_SINGLE_PATHS";

IR_SINGLE_PATHS::IR_SINGLE_PATHS(Problem* _P) : IR(_P)
{
  solver_name = IR_SINGLE_PATHS::SOLVER_NAME;
}

void IR_SINGLE_PATHS::refinePlan()
{
  updatePlanFocusOneAgent(updateBySinglePaths);
}

void IR_SINGLE_PATHS::printHelp()
{
  std::cout << IR_SINGLE_PATHS::SOLVER_NAME << "\n"
            << "  (no option)" << std::endl;
}
