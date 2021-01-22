#include "../include/ir_mdd.hpp"

const std::string IR_MDD::SOLVER_NAME = "IR_MDD";

IR_MDD::IR_MDD(Problem* _P)
  : IR_FOCUS_ONE_AGENT(_P)
{
  solver_name = IR_MDD::SOLVER_NAME;
}

void IR_MDD::updatePlanFocusOneAgent(const int i, Plan& plan)
{
  const auto modif_list =
    LibIR::identifyInteractingSetByMDD(i, plan, this, true, getRefineTimeLimit(), MT);
  if (modif_list.empty()) return;
  Problem _P = Problem(P, getRefineTimeLimit());
  plan = std::get<1>(getOptimalPlan(&_P, plan, modif_list));
  updateSolution(plan);
}

void IR_MDD::printHelp()
{
  std::cout
    << IR_MDD::SOLVER_NAME << "\n"
    << "  (no option)"
    << std::endl;
}
