#include "../include/ir_bottleneck.hpp"

const std::string IR_Bottleneck::SOLVER_NAME = "IR_BOTTLENECK";

IR_Bottleneck::IR_Bottleneck(Problem* _P)
  : IR_FOCUS_ONE_AGENT(_P)
{
  solver_name = IR_Bottleneck::SOLVER_NAME;
}

void IR_Bottleneck::updatePlanFocusOneAgent(const int i, Plan& plan)
{
  const auto modif_list = std::get<1>(LibIR::identifyBottleneckAgentsWithScore
                                      (i, plan, this, getRefineTimeLimit()));
  if (modif_list.empty()) return;
  Problem _P = Problem(P, getRefineTimeLimit());
  plan = std::get<1>(getOptimalPlan(&_P, plan, modif_list));
  updateSolution(plan);
}

void IR_Bottleneck::printHelp()
{
  std::cout
    << IR_Bottleneck::SOLVER_NAME << "\n"
    << "  (no option)"
    << std::endl;
}
