#include "../include/ir_bottleneck.hpp"

const std::string IR_Bottleneck::SOLVER_NAME = "IR_Bottleneck";

IR_Bottleneck::IR_Bottleneck(Problem* _P)
  : IR(_P)
{
  solver_name = IR_Bottleneck::SOLVER_NAME;
}

void IR_Bottleneck::refinePlan()
{
  Plan plan = solution;

  while (current_iteration < max_iteration && !overCompTime()) {
    for (int i = 0; i < P->getNum(); ++i) {
      if (overCompTime()) break;

      // identify modification list
      const auto modif_list = std::get<1>(LibIR::identifyBottleneckAgentsWithScore
                                          (i, plan, P, getRefineTimeLimit()));
      if (modif_list.empty()) continue;

      Problem _P = Problem(P, getRefineTimeLimit());
      auto res = getOptimalPlan(&_P, plan, modif_list);
      plan = std::get<1>(res);
      updateSolution(plan);

      if (current_iteration >= max_iteration) break;
    }
  }
}

void IR_Bottleneck::printHelp()
{
  std::cout
    << IR_Bottleneck::SOLVER_NAME << "\n"
    << "  (no option)"
    << std::endl;
}
