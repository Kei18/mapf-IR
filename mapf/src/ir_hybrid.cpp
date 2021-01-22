#include "../include/ir_hybrid.hpp"

const std::string IR_HYBRID::SOLVER_NAME = "IR_HYBRID";

IR_HYBRID::IR_HYBRID(Problem* _P) : IR(_P)
{
  solver_name = IR_HYBRID::SOLVER_NAME;
}

void IR_HYBRID::refinePlan()
{
  Plan plan = solution;
  int last_itr_soc = plan.getSOC();

  // fix at goal
  do {
    last_itr_soc = plan.getSOC();
    for (int i = 0; i < P->getNum(); ++i) {
      if (overCompTime() || current_iteration >= max_iteration) break;
      if (plan.getPathCost(i) - pathDist(i) == 0) continue;
      plan = LibIR::refineTwoPathsAtGoal(i, plan, P, getRefineTimeLimit());
      updateSolution(plan);
    }
  } while (last_itr_soc != plan.getSOC() && !overCompTime() && current_iteration < max_iteration);

  // single paths
  do {
    last_itr_soc = plan.getSOC();
    for (int i = 0; i < P->getNum(); ++i) {
      if (overCompTime() || current_iteration >= max_iteration) break;
      if (plan.getPathCost(i) - pathDist(i) == 0) continue;
      plan = LibIR::refineSinglePath(i, plan, P, getRefineTimeLimit());
      updateSolution(plan);
    }
  } while (last_itr_soc != plan.getSOC() && !overCompTime() && current_iteration < max_iteration);

  // focus goals
  do {
    last_itr_soc = plan.getSOC();
    for (int i = 0; i < P->getNum(); ++i) {
      if (overCompTime() || current_iteration >= max_iteration) break;
      if (plan.getPathCost(i) - pathDist(i) == 0) continue;
      const auto modif_list = LibIR::identifyAgentsAtGoal(i, plan, P->getGoal(i), pathDist(i));
      if (modif_list.empty()) continue;
      Problem _P = Problem(P, getRefineTimeLimit());
      plan = std::get<1>(getOptimalPlan(&_P, plan, modif_list));
      updateSolution(plan);
    }
  } while (last_itr_soc != plan.getSOC() && !overCompTime() && current_iteration < max_iteration);

  // mdd
  do {
    last_itr_soc = plan.getSOC();
    for (int i = 0; i < P->getNum(); ++i) {
      if (overCompTime() || current_iteration >= max_iteration) break;
      if (plan.getPathCost(i) - pathDist(i) == 0) continue;
      const auto modif_list =
        LibIR::identifyInteractingSetByMDD(i, plan, P, true, getRefineTimeLimit(), MT);
      if (modif_list.empty()) continue;
      Problem _P = Problem(P, getRefineTimeLimit());
      plan = std::get<1>(getOptimalPlan(&_P, plan, modif_list));
      updateSolution(plan);
    }
  } while (last_itr_soc != plan.getSOC() && !overCompTime() && current_iteration < max_iteration);

  // random
  std::vector<int> A(P->getNum());
  std::iota(A.begin(), A.end(), 0);
  while (!overCompTime() && current_iteration < max_iteration) {
    std::shuffle(A.begin(), A.end(), *MT);
    std::vector<int> modif_list(sampling_num);
    std::copy(A.begin(), A.begin() + sampling_num, modif_list.begin());
    Problem _P = Problem(P, getRefineTimeLimit());
    plan = std::get<1>(getOptimalPlan(&_P, plan, modif_list));
    updateSolution(plan);
  }
}

void IR_HYBRID::printHelp()
{
  std::cout
    << IR_HYBRID::SOLVER_NAME << "\n"
    << "  (no option)"
    << std::endl;
}
