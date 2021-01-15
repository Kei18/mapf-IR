#include "../include/ir_fix_at_goals.hpp"

const std::string IR_FixAtGoals::SOLVER_NAME = "IR_FixAtGoals";

IR_FixAtGoals::IR_FixAtGoals(Problem* _P)
  : IR(_P)
{
  solver_name = IR_FixAtGoals::SOLVER_NAME;
}

void IR_FixAtGoals::refinePlan()
{
  Plan plan = solution;

  while (current_iteration < max_iteration) {

    // calculate gaps
    std::vector<int> gaps;
    for (int i = 0; i < P->getNum(); ++i) {
      gaps.push_back(plan.getPathCost(i) - pathDist(i));
    }

    // sort in the increasing order
    std::vector<int> A(P->getNum());
    std::iota(A.begin(), A.end(), 0);
    std::sort(A.begin(), A.end(), [&] (int i, int j) { return gaps[i] > gaps[j]; });

    for (auto i : A) {
      if (overCompTime()) break;
      plan = LibIR::refineTwoPathsAtGoal(i, plan, P, getRefineTimeLimit());
      updateSolution(plan);

      if (current_iteration >= max_iteration) break;
    }
  }
}

void IR_FixAtGoals::printHelp()
{
  std::cout
    << IR_FixAtGoals::SOLVER_NAME << "\n"
    << "  (no option)"
    << std::endl;
}
