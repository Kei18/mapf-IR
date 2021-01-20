#include "../include/ir_single_paths.hpp"

const std::string IR_SinglePaths::SOLVER_NAME = "IR_SinglePaths";

IR_SinglePaths::IR_SinglePaths(Problem* _P)
  : IR(_P)
{
  solver_name = IR_SinglePaths::SOLVER_NAME;
}

void IR_SinglePaths::refinePlan()
{
  Plan plan = solution;

  while (current_iteration < max_iteration && !overCompTime()) {

    // calculate gaps
    std::vector<int> gaps;
    for (int i = 0; i < P->getNum(); ++i) {
      gaps.push_back(plan.getPathCost(i) - pathDist(i));
    }

    // sort in the increasing order
    std::vector<int> A(P->getNum());
    std::iota(A.begin(), A.end(), 0);
    std::sort(A.begin(), A.end(), [&] (int i, int j) { return gaps[i] > gaps[j]; });

    const int last_itr_soc = last_soc;

    for (auto i : A) {
      if (overCompTime()) break;
      plan = LibIR::refineSinglePath(i, plan, P, getRefineTimeLimit());
      updateSolution(plan);

      if (current_iteration >= max_iteration) break;
    }

    // no update
    if (last_itr_soc == solution.getSOC()) break;
  }
}

void IR_SinglePaths::printHelp()
{
  std::cout
    << IR_SinglePaths::SOLVER_NAME << "\n"
    << "  (no option)"
    << std::endl;
}
