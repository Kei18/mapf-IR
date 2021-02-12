#include <ir_fix_at_goals.hpp>

#include "gtest/gtest.h"

TEST(IR_FIX_AT_GOALS, solve)
{
  Problem P = Problem("../tests/instances/ir_fix_at_goals.txt");
  auto G = P.getG();
  auto solver = IR_FIX_AT_GOALS(&P);

  Plan plan;
  plan.add({G->getNode(2, 0), G->getNode(0, 0)});
  plan.add({G->getNode(3, 0), G->getNode(1, 0)});
  plan.add({G->getNode(3, 0), G->getNode(2, 0)});
  plan.add({G->getNode(3, 1), G->getNode(3, 0)});
  plan.add({G->getNode(3, 0), G->getNode(4, 0)});
  ASSERT_TRUE(plan.validate(P.getConfigStart(), P.getConfigGoal()));
  ASSERT_EQ(plan.getSOC(), 8);
  ASSERT_EQ(plan.getMakespan(), 4);

  solver.setInitialPlan(plan);
  solver.solve();

  auto solution = solver.getSolution();
  ASSERT_TRUE(solver.succeed());
  ASSERT_TRUE(solution.validate(&P));
  ASSERT_EQ(solution.getSOC(), 7);
  ASSERT_EQ(solution.getMakespan(), 6);
}
