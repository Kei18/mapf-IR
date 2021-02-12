#include <ir_single_paths.hpp>

#include "gtest/gtest.h"

TEST(IR_SINGLE_PATHS, solve)
{
  Problem P = Problem("../tests/instances/ir_single_paths.txt");
  auto G = P.getG();
  auto solver = IR_SINGLE_PATHS(&P);

  Plan plan;
  plan.add({ G->getNode(0, 0), G->getNode(2, 0) });
  plan.add({ G->getNode(0, 0), G->getNode(3, 0) });
  plan.add({ G->getNode(1, 0), G->getNode(3, 0) });
  ASSERT_TRUE(plan.validate(P.getConfigStart(), P.getConfigGoal()));
  ASSERT_EQ(plan.getSOC(), 3);

  solver.setInitialPlan(plan);
  solver.solve();

  ASSERT_TRUE(solver.succeed());
  ASSERT_TRUE(solver.getSolution().validate(&P));
  ASSERT_EQ(solver.getSolution().getSOC(), 2);
}
