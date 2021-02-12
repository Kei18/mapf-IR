#include <lib_solver.hpp>

#include "gtest/gtest.h"

TEST(planToPaths, convert)
{
  Grid G("8x8.map");
  Node* v = G.getNode(0);
  Node* u = G.getNode(1);
  Node* w = G.getNode(2);
  Node* x = G.getNode(3);

  Plan plan1;
  plan1.add({ v, w });
  plan1.add({ u, x });

  Paths paths = planToPaths(plan1);
  ASSERT_EQ(paths.get(0, 0), v);
  ASSERT_EQ(paths.get(0, 1), u);
  ASSERT_EQ(paths.get(1, 0), w);
  ASSERT_EQ(paths.get(1, 1), x);

  Plan plan2 = pathsToPlan(paths);
  ASSERT_EQ(plan2.get(0, 0), v);
  ASSERT_EQ(plan2.get(0, 1), w);
  ASSERT_EQ(plan2.get(1, 0), u);
  ASSERT_EQ(plan2.get(1, 1), x);
}
