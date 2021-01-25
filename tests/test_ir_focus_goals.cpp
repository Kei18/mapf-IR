#include <ir_focus_goals.hpp>

#include "gtest/gtest.h"

TEST(IR_FOCUS_GOALS, solve)
{
  Problem P = Problem("../tests/instances/example.txt");
  std::unique_ptr<Solver> solver = std::make_unique<IR_FOCUS_GOALS>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}
