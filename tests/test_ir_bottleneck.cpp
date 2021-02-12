#include <ir_bottleneck.hpp>

#include "gtest/gtest.h"

TEST(IR_BOTTLENECK, solve)
{
  Problem P = Problem("../tests/instances/example.txt");
  std::unique_ptr<Solver> solver = std::make_unique<IR_BOTTLENECK>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}
