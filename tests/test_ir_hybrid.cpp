#include <ir_hybrid.hpp>

#include "gtest/gtest.h"

TEST(IR_HYBRID, solve)
{
  Problem P = Problem("../tests/instances/example.txt");
  std::unique_ptr<Solver> solver = std::make_unique<IR_HYBRID>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}
