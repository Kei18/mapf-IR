#include <pibt_complete.hpp>

#include "gtest/gtest.h"

TEST(PIBT_COMPLETE, solve)
{
  Problem P = Problem("../tests/instances/02.txt");
  std::unique_ptr<Solver> solver = std::make_unique<PIBT_COMPLETE>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}
