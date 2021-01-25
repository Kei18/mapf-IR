#include <icbs.hpp>

#include "gtest/gtest.h"

TEST(ICBS, solve)
{
  Problem P = Problem("../tests/instances/example.txt");
  std::unique_ptr<Solver> solver = std::make_unique<ICBS>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
  ASSERT_EQ(solver->getSolution().getSOC(), 635);
}
