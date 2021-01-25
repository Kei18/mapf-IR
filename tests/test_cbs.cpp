#include <cbs.hpp>

#include "gtest/gtest.h"

TEST(CBS, solve)
{
  Problem P = Problem("../tests/instances/example.txt");
  std::unique_ptr<Solver> solver = std::make_unique<CBS>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
  ASSERT_EQ(solver->getSolution().getSOC(), 635);
}
