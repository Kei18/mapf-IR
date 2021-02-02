#include <winpibt.hpp>

#include "gtest/gtest.h"

TEST(winPIBT, solve)
{
  Problem P = Problem("../tests/instances/example.txt");
  std::unique_ptr<Solver> solver = std::make_unique<winPIBT>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}
