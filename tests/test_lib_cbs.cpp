#include <graph.hpp>
#include <lib_cbs.hpp>

#include "gtest/gtest.h"

TEST(LibCBS, getFirstConstraints)
{
  auto G = Grid("8x8.map");
  auto paths = Paths(2);
  paths.insert(0, {G.getNode(0), G.getNode(1), G.getNode(2)});
  paths.insert(1, {G.getNode(1), G.getNode(1), G.getNode(2)});
  auto constraints = LibCBS::getFirstConstraints(paths);

  ASSERT_EQ((int)constraints.size(), 2);

  auto c0 = constraints[0];
  ASSERT_EQ(c0->id, 0);
  ASSERT_EQ(c0->t, 1);
  ASSERT_EQ(c0->v->id, 1);

  auto c1 = constraints[1];
  ASSERT_EQ(c1->id, 1);
  ASSERT_EQ(c1->t, 1);
  ASSERT_EQ(c1->v->id, 1);
}

// mdd is difficult to test...
