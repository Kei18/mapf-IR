#include <paths.hpp>

#include "gtest/gtest.h"

TEST(Paths, basic)
{
  Grid G("8x8.map");
  Node* v = G.getNode(0);
  Node* u = G.getNode(1);
  Node* w = G.getNode(2);

  // format
  Paths paths(2);
  paths.insert(0, {v});
  ASSERT_EQ(paths.get(0).size(), 1);
  ASSERT_TRUE(paths.empty(1));
  paths.insert(1, {u, w});
  ASSERT_EQ(paths.get(0).size(), 2);
  ASSERT_EQ(paths.get(1).size(), 2);
  ASSERT_EQ(paths.getMakespan(), 1);
  ASSERT_EQ(paths.getSOC(), 1);
  ASSERT_EQ(paths.get(0, 0), v);
  ASSERT_EQ(paths.get(0, 1), v);
  ASSERT_EQ(paths.get(1, 0), u);
  ASSERT_EQ(paths.get(1, 1), w);
  ASSERT_FALSE(paths.empty(1));

  // shrink
  paths.insert(1, {u});
  ASSERT_EQ(paths.get(0).size(), 1);
  ASSERT_EQ(paths.getMakespan(), 0);
  ASSERT_EQ(paths.getSOC(), 0);
}

TEST(Paths, add)
{
  Grid G("8x8.map");
  Node* v = G.getNode(0);
  Node* u = G.getNode(1);
  Node* w = G.getNode(2);
  Node* x = G.getNode(3);

  Path path_1_0 = {v, u};
  Path path_1_1 = {u, w};
  Path path_2_0 = {u, w};
  Path path_2_1 = {w, x};

  Paths paths1(2);
  paths1.insert(0, path_1_0);
  paths1.insert(1, path_1_1);

  Paths paths2(2);
  paths2.insert(0, path_2_0);
  paths2.insert(1, path_2_1);

  ASSERT_EQ(paths1.getMakespan(), 1);
  paths1 += paths2;
  ASSERT_EQ(paths1.getMakespan(), 2);
}

TEST(Paths, conflict)
{
  Grid G("8x8.map");
  Node* v = G.getNode(0);
  Node* u = G.getNode(1);
  Node* w = G.getNode(2);
  Node* x = G.getNode(3);

  // no conflict
  Paths paths0(2);
  paths0.insert(0, {v, u, w});
  paths0.insert(1, {u, w, x});
  ASSERT_EQ(paths0.countConflict(), 0);

  // vertex conflict
  Paths paths1(2);
  paths1.insert(0, {v, u, u});
  paths1.insert(1, {u, u, w});
  ASSERT_EQ(paths1.countConflict(), 1);

  // swap conflict
  Paths paths2(2);
  paths2.insert(0, {v, u});
  paths2.insert(1, {u, v});
  ASSERT_EQ(paths2.countConflict(), 1);

  // three agents
  Paths paths3(3);
  paths3.insert(0, {v, u, w});
  paths3.insert(1, {u, v, v});
  paths3.insert(2, {w, w, w});
  ASSERT_EQ(paths3.countConflict(), 2);

  // insert agents
  Paths paths4(3);
  paths4.insert(0, {v, u, w});
  paths4.insert(1, {u, v, v});
  paths4.insert(2, {w, w, w});
  ASSERT_EQ(paths4.countConflict(2, {w, w, u}), 1);
}
