#include <plan.hpp>
#include <graph.hpp>

#include "gtest/gtest.h"

TEST(Plan, basic)
{
  Grid G("8x8.map");
  Node* v = G.getNode(0);
  Node* u = G.getNode(1);
  Node* w = G.getNode(2);

  Config c0 = { v, u };
  Config c1 = { v, w };

  Plan plan;

  ASSERT_TRUE(plan.empty());

  plan.add(c0);
  ASSERT_EQ(plan.getMakespan(), 0);
  ASSERT_EQ(plan.getPathCost(0), 0);
  ASSERT_EQ(plan.getPathCost(1), 0);
  ASSERT_EQ(plan.getSOC(), 0);
  ASSERT_EQ(plan.size(), 1);

  plan.add(c1);
  ASSERT_EQ(plan.getMakespan(), 1);
  ASSERT_EQ(plan.getPathCost(0), 0);
  ASSERT_EQ(plan.getPathCost(1), 1);
  ASSERT_EQ(plan.getSOC(), 1);
  ASSERT_EQ(plan.size(), 2);

  ASSERT_EQ(plan.get(0)[0], v);
  ASSERT_EQ(plan.get(1)[1], w);

  ASSERT_EQ(plan.last()[1], w);

  Path path = plan.getPath(1);
  ASSERT_EQ(path[0], u);
  ASSERT_EQ(path[1], w);
}

TEST(Plan, add)
{
  Grid G("8x8.map");
  Node* v = G.getNode(0);
  Node* u = G.getNode(1);
  Node* w = G.getNode(2);
  Node* x = G.getNode(3);

  Config c1_0 = { v, u };
  Config c1_1 = { v, w };
  Config c2_0 = { v, w };
  Config c2_1 = { v, x };

  Plan plan1;
  plan1.add(c1_0);
  plan1.add(c1_1);

  Plan plan2;
  plan2.add(c2_0);
  plan2.add(c2_1);

  Plan plan3 = plan1 + plan2;
  ASSERT_EQ(plan3.getMakespan(), 2);
  ASSERT_EQ(plan3.getSOC(), 2);

  ASSERT_EQ(plan1.getMakespan(), 1);
  plan1 += plan2;
  ASSERT_EQ(plan1.getMakespan(), 2);
}

TEST(Plan, validate)
{
  Grid G("8x8.map");
  Node* v = G.getNode(0);
  Node* u = G.getNode(1);
  Node* w = G.getNode(2);

  // normal
  Plan plan0;
  plan0.add({ v, u });
  plan0.add({ u, w });
  ASSERT_TRUE(plan0.validate({ v, u }, { u, w }));

  // different starts
  Plan plan1;
  plan1.add({ v, w });
  plan1.add({ u, w });
  ASSERT_FALSE(plan1.validate({ v, u }, { u, w }));

  // different goals
  Plan plan2;
  plan2.add({ v, u });
  plan2.add({ v, w });
  ASSERT_FALSE(plan2.validate({ v, u }, { u, w }));

  // vertex conflict
  Plan plan3;
  plan3.add({ v, u });
  plan3.add({ u, u });
  plan3.add({ u, w });
  ASSERT_FALSE(plan3.validate({ v, u }, { u, w }));

  // swap conflict
  Plan plan4;
  plan4.add({ v, u });
  plan4.add({ u, v });
  ASSERT_FALSE(plan4.validate({ v, u }, { u, w }));

  // invalid move
  Plan plan5;
  plan5.add({ v });
  plan5.add({ w });
  ASSERT_FALSE(plan5.validate({ v }, { w }));
}

TEST(Plan, maxConstraintTime)
{
  Grid G("8x8.map");
  Node* v = G.getNode(0);
  Node* u = G.getNode(1);
  Node* w = G.getNode(2);

  Plan plan;
  plan.add({v, u});
  plan.add({v, u});
  plan.add({u, w});
  ASSERT_EQ(plan.getMaxConstraintTime(0, v, u, &G), 1);
  ASSERT_EQ(plan.getMaxConstraintTime(1, u, w, &G), 0);
}
