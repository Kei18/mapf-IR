#include <lib_ir.hpp>

#include "gtest/gtest.h"

TEST(LibIR, refineSinglePaths)
{
  Grid G("8x8.map");

  Node* v = G.getNode(0);
  Node* u = G.getNode(1);
  Node* w = G.getNode(2);
  Node* x = G.getNode(3);

  Config starts = { v, w };
  Config goals  = { u, x };

  Plan plan;
  plan.add(starts);
  plan.add({ v, x });
  plan.add(goals);
  ASSERT_TRUE(plan.validate(starts, goals));
  ASSERT_EQ(plan.getSOC(), 3);

  Plan refined_plan = LibIR::refineSinglePaths(plan, &G, starts, goals, 1000);
  ASSERT_TRUE(plan.validate(starts, goals));
  ASSERT_EQ(refined_plan.getSOC(), 2);
}

TEST(LibIR, refineTwoPathsAtGoal)
{
  Grid G("8x8.map");

  Node* a = G.getNode(0);
  Node* b = G.getNode(1);
  Node* c = G.getNode(2);
  Node* d = G.getNode(3);
  Node* e = G.getNode(4);
  Node* x = G.getNode(3, 1);

  Config starts = { c, a };
  Config goals  = { d, e };
  Plan plan;
  plan.add({ c, a });
  plan.add({ d, b });
  plan.add({ d, c });
  plan.add({ x, d });
  plan.add({ d, e });
  ASSERT_TRUE(plan.validate(starts, goals));
  ASSERT_EQ(plan.getSOC(), 8);
  ASSERT_EQ(plan.getMakespan(), 4);

  Plan refined_plan = LibIR::refineTwoPathsAtGoal(plan, &G, starts, goals, 1000);
  ASSERT_TRUE(refined_plan.validate(starts, goals));
  ASSERT_EQ(refined_plan.getSOC(), 7);
  ASSERT_EQ(refined_plan.getMakespan(), 6);
}

TEST(LibIR, refineTwoPathsAtGoal_challenging)
{
  Grid G("8x8.map");

  Node* a = G.getNode(0);
  Node* b = G.getNode(1);
  Node* c = G.getNode(2);
  Node* d = G.getNode(3);
  Node* e = G.getNode(4);
  Node* f = G.getNode(5);
  Node* g = G.getNode(6);
  Node* h = G.getNode(7);
  Node* x = G.getNode(6, 1);
  Node* y = G.getNode(7, 1);

  Config starts = { f, d, a };
  Config goals  = { g, y, x };
  Plan plan;
  plan.add({ f, d, a });
  plan.add({ g, e, b });
  plan.add({ g, f, c });
  plan.add({ x, g, d });
  plan.add({ g, h, e });
  plan.add({ g, y, f });
  plan.add({ h, y, g });
  plan.add({ g, y, x });
  ASSERT_TRUE(plan.validate(starts, goals));
  ASSERT_EQ(plan.getSOC(), 19);
  ASSERT_EQ(plan.getMakespan(), 7);

  Plan refined_plan = LibIR::refineTwoPathsAtGoal(plan, &G, starts, goals, 1000);
  ASSERT_TRUE(refined_plan.validate(starts, goals));
  ASSERT_EQ(refined_plan.getSOC(), 13);
  ASSERT_EQ(refined_plan.getMakespan(), 7);
}

TEST(LibIR, identifyInteractingSetByMDD)
{
  Grid G("8x8.map");

  Node* s1 = G.getNode(0, 1);
  Node* s2 = G.getNode(1, 0);
  Node* s3 = G.getNode(2, 0);
  Node* g1 = G.getNode(2, 1);
  Node* g2 = G.getNode(0, 2);
  Node* g3 = G.getNode(2, 2);

  Config starts = { s1, s2, s3 };
  Config goals =  { g1, g2, g3 };
  Plan plan;
  plan.add({ s1, s2, s3 });
  plan.add({ G.getNode(0, 1), G.getNode(1, 1), G.getNode(2, 1) });
  plan.add({ G.getNode(1, 1), G.getNode(1, 2), G.getNode(2, 2) });
  plan.add({ g1, g2, g3 });
  ASSERT_TRUE(plan.validate(starts, goals));

  auto modif_list1 = LibIR::identifyInteractingSetByMDD(0, plan, &G, s1, g1);
  ASSERT_EQ(modif_list1.size(), 2);
  auto modif_list2 = LibIR::identifyInteractingSetByMDD(1, plan, &G, s2, g2);
  ASSERT_EQ(modif_list2.size(), 0);
}

TEST(libIR, identifyInteractingSetByMDD_Advanced)
{
  Grid G("8x8.map");

  Config starts = { G.getNode(0,3), G.getNode(1,2), G.getNode(2,0) };
  Config goals  = { G.getNode(3,3), G.getNode(1,4), G.getNode(2,4) };
  Plan plan;
  plan.add({ G.getNode(0,3), G.getNode(1,2), G.getNode(2,0) });
  plan.add({ G.getNode(0,3), G.getNode(1,3), G.getNode(2,1) });
  plan.add({ G.getNode(1,3), G.getNode(1,4), G.getNode(2,2) });
  plan.add({ G.getNode(1,3), G.getNode(1,4), G.getNode(2,3) });
  plan.add({ G.getNode(2,3), G.getNode(1,4), G.getNode(2,4) });
  plan.add({ G.getNode(3,3), G.getNode(1,4), G.getNode(2,4) });
  ASSERT_TRUE(plan.validate(starts, goals));

  auto modif_list1 = LibIR::identifyInteractingSetByMDD
    (0, plan, &G, G.getNode(0,3), G.getNode(3,3));
  ASSERT_EQ(modif_list1.size(), 2);
  auto modif_list2 = LibIR::identifyInteractingSetByMDD
    (0, plan, &G, G.getNode(0,3), G.getNode(3,3), true);
  ASSERT_EQ(modif_list2.size(), 3);
}

TEST(libIR, identifyInteractingSetByMDDAggressive)
{
  Grid G("8x8.map");

  Config starts = { G.getNode(2,0), G.getNode(1,1), G.getNode(0,2) };
  Config goals  = { G.getNode(2,3), G.getNode(3,1), G.getNode(3,2) };
  Plan plan;
  plan.add({ G.getNode(2,0), G.getNode(1,1), G.getNode(0,2) });
  plan.add({ G.getNode(2,1), G.getNode(1,1), G.getNode(1,2) });
  plan.add({ G.getNode(2,1), G.getNode(1,1), G.getNode(2,2) });
  plan.add({ G.getNode(2,2), G.getNode(2,1), G.getNode(3,2) });
  plan.add({ G.getNode(2,3), G.getNode(3,1), G.getNode(3,2) });

  ASSERT_TRUE(plan.validate(starts, goals));

  auto modif_list1 = LibIR::identifyInteractingSetByMDD
    (1, plan, &G, G.getNode(1,1), G.getNode(3,1));
  ASSERT_EQ(modif_list1.size(), 2);
  auto modif_list2 = LibIR::identifyInteractingSetByMDDAggressive
    (1, plan, &G, starts, goals);
  ASSERT_EQ(modif_list2.size(), 3);
}

TEST(libIR, identifyAgentsAtGoal)
{
  Grid G("8x8.map");

  Node* a = G.getNode(0);
  Node* b = G.getNode(1);
  Node* c = G.getNode(2);
  Node* d = G.getNode(3);
  Node* e = G.getNode(4);
  Node* x = G.getNode(3, 1);

  Config starts = { c, a };
  Config goals  = { d, e };
  Plan plan;
  plan.add({ c, a });
  plan.add({ d, b });
  plan.add({ d, c });
  plan.add({ x, d });
  plan.add({ d, e });
  ASSERT_TRUE(plan.validate(starts, goals));

  auto modif_list1 = LibIR::identifyAgentsAtGoal(0, plan, &G, c, d);
  ASSERT_EQ(modif_list1.size(), 2);
  auto modif_list2 = LibIR::identifyAgentsAtGoal(1, plan, &G, a, e);
  ASSERT_EQ(modif_list2.size(), 0);
}

TEST(libIR, identifyBottleneckAgents)
{
  Grid G("8x8.map");

  Node* a = G.getNode(0);
  Node* b = G.getNode(1);
  Node* c = G.getNode(2);
  Node* d = G.getNode(3);
  Node* e = G.getNode(4);
  Node* x = G.getNode(3, 1);

  Config starts = { c, a };
  Config goals  = { d, e };
  Plan plan;
  plan.add({ c, a });
  plan.add({ d, b });
  plan.add({ d, c });
  plan.add({ x, d });
  plan.add({ d, e });
  ASSERT_TRUE(plan.validate(starts, goals));

  auto res0 = LibIR::identifyBottleneckAgentsWithScore(0, plan, &G, starts, goals);
  ASSERT_EQ(std::get<0>(res0), 0);
  ASSERT_EQ(std::get<1>(res0).size(), 0);
  auto res1 = LibIR::identifyBottleneckAgentsWithScore(1, plan, &G, starts, goals);
  ASSERT_EQ(std::get<0>(res1), 3);
  ASSERT_EQ(std::get<1>(res1).size(), 2);
}
