#include <ir.hpp>

#include "gtest/gtest.h"

TEST(LibIR, identifyInteractingSetByMDD)
{
  Problem P = Problem("../tests/instances/libir_mdd.txt");
  auto G = P.getG();
  Solver solver = IR(&P);
  solver.createDistanceTable();

  Config starts = P.getConfigStart();
  Config goals = P.getConfigGoal();
  Plan plan;
  plan.add(starts);
  plan.add({G->getNode(0, 1), G->getNode(1, 1), G->getNode(2, 1)});
  plan.add({G->getNode(1, 1), G->getNode(1, 2), G->getNode(2, 2)});
  plan.add(goals);
  ASSERT_TRUE(plan.validate(starts, goals));

  auto modif_list1 = IR::identifyInteractingSetByMDD(0, plan, &solver);
  ASSERT_EQ(modif_list1.size(), 2);
  auto modif_list2 = IR::identifyInteractingSetByMDD(1, plan, &solver);
  ASSERT_EQ(modif_list2.size(), 0);
}

TEST(libIR, identifyInteractingSetByMDD_Advanced)
{
  Problem P = Problem("../tests/instances/libir_mdd_advanced.txt");
  auto G = P.getG();
  Solver solver = IR(&P);
  solver.createDistanceTable();

  Config starts = P.getConfigStart();
  Config goals = P.getConfigGoal();
  Plan plan;
  plan.add({G->getNode(0, 3), G->getNode(1, 2), G->getNode(2, 0)});
  plan.add({G->getNode(0, 3), G->getNode(1, 3), G->getNode(2, 1)});
  plan.add({G->getNode(1, 3), G->getNode(1, 4), G->getNode(2, 2)});
  plan.add({G->getNode(1, 3), G->getNode(1, 4), G->getNode(2, 3)});
  plan.add({G->getNode(2, 3), G->getNode(1, 4), G->getNode(2, 4)});
  plan.add({G->getNode(3, 3), G->getNode(1, 4), G->getNode(2, 4)});
  ASSERT_TRUE(plan.validate(starts, goals));

  auto modif_list1 = IR::identifyInteractingSetByMDD(0, plan, &solver);
  ASSERT_EQ(modif_list1.size(), 2);
  auto modif_list2 = IR::identifyInteractingSetByMDD(0, plan, &solver, true);
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

  Config starts = {c, a};
  Config goals = {d, e};
  Plan plan;
  plan.add({c, a});
  plan.add({d, b});
  plan.add({d, c});
  plan.add({x, d});
  plan.add({d, e});
  ASSERT_TRUE(plan.validate(starts, goals));

  auto modif_list1 = IR::identifyAgentsAtGoal(0, plan, d, G.pathDist(c, d));
  ASSERT_EQ(modif_list1.size(), 2);
  auto modif_list2 = IR::identifyAgentsAtGoal(1, plan, e, G.pathDist(a, e));
  ASSERT_EQ(modif_list2.size(), 0);
}

TEST(libIR, identifyBottleneckAgents)
{
  Problem P = Problem("../tests/instances/libir_bottleneck.txt");
  auto G = P.getG();
  Solver solver = IR(&P);
  solver.createDistanceTable();

  const auto starts = P.getConfigStart();
  const auto goals = P.getConfigGoal();

  Plan plan;
  plan.add(starts);
  plan.add({G->getNode(0, 3), G->getNode(0, 1)});
  plan.add({G->getNode(0, 3), G->getNode(0, 2)});
  plan.add({G->getNode(1, 3), G->getNode(0, 3)});
  plan.add(goals);
  ASSERT_TRUE(plan.validate(starts, goals));

  auto res0 = IR::identifyBottleneckAgentsWithScore(0, Solver::planToPaths(plan), &solver);
  ASSERT_EQ(std::get<0>(res0), 0);
  ASSERT_EQ(std::get<1>(res0).size(), 0);
  auto res1 = IR::identifyBottleneckAgentsWithScore(1, Solver::planToPaths(plan), &solver);
  ASSERT_EQ(std::get<0>(res1), 3);
  ASSERT_EQ(std::get<1>(res1).size(), 2);
}

TEST(IR, solve)
{
  Problem P = Problem("../tests/instances/example.txt");
  std::unique_ptr<Solver> solver = std::make_unique<IR>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}

TEST(IR_SINGLE_PATHS, solve)
{
  Problem P = Problem("../tests/instances/ir_single_paths.txt");
  auto G = P.getG();
  auto solver = IR_SINGLE_PATHS(&P);

  Plan plan;
  plan.add({G->getNode(0, 0), G->getNode(2, 0)});
  plan.add({G->getNode(0, 0), G->getNode(3, 0)});
  plan.add({G->getNode(1, 0), G->getNode(3, 0)});
  ASSERT_TRUE(plan.validate(P.getConfigStart(), P.getConfigGoal()));
  ASSERT_EQ(plan.getSOC(), 3);

  solver.setInitialPlan(plan);
  solver.solve();

  ASSERT_TRUE(solver.succeed());
  ASSERT_TRUE(solver.getSolution().validate(&P));
  ASSERT_EQ(solver.getSolution().getSOC(), 2);
}

TEST(IR_FIX_AT_GOALS, solve)
{
  Problem P = Problem("../tests/instances/ir_fix_at_goals.txt");
  auto G = P.getG();
  auto solver = IR_FIX_AT_GOALS(&P);

  Plan plan;
  plan.add({G->getNode(2, 0), G->getNode(0, 0)});
  plan.add({G->getNode(3, 0), G->getNode(1, 0)});
  plan.add({G->getNode(3, 0), G->getNode(2, 0)});
  plan.add({G->getNode(3, 1), G->getNode(3, 0)});
  plan.add({G->getNode(3, 0), G->getNode(4, 0)});
  ASSERT_TRUE(plan.validate(P.getConfigStart(), P.getConfigGoal()));
  ASSERT_EQ(plan.getSOC(), 8);
  ASSERT_EQ(plan.getMakespan(), 4);

  solver.setInitialPlan(plan);
  solver.solve();

  auto solution = solver.getSolution();
  ASSERT_TRUE(solver.succeed());
  ASSERT_TRUE(solution.validate(&P));
  ASSERT_EQ(solution.getSOC(), 7);
  ASSERT_EQ(solution.getMakespan(), 6);
}

TEST(IR_FOCUS_GOALS, solve)
{
  Problem P = Problem("../tests/instances/example.txt");
  std::unique_ptr<Solver> solver = std::make_unique<IR_FOCUS_GOALS>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}

TEST(IR_MDD, solve)
{
  Problem P = Problem("../tests/instances/example.txt");
  std::unique_ptr<Solver> solver = std::make_unique<IR_MDD>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}

TEST(IR_BOTTLENECK, solve)
{
  Problem P = Problem("../tests/instances/example.txt");
  std::unique_ptr<Solver> solver = std::make_unique<IR_BOTTLENECK>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}
TEST(IR_HYBRID, solve)
{
  Problem P = Problem("../tests/instances/example.txt");
  std::unique_ptr<Solver> solver = std::make_unique<IR_HYBRID>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}
