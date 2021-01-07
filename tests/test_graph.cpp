#include <graph.hpp>
#include <problem.hpp>

#include "gtest/gtest.h"

TEST(Graph, loading)
{
  Grid G("lak105d.map");
  ASSERT_TRUE(G.existNode(0));
  ASSERT_TRUE(G.existNode(455));
  ASSERT_TRUE(G.existNode(0, 0));
  ASSERT_FALSE(G.existNode(5));
  ASSERT_EQ(G.getMapFileName(), "lak105d.map");
  ASSERT_EQ(G.getNode(0)->getDegree(), 2);
  ASSERT_EQ(G.getV().size(), 443);
  ASSERT_EQ(G.getWidth(), 31);
  ASSERT_EQ(G.getHeight(), 25);
}

TEST(Graph, distance)
{
  Grid G("lak105d.map");
  Node* v = G.getNode(0);
  Node* u = G.getNode(455);
  ASSERT_EQ(v->manhattanDist(u), 35);
  ASSERT_TRUE(25 < v->euclideanDist(u) && v->euclideanDist(u) < 26);
  ASSERT_EQ(G.pathDist(v, u), 39);
}

TEST(Graph, large_field)
{
  Grid G("brc202d.map");
  Node* v = G.getNode(216, 138);
  Node* u = G.getNode(203, 303);
  ASSERT_EQ(G.pathDist(v, u), 782);
}

TEST(Graph, huge_field)
{
  Grid G("ost000a.map");
  Node* v = G.getNode(273, 721);
  Node* u = G.getNode(84, 461);
  ASSERT_EQ(G.pathDist(v, u), 863);
}
