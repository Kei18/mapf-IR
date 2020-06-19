#pragma once
#include "graph.hpp"
#include <random>

class Problem {
private:
  std::string instance;
  Graph* G;
  std::mt19937* MT;
  Config config_s;
  Config config_g;
  int num_agents;
  int max_timestep;   // timestep
  int max_comp_time;  // msec

  void setRandomStartsGoals ();
  void setScenStartsGoals(const std::string& scen_file);

public:
  Problem(const std::string& _instance);
  ~Problem() {};

  Graph* getG() { return G; }
  int getNum() { return num_agents; }
  std::mt19937* getMT() { return MT; }
  Node* getStart(int i) const;
  Node* getGoal(int i) const;
  int getMaxTimestep() { return max_timestep; };
  int getMaxCompTime() { return max_comp_time; };
  std::string getInstanceFileName() { return instance; };
};
