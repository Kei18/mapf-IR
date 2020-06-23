#pragma once
#include "graph.hpp"
#include "default_params.hpp"
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
  Problem(Problem* P,
          Config _config_s,
          Config _config_g,
          int _max_comp_time,
          int _max_timestep);
  ~Problem() {};

  Graph* getG() { return G; }
  int getNum() { return num_agents; }
  std::mt19937* getMT() { return MT; }
  Node* getStart(int i) const;
  Node* getGoal(int i) const;
  Config getConfigStart() const { return config_s; };
  Config getConfigGoal() const { return config_g; };
  int getMaxTimestep() { return max_timestep; };
  int getMaxCompTime() { return max_comp_time; };
  std::string getInstanceFileName() { return instance; };
};
