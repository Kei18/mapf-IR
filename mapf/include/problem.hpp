#pragma once
#include <random>

#include "default_params.hpp"
#include "graph.hpp"
#include "util.hpp"

class Problem
{
private:
  std::string instance;  // instance name
  Graph* G;              // graph
  std::mt19937* MT;      // seed
  Config config_s;       // initial configuration
  Config config_g;       // goal configuration
  int num_agents;        // number of agents
  int max_timestep;      // timestep limit
  int max_comp_time;     // comp_time limit, ms

  const bool instance_initialized;  // for memory manage

  // set starts and goals randomly
  void setRandomStartsGoals();

  // set well-formed instance
  void setWellFormedInstance();

public:
  Problem(const std::string& _instance);
  Problem(Problem* P, Config _config_s, Config _config_g, int _max_comp_time,
          int _max_timestep);
  Problem(Problem* P, int _max_comp_time);
  ~Problem();

  Graph* getG() { return G; }
  int getNum() { return num_agents; }
  std::mt19937* getMT() { return MT; }
  Node* getStart(int i) const;  // return start of a_i
  Node* getGoal(int i) const;   // return  goal of a_i
  Config getConfigStart() const { return config_s; };
  Config getConfigGoal() const { return config_g; };
  int getMaxTimestep() { return max_timestep; };
  int getMaxCompTime() { return max_comp_time; };
  std::string getInstanceFileName() { return instance; };

  // used when making new instance file
  void makeScenFile(const std::string& output_file);
};
