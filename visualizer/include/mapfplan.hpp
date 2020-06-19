#pragma once
#include "../include/graph.hpp"

struct MAPFPlan {
  std::string scen_file;
  int num_agents;
  Graph* G;
  std::string solver;
  bool solved;
  int soc;
  int makespan;
  int comp_time;
  Config config_s;
  Config config_g;
  Configs transitions;
};
