#pragma once
#include "ir.hpp"


class IR_PATHS : public IR {
public:
  static const std::string SOLVER_NAME;

private:
  std::vector<int> CLOSE_GAP;

  Plan refinePlan(const Config& config_s,
                  const Config& config_g,
                  const Plan& old_plan);

  bool find_all_interacting_agents;
  std::vector<int> getDirectInteractingAgents(const Paths& old_paths,
                                              const int id_largest_gap);
  std::vector<int> getAllInteractingAgents(const Paths& old_paths,
                                           const int id_largest_gap);

private:
public:
  IR_PATHS(Problem* _P);
  ~IR_PATHS();

  void setParams(int argc, char *argv[]);
  static void printHelp();
};
