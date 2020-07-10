#pragma once
#include "ir.hpp"

class IR_PATHS : public IR {
public:
  static const std::string SOLVER_NAME;

protected:
  // for log
  Ints HIST_GAP;
  Ints HIST_GROUP_SIZE;

  bool stopRefinement();
  virtual Plan refinePlan(const Config& config_s,
                          const Config& config_g,
                          const Plan& old_plan);

  int max_iteration;
  static const int DEFAULT_MAX_ITERATION;

  Ints CLOSE_GAP;
  bool find_all_interacting_agents;

  Ints getDirectInteractingAgents(const Paths& old_paths,
                                  const int id_largest_gap);
  Ints getAllInteractingAgents(const Paths& old_paths,
                               const int id_largest_gap);

public:
  IR_PATHS(Problem* _P);
  ~IR_PATHS();

  void makeLog(const std::string& logfile);
  void setParams(int argc, char *argv[]);
  static void printHelp();
};
