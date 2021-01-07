/*
 * Implementation of Iterative Refinement (IR)
 */

#pragma once
#include "../include/cbs_refine.hpp"
#include "../include/ecbs.hpp"
#include "../include/hca.hpp"
#include "../include/icbs_refine.hpp"
#include "../include/pibt.hpp"
#include "../include/pibt_complete.hpp"
#include "../include/whca.hpp"
#include "solver.hpp"

class IR : public Solver
{
public:
  static const std::string SOLVER_NAME;

private:
  // init-solver
  enum struct INIT_SOLVER_TYPE { PIBT, HCA, WHCA, ECBS, PIBT_COMPLETE };
  INIT_SOLVER_TYPE init_solver;
  std::vector<std::string> option_init_solver;

  // refine-solver
  enum struct OPTIMAL_SOLVER_TYPE { CBS, CBS_NORMAL, ICBS, ICBS_NORMAL };
  OPTIMAL_SOLVER_TYPE refine_solver;
  std::vector<std::string> option_optimal_solver;

  // max iteration
  int max_iteration;

  std::vector<int> CLOSE;

  // for log
  std::string output_file;
  bool make_log_every_itr;  // true -> create log for every iteration
  std::vector<std::tuple<int, Plan>> HIST;  // elapsed timestep, plan
  std::vector<int> HIST_GAP;
  std::vector<int> HIST_GROUP_SIZE;

  // early stop
  int timeout_refinement;

  // print underlying solver info
  bool verbose_underlying_solver;

  // default params
  static const INIT_SOLVER_TYPE DEFAULT_INIT_SOLVER;
  static const OPTIMAL_SOLVER_TYPE DEFAULT_REFINE_SOLVER;
  static const int DEFAULT_MAX_ITERATION;

  void run();
  Plan getInitialPlan();
  // success?, solution
  std::tuple<bool, Plan> getOptimalPlan(Problem* _P, const Plan& current_plan,
                                        const std::vector<int>& sample);

  // should be defined
  bool stopRefinement();
  Plan refinePlan(const Config& config_s, const Config& config_g,
                  const Plan& current_plan);
  std::vector<int> getInteractingAgents(const Paths& current_paths,
                                        const int id_largest_gap);

public:
  IR(Problem* _P);
  ~IR();

  void makeLog(const std::string& logfile);
  void setParams(int argc, char* argv[]);
  static void printHelp();
};
